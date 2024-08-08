from sbot import Robot
from sbot import GPIOPinMode
from sbot import AnalogPins
import time
import numpy
from typing import List

robot = Robot()
IO = robot.arduino

#fudge = 0.96
fudge = 1 # for simulator

turnFactor = 0.4 #for simulator

RightMotor = robot.motor_board.motors[0]
LeftMotor = robot.motor_board.motors[1]

# swap motors for sim

RightMotor = robot.motor_board.motors[1]
LeftMotor = robot.motor_board.motors[0]

def frontUltrasound() -> int:
    """
    Returns the distance from the front Ultrasound sensor to an obstacle. 
    """
    return IO.ultrasound_measure(2, 3)

def leftUltrasound() -> int:
    return IO.ultrasound_measure(4, 5)

def rightUltrasound() -> int:
    return IO.ultrasound_measure(6, 7)
    
def setMotors(left: int, right: int):
    RightMotor.power = right# * fudge
    LeftMotor.power = left

# This function checks visible maker list for a specific target
def find_marker(markers: List[int], suitableMarkers: List[int]): #it will now prefer otherID over the id_number
    for i in suitableMarkers:
        for marker in markers:
            if marker.id == i:
                return marker
    return None

def turntowardsMarker(suitableMarkers: List[int]):

    lastError = 0
    totalError = 0
    numRotations = 0

    while True:

        setMotors(0,0)
        robot.sleep(0.4)
        markers = robot.camera.see()
        targetMarker = find_marker(markers, suitableMarkers)

        if targetMarker != None:
            angleToMarker = targetMarker.position.horizontal_angle

            if(abs(angleToMarker) < 0.125):

                #Turn back a tiny bit
                pVal = angleToMarker * 0.5
                iVal = totalError * 0.01
                dVal = (angleToMarker - lastError) * 0.005
                
                # sim values
                
                pVal = angleToMarker * 0.5
                iVal = totalError * 0.01
                dVal = (angleToMarker - lastError) * 0.005

                motorTime = pVal + iVal + dVal #PID!!!! (P seems to be the most useful)
                #setMotors(0.25 * numpy.sign(angleToMarker)*turnFactor, -0.25 * numpy.sign(angleToMarker)*turnFactor)
                robot.sleep(abs(motorTime))
                
                return targetMarker.id
                
            else:
                pVal = angleToMarker * 0.3
                iVal = totalError * 0.025
                dVal = (angleToMarker - lastError) * 0.02
                
                #sim values
                
                pVal = angleToMarker * 0.2
                iVal = totalError * 0.01
                dVal = (angleToMarker - lastError) * 0.01
                if(lastError == 0):
                    pVal = pVal * 1.8

                motorTime = pVal + iVal + dVal #PID!!!! (P seems to be the most useful)
                setMotors(0.25 * numpy.sign(angleToMarker)*turnFactor, -0.25 * numpy.sign(angleToMarker)*turnFactor)
                robot.sleep(abs(motorTime))
                
                lastError = angleToMarker
                totalError += angleToMarker

        else:
            setMotors(-0.25*turnFactor,0.25*turnFactor)
            robot.sleep(0.5)
            if(numRotations <= 15):
                #setMotors(-0.4,-0.4) # surely we don't want to move backwards until we have done a full rotation?
                #robot.sleep(0.5)
                numRotations += 1
            else:
                print("I AM LOST!!!")
                setMotors(-0.5,-0.5)
                robot.sleep(0.5)
                return turntowardsMarker([0,1,2,3,4,5,6,7])


"""
    if targetMarker != None:
        angleToMarker = targetMarker.position.horizontal_angle

        if(abs(angleToMarker) < 0.125):

            #Turn back a tiny bit
            pVal = angleToMarker * 0.5
            iVal = totalError * 0.01
            dVal = (angleToMarker - lastError) * 0.005

            motorTime = pVal + iVal + dVal #PID!!!! (P seems to be the most useful)
            setMotors(0.25 * numpy.sign(angleToMarker), -0.25 * numpy.sign(angleToMarker))
            robot.sleep(abs(motorTime))

            return targetMarker.id
        else:
            pVal = angleToMarker * 0.3
            iVal = totalError * 0.025
            dVal = (angleToMarker - lastError) * 0.02
            if(lastError == 0):
                pVal = pVal * 1.8

            motorTime = pVal + iVal + dVal #PID!!!! (P seems to be the most useful)
            setMotors(0.25 * numpy.sign(angleToMarker), -0.25 * numpy.sign(angleToMarker))
            robot.sleep(abs(motorTime))

            turntowardsMarker(suitableMarkers, angleToMarker, totalError + angleToMarker,numRotations)

    else:
        setMotors(-0.25,0.25)
        robot.sleep(0.5)
        if(numRotations % 15 == 0 and numRotations > 20):
            setMotors(-0.4,-0.4)
            robot.sleep(0.5)
        
        if numRotations <= 15:
            turntowardsMarker(suitableMarkers, 0, 0, numRotations + 1)
        else:
            turntowardsMarker([0,1,2,3,4,5,6,7], 0, 0, numRotations + 1)"""


def movetowardsMarker(suitableMarkers: List[int], forwardPower: int, forwardDistance: int, turnPower: int, turnTime: int) -> int:
    """
    This function moves the robot towards a set marker. It takes a list of marker numbers, ordered from high to low priority. 
    - forwardPower is the power at which the motors are turned. 
    - forwardDistance is the distance from the robot to the wall when it stops. 
    - turnPower changes the speed of the turn. 
    - turnTime changes the time the turn is executed for.
    """

    #Note: turnTime and Power refer to turning the motors after it's detected the wall
    # Basically whether the next turn is left/right and by how much (so turntowardsMarker doesn't have to do all the turning)

    #Order of operations (for this function):
    # turn towards marker - markerID
    # move forwards until close enough - forwardPower, forwardDistance
    # turn - turnPower,turnTime

    #various acceleration variables
    timeToFullPower = 0.2
    time = 0
    timeIncrement = 0.025
    sigmoidAmount = 10

    t = turntowardsMarker(suitableMarkers)
    if t == None:
        t = turntowardsMarker([0,1,2,3,4,5,6,7])
    motorPower = 0
    setMotors(motorPower, motorPower)
    disArray = [-1,-1,-1]
    index = 0
    while True:
        time += timeIncrement

        motorPower = forwardPower / (1 + numpy.exp(-1 * sigmoidAmount * (time - (timeToFullPower/2))))
        if(time > timeToFullPower):
            time = timeToFullPower
            motorPower = forwardPower
        setMotors(motorPower,motorPower)
        dis = frontUltrasound()
        if dis == 4000:  ## fix it for simulation
            dis = 0
        disArray[index] = max(dis,0)
        index += 1
        if(index >= 3):
            index = 0

        avgDis = 0
        divisor = 0
        for i in disArray:
            if i > 0:
                avgDis += i
                divisor += 1
        if divisor > 0:
            avgDis = avgDis / divisor
        else:
            avgDis = forwardDistance + 1

        if(avgDis < forwardDistance and avgDis > 0):
            break

        robot.sleep(0.025)
    
    while time > 0:
        time -= timeIncrement
        motorPower = forwardPower / (1 + numpy.exp(-1 * sigmoidAmount * (time - (timeToFullPower/2))))
        setMotors(motorPower,motorPower)
        robot.sleep(0.025)

    setMotors(0,0)
    robot.sleep(0.01)

    setMotors(turnPower, -turnPower)
    robot.sleep(turnTime*turnFactor)

    return t // 2

c = [[[1,0,3,2],0.5,950,-0.25,0.75],
    [[3,2,5,4],0.5,750,0.25,0.75],
    [[5,4,7,6],0.5,1000,-0.25,0.75],
    [[7,6,0,1],0.5,750,-0.25,0.75]
]
# 2D list of (ordererd list of markers), forward speed, distance from wall after turn, direction / speed to turn, time to turn.

## edited for sim:
c = [[[1,0,3,2],0.5,950,-0.25,0.75],
    [[3,2,5,4],0.5,750,0.25,0.75],
    [[5,4,7,6],0.5,1000,-0.25,0.75],
    [[7,6,0,1],0.5,1500,0,0] ## in sim when it gets to this corner, it sometimes gets confused and goes to the previous one 6,7?. mostly fixed by not automatically turning. still occasionally doing it though.
] # in sim it got 5 laps in 2:30 minutes, 113 in 60 minutes -> average 4.7 per 2.5 minutes

def competition():

    crntIndex = 0
    laps = 0
    while True:
        print(f"Corner {crntIndex + 1}")
        """if crntIndex == 3:
            setMotors(0.5,0.5)
            robot.sleep(3)
            setMotors(-0.23,0.23)
            robot.sleep(0.75)
            setMotors(0,0)  
        else:"""
        crntIndex = movetowardsMarker(c[crntIndex][0],c[crntIndex][1],c[crntIndex][2],c[crntIndex][3],c[crntIndex][4])
        #movetowardsMarker(*[x for x in c[crntIndex]])
        if crntIndex != 3:
            crntIndex += 1
        else:
            laps += 0.5
            print("Laps: ", laps)
            crntIndex = 0



def testing():
    #setMotors(0,1)
    #while True:
    #    print(frontUltrasound())
    while True:
        movetowardsMarker([1,0],0.5,500,0,0.1)
    #    robot.sleep(0.5)

def forwardTest():
    setMotors(0.5,0.5)
    robot.sleep(1)
    setMotors(0,0)


#robot.sleep(1)
competition()
#forwardTest()
#testing()
robot.sleep(1)


