from sbot import Robot
from sbot import GPIOPinMode
from sbot import AnalogPins
import time
import numpy
from typing import List

robot = Robot()
IO = robot.arduino

fudge = 0.96

RightMotor = robot.motor_board.motors[0]
LeftMotor = robot.motor_board.motors[1]

def frontUltrasound():
    return IO.ultrasound_measure(2, 3)

def leftUltrasound():
    return IO.ultrasound_measure(4, 5)

def rightUltrasound():
    return IO.ultrasound_measure(6, 7)
    
def setMotors(left, right):
    LeftMotor.power = left
    RightMotor.power = right * fudge

# This function checks visible maker list for a specific target
def find_marker(markers, suitableMarkers: List[int]): #it will now prefer otherID over the id_number
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
                
                lastError = angleToMarker
                totalError += angleToMarker

        else:
            setMotors(-0.25,0.25)
            robot.sleep(0.5)
            if(numRotations <= 15):
                setMotors(-0.4,-0.4)
                robot.sleep(0.5)
                numRotations += 1
            else:
                print("finding any target")
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


def movetowardsMarker(suitableMarkers, forwardPower,forwardDistance,turnPower,turnTime):

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

        disArray[index] = max(frontUltrasound(),0)
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
    robot.sleep(turnTime)

    return t // 2

c = [[[1,0,3,2],0.5,950,-0.25,0.75],
    [[3,2,5,4],0.5,750,0.25,0.75],
    [[5,4,7,6],0.5,1000,-0.25,0.75],
    [[7,6,0,1],0.5,750,-0.25,0.75]
]# 2D list of marker, secondary marker (or None), direction to turn, distance from wall after turn

def competition():

    crntIndex = 0
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

        if crntIndex != 3:
            crntIndex += 1
        else:
            crntIndex = 0



def testing():

    while True:
        movetowardsMarker([5,4],0.5,500,0,0.1)
        robot.sleep(0.5)

def forwardTest():
    setMotors(0.5,0.5)
    robot.sleep(2)
    setMotors(0,0)



competition()


