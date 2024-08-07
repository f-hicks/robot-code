from sbot import Robot
from sbot import GPIOPinMode
from sbot import AnalogPins
import time
import numpy

robot = Robot()
IO = robot.arduino

fudge = 0.95

LeftMotor = robot.motor_board.motors[0]
RightMotor = robot.motor_board.motors[1]

def frontUltrasound():
    return IO.ultrasound_measure(2, 3)

def leftUltrasound():
    return IO.ultrasound_measure(4, 5)

def rightUltrasound():
    return IO.ultrasound_measure(6, 7)
    
def setMotors(left, right):
    RightMotor.power = right*fudge
    LeftMotor.power = left

# This function checks visible maker list for a specific target
def find_marker(markers, id_number):
    for marker in markers:
        if marker.id == id_number:
            return marker
    return None

def turntowardsMarker(markerID, lastError, totalError):

    setMotors(0,0)
    robot.sleep(0.25)
    markers = robot.camera.see()
    targetMarker = find_marker(markers, markerID)
    angleToMarker = -1

    if targetMarker != None:
        angleToMarker = targetMarker.position.horizontal_angle
    else:
        setMotors(0.3,-0.3)
        robot.sleep(0.15)
        turntowardsMarker(markerID, 0, 0)
    
    if(abs(angleToMarker) < 0.25 and angleToMarker != -1):
        return None
    elif (angleToMarker != -1):
        pVal = angleToMarker * 0.05
        iVal = totalError * 0.001
        dVal = (angleToMarker - lastError) * 0.02

        motorTime = pVal + iVal + dVal #PID!!!! (P seems to be the most useful)
        setMotors(-0.25 * numpy.sign(angleToMarker), 0.25 * numpy.sign(angleToMarker))
        robot.sleep(abs(motorTime))

        turntowardsMarker(markerID, angleToMarker, totalError + abs(angleToMarker))

def movetowardsMarker(markerID,forwardPower,forwardDistance,turnPower,turnTime):

    #Note: turnTime and Power refer to turning the motors after it's detected the wall
    # Basically whether the next turn is left/right and by how much (so turntowardsMarker doesn't have to do all the turning)

    #Order of operations (for this function):
    # turn towards marker - markerID
    # move forwards until close enough - forwardPower, forwardDistance
    # turn - turnPower,turnTime

    turntowardsMarker(markerID, 0, 0)
    setMotors(forwardPower, forwardPower)
    while True:
        if(frontUltrasound() < forwardDistance):
            break
    
    setMotors(-0.25,-0.25) #Brakes the robot
    robot.sleep(0.1)

    setMotors(turnPower, -turnPower)
    robot.sleep(turnTime)

#lap = 0
#while True:
#    print("Lap: " + str(lap))
#    movetowardsMarker(1,0.3,500,-0.1,0.7)
#    movetowardsMarker(3,0.3,500,0.1,0.7)
#    movetowardsMarker(5,0.3,500,-0.1,0.7)
#    movetowardsMarker(7,0.3,750,-0.1,0.7)
#    lap += 0.5

movetowardsMarker(3,0.5,500,0,0)

movetowardsMarker(7,0.5,500,0,0)