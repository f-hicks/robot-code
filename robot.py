from sbot import Robot
from sbot import BRAKE, COAST
from sbot import AnalogPins
from sbot import GPIOPinMode
from enum import Enum

def sleep(time):
    robot.sleep(time)

robot = Robot()

USDirection = Enum("Ultrasound", "LEFT CENTRE RIGHT")

def getUltraSound(side: USDirection):
    if side == USDirection.CENTRE:
        return robot.arduino.ultrasound_measure(2,3) ## change to the pins we actually use for the different sensors
    elif side == USDirection.LEFT:
        return robot.arduino.ultrasound_measure(4,5) ## change to the pins we actually use for the different sensors
    elif side == USDirection.RIGHT:
        return robot.arduino.ultrasound_measure(6,7) ## change to the pins we actually use for the different sensors

leftMotor = robot.motor_board.motors[0]
rightMotor = robot.motor_board.motors[1]

def driveStraight(time, power, after=COAST):
    leftMotor.power = power
    rightMotor.power = power
    if time!=0:
        sleep(time)
        leftMotor.power = after
        rightMotor.power = after
    
def turn(degrees, direction, power, after=COAST):
    if direction == 1:
        leftMotor.power = -power
        rightMotor.power = power
    else:
        leftMotor.power = power
        rightMotor.power = -power
    
    sleep(degrees*0.0073)
    leftMotor.power = after
    rightMotor.power = after
    
def stop():
    leftMotor.power = BRAKE
    rightMotor.power = BRAKE
    
#TODO use a list to iteration through direction for each turn.    
#TODO add support for multiple ultrasonic sensors

sleep(1)
#first corner
while getUltraSound(USDirection.CENTRE) > 720:
    print(getUltraSound(USDirection.CENTRE))
    driveStraight(0, 0.1)
stop()
turn(90, 1, 0.1)

#second corner
while getUltraSound(USDirection.CENTRE) > 720:
    print(getUltraSound(USDirection.CENTRE))
    driveStraight(0, 0.1)
stop()
turn(90, -1, 0.1)

#third corner
while getUltraSound(USDirection.CENTRE) > 720:
    print(getUltraSound(USDirection.CENTRE))
    driveStraight(0, 0.1)
stop()
turn(90, 1, 0.1)

#diagonal bit
driveStraight(0, 0.1)

#TODO make a function to make sure the robot is exactly straight so that it doesn't detect wrong distances.

def straighten():
     #compare left and right sonars 
    left = getUltraSound(USDirection.LEFT)
    right = getUltraSound(USDirection.RIGHT)
    print(left,right)
    if right < 600 or left < 600: # a can is detected
        print("Can")
        return
    before = leftMotor.power, rightMotor.power
    if left > right:
        turn(1, -1, 0.05, BRAKE)
        
    elif right > left:
        turn(1, 1, 0.05, BRAKE)
    leftMotor.power, rightMotor.power = before
        
    

passed1 = False
passed2 = False
while True:
    left = getUltraSound(USDirection.LEFT)
    print(left)
    if not passed2:
        if left < 800 and not passed1:
            #now into the middle bit
            passed1 = True
        elif passed1 and left > 800:
            passed2 = True
        #if passed1:
            #straighten()
    else:# wait for gettings past next block
        if left < 4000 and left > 3000:
            stop()
            break

turn(90, 1, 0.1, BRAKE)

driveStraight(5,0.1, BRAKE)
        
    

