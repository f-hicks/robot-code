from sbot import Robot
from sbot import BRAKE, COAST
from sbot import AnalogPins
from sbot import GPIOPinMode
from enum import Enum

def sleep(time):
    robot.sleep(time)

robot = Robot()

USDirection = Enum("Ultrasound", "LEFT CENTRE RIGHT")

def updateUltraSound(side: USDirection):
    if side == USDirection.CENTRE:
        return robot.arduino.ultrasound_measure(2,3) ## change to the pins we actually use for the different sensors
    elif side == USDirection.LEFT:
        return robot.arduino.ultrasound_measure(0,0) ## change to the pins we actually use for the different sensors
    elif side == USDirection.RIGHT:
        return robot.arduino.ultrasound_measure(0,0) ## change to the pins we actually use for the different sensors

leftMotor = robot.motor_board.motors[0]
rightMotor = robot.motor_board.motors[1]

def driveStraight(time, power, after=COAST):
    leftMotor.power = power
    rightMotor.power = power
    if time!=0:
        robot.sleep(time)
        leftMotor.power = after
        rightMotor.power = after
    
def turn(degrees, direction, power, after=COAST):
    if direction == 1:
        leftMotor.power = power
        rightMotor.power = -power
    else:
        leftMotor.power = -power
        rightMotor.power = power
    
    robot.sleep(degrees*0.07)
    leftMotor.power = COAST
    rightMotor.power = COAST
    
def stop():
    leftMotor.power = BRAKE
    rightMotor.power = BRAKE
    
sleep(1)

while True:
    driveStraight(2, 0.1)
    