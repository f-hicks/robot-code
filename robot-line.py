from typing import Tuple
from sbot import Robot
from sbot import BRAKE, COAST
from sbot import AnalogPins
from sbot import GPIOPinMode

from enum import Enum

LightSensor = Enum("Light Sensor", "LEFT CENTRE RIGHT")
Direction = Enum("direction", "LEFT  RIGHT")


rbot = Robot()

fudgeFactor=0.99403

arduino = rbot.arduino
rbot.arduino.pins[AnalogPins.A0].mode = GPIOPinMode.INPUT
pin_value = rbot.arduino.pins[AnalogPins.A0].analog_value

def update_analogue() -> Tuple[float,float,float]:
    left = rbot.arduino.pins[AnalogPins.A0].analog_value
    centre = rbot.arduino.pins[AnalogPins.A1].analog_value
    right = rbot.arduino.pins[AnalogPins.A2].analog_value
    return left, centre, right
    
def get_sensor_value(sensor):
    if sensor == LightSensor.LEFT:
        pin = AnalogPins.A0
    elif sensor == LightSensor.CENTRE:
        pin = AnalogPins.A1
    elif sensor == LightSensor.RIGHT:
        pin = AnalogPins.A2
    

def drive_straight(time, power):
    rbot.motor_board.motors[0].power = power*fudgeFactor
    rbot.motor_board.motors[1].power = power
    if time!=0:
        rbot.sleep(time)
        rbot.motor_board.motors[0].power = COAST
        rbot.motor_board.motors[1].power = COAST

def turn(degrees, direction: Direction, power):
    if direction == Direction.RIGHT: 
        rbot.motor_board.motors[0].power = power*fudgeFactor
        rbot.motor_board.motors[1].power = -power
    else:
        rbot.motor_board.motors[0].power = -power*fudgeFactor
        rbot.motor_board.motors[1].power = power
    rbot.sleep(degrees*0.007)
    rbot.motor_board.motors[0].power = COAST
    rbot.motor_board.motors[1].power = COAST

def stop():
    rbot.motor_board.motors[0].power = BRAKE
    rbot.motor_board.motors[1].power = BRAKE


rbot.sleep(2)
print(update_analogue())

offTrack = False

while True:
    left, centre, right = update_analogue()
    print(left, centre, right)
    
    if offTrack and not( left > 1.5 and left < 3 and centre > 1.5 and centre < 3 and right > 1.5 and right < 3): # back on the line
        stop()
        print("Reversing...")
        offTrack = False
        continue
        
    
    if centre > 4.7:
        drive_straight(0, 0.25)
    elif left > 1.5 and left < 3 and centre > 1.5 and centre < 3 and right > 1.5 and right < 3: # lost the line.
        drive_straight(0,-0.25)
        print("OFF TRACK")
        offTrack = True
    
    elif left > 1.5 and left < 3 and centre < 1.5 and right > 4.7: # drifted off to the left
        turn(10, Direction.right, 0.25)
    
    elif right > 1.5 and right < 3 and centre < 1.5 and left > 4.7: # drifted off to the right
        turn(10, Direction.LEFT, 0.25)
    
    
    else:
        rbot.motor_board.motors[0].power = 0
        rbot.motor_board.motors[1].power = 0

#drive_straight(10,0.25)

#turn(90,1,0.25)
