from sbot import Robot
from sbot import BRAKE, COAST
rbot = Robot()

fudgeFactor=0.99403

def drive_straight(time, power):
    rbot.motor_board.motors[0].power = power*fudgeFactor
    rbot.motor_board.motors[1].power = power
    rbot.sleep(10)
    rbot.motor_board.motors[0].power = COAST
    rbot.motor_board.motors[1].power = COAST

def turn(degrees, direction, power):
    if direction: 
        rbot.motor_board.motors[0].power = power*fudgeFactor
        rbot.motor_board.motors[1].power = -power
    else:
        rbot.motor_board.motors[0].power = -power*fudgeFactor
        rbot.motor_board.motors[1].power = power
    rbot.sleep(degrees*0.007)
    rbot.motor_board.motors[0].power = COAST
    rbot.motor_board.motors[1].power = COAST




#drive_straight(10,0.25)

turn(90,1,0.25)
