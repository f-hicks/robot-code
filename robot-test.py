from sbot import Robot

robot = Robot()

#while True:
#    robot.camera.see()
#    print(robot.arduino.ultrasound_measure(2, 3))
#    robot.sleep(0.5)

robot.camera.see()
fudge = 0.95
leftMotor = robot.motor_board.motors[1]
rightMotor = robot.motor_board.motors[0]
#
leftMotor.power = 0.25
rightMotor.power = 0.25*fudge
#
robot.sleep(5)
#while True:
#    robot.sleep(1)