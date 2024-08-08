from sbot import Robot
rbot = Robot()
rbot.motor_board.motors[0].power = 0.5
rbot.motor_board.motors[1].power = 0.5
rbot.sleep(5)
rbot.motor_board.motors[0].power = 0
rbot.motor_board.motors[1].power = 0
rbot.sleep(5)
rbot.motor_board.motors[0].power = -1
rbot.motor_board.motors[1].power = 0
rbot.sleep(5)
rbot.motor_board.motors[0].power = 0
rbot.motor_board.motors[1].power = 0
rbot.sleep(5)