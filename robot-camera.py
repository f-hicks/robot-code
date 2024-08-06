import math
from sbot import Robot
robot = Robot()
def find_marker(markers, id_number):
    for marker in markers:
        if marker.id == id_number:
            return marker
    return None

markers = robot.camera.see()
target_marker = find_marker(markers, 1)
#exit(0)
# Name the motors to make it easier
LeftMotor = robot.motor_board.motors[0]
RightMotor = robot.motor_board.motors[1]
# Helper function to set motor speeds
def set_motors(left, right):
    LeftMotor.power = left
    RightMotor.power = right
# This function checks visible maker list for a specific target
while True:
    set_motors(0, 0) #Stop the robot before taking a photo, avoids motion blur
    robot.sleep(0.2) #Time for robot to actually stop
    markers = robot.camera.see()
    target_marker = find_marker(markers, 1)
    if target_marker != None:
        angle_to_marker = math.degrees(target_marker.position.horizontal_angle)
        print(angle_to_marker)
        if angle_to_marker > 30: #REPLACE X WITH SUITABLE VALUE
            print("turn right")
            set_motors(-0.5,0.5)
        elif angle_to_marker < -30: #REPLACE X WITH SUITABLE VALUE
            print("turn left")#ADD CODE HERE TO TURN TOWARD MARKER
            set_motors(0.5,-0.5)
        elif target_marker.position.distance > 1.4: #REPLACE Y WITH SUITABLE DISTANCE
            print("continue")
            set_motors(0.5, 0.5)
            robot.sleep(0.5)
        elif target_marker.position.distance < 1.4:
            print("turning")
            set_motors(-0.5, 0.5)
            robot.sleep(0.01)
    elif find_marker(markers, 2) != None:
        print("Found marker 2")
        break

    else:
        set_motors(-0.5, 0.5)
        robot.sleep(0.01)

while True:
    set_motors(0, 0) #Stop the robot before taking a photo, avoids motion blur
    robot.sleep(0.2) #Time for robot to actually stop
    markers = robot.camera.see()
    target_marker = find_marker(markers, 2)
    if target_marker != None:
        angle_to_marker = math.degrees(target_marker.position.horizontal_angle)
        print(angle_to_marker)
        if angle_to_marker > 30: #REPLACE X WITH SUITABLE VALUE
            print("turn left")
            set_motors(0.5,-0.5)
        elif angle_to_marker < -30: #REPLACE X WITH SUITABLE VALUE
            print("turn right")#ADD CODE HERE TO TURN TOWARD MARKER
            set_motors(-0.5,0.5)
        elif target_marker.position.distance > 0.7: #REPLACE Y WITH SUITABLE DISTANCE
            print("continue")
            set_motors(0.5, 0.5)
            robot.sleep(0.5)
        elif target_marker.position.distance < 0.7:
            print("stopping")
            set_motors(-0.5, 0.5)
            robot.sleep(2)
        else:
            break

    

    #ADD CODE HERE TO FIND NEXT MARKER AND COMPLETE HALF A LAP