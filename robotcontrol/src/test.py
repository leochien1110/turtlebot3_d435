#!/usr/bin/env python3
import time
from robot_control_class import RobotControl

robotcontrol = RobotControl()

# robotcontrol.move_straight_time("forward", 1, 3)
# # robotcontrol.stop_robot()
# d = robotcontrol.get_laser(0)
# print(d)

while True:
    s = robotcontrol.move_straight_time("forward", 0.2, 2)
    print(s)

    d = robotcontrol.get_laser(0)
    print(d)

    # time.sleep(3)
    
    # robotcontrol.turn("counter_clockwise", 1, 3)
    # robotcontrol.stop_robot()