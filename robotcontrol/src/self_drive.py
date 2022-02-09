#!/usr/bin/env python3

from ast import mod
import math
import numpy as np

from robot_control_class import RobotControl

robotcontrol = RobotControl()

t = 1
stop = False
while t >= 0:
  
  scan = robotcontrol.get_laser_full()
  left = min(scan[20:159])
  right = min(scan[200:339])
  d = right - left
  print("d = ", d)
  if d == float("inf"):
      print("Finish tunnel, terminating self-driving...")
      robotcontrol.stop_robot()
      break

  left_pos = scan.index(left)
  right_pos = scan.index(right)

  pos = 180 - (left_pos + right_pos) / 2
  modify_pos = math.radians(pos)

  left_slant_pos = left_pos - 20
  right_slant_pos = right_pos + 20

  left_slant = scan[left_slant_pos]
  right_slant = scan[right_slant_pos]

  left_side = math.sqrt( left**2 + left_slant**2 - 2 * left * left_slant * math.cos(math.pi / 9))
  left_angle = math.asin( left_slant / left_side * math.sin(math.pi / 9))
  left_angle = math.pi / 2 - left_angle

  right_side = math.sqrt( right**2 + right_slant**2 - 2 * right * right_slant * math.cos(math.pi / 9))
  right_angle = math.asin( right_slant / right_side * math.sin(math.pi / 9))
  right_angle = math.pi / 2 - right_angle

  modify_angle = (right_angle + left_angle) / 2
  if np.isnan(modify_angle):
      continue
  angle = round(math.degrees(modify_angle))
  

  if stop == False :

    if (d > 0.2 and d <= 0.75) :

      if (modify_angle < 0) :
        print("modify angle=", -angle)
        robotcontrol.turn_and_move("counter_clockwise", 0.5, modify_angle)

      else :
        print("modify angle=", angle)
        robotcontrol.turn_and_move("clockwise", 0.5, modify_angle)

    elif (d < -0.2 and d >= -0.75):

      if (modify_angle > 0) :
        print("modify angle=", angle)
        robotcontrol.turn_and_move("clockwise", 0.5, modify_angle)

      else :
        print("modify angle=", -angle)
        robotcontrol.turn_and_move("counter_clockwise", 0.5, modify_angle)

    elif (d > 0.75) :
      modify_angle += math.pi / 18
      print("modify angle=" , angle)
      robotcontrol.turn_and_move("clockwise", 0.5, modify_angle)

    elif (d < -0.75) :
      modify_angle += math.pi / 18
      print("modify angle=", -angle)
      robotcontrol.turn_and_move("counter_clockwise", 0.5, modify_angle)

    else :
      print("modify pos=", pos)
      robotcontrol.turn_and_move("clockwise", 0.5, modify_pos)

  else :

    if t <= 8 :
      x = 0.5 - t * 0.0625
      print("slow down , modify pos=", pos)
      robotcontrol.turn_and_move("clockwise", x, modify_pos)

    else :
      print("stop")
      robotcontrol.turn_and_move("clockwise", 0, 0)

  t = t + 1

  if t > 10:
    t  = 1
    stop = not stop