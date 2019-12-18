#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import tf
from elevator_navigation_srv.srv import ArmMotion
from elevator_navigation_srv.srv import ArmMotionDown
import time

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("interaction_arm")
motion_list = ["home","up_bottun_45_1", "up_bottun_45_2", "up_bottun_45_3", "up_bottun_45_4", "up_bottun_45_5", "up_bottun_45_6", "up_bottun_45_7", "up_bottun_45_8", "up_bottun_45_9", "up_bottun_45_10", "up_bottun_45_11", "up_bottun_45_12", "up_bottun_45_13", "up_bottun_45_14", "up_bottun_45_15"]
def handle_arm_motion(req):

  global arm
  global motion_list

  m = motion_list[req.number]
  print("move", m)
  arm.set_named_target(m)
  arm.go()
  rospy.sleep(1)
  return 1

down_motion_list = [ "1k_bottun_45_0", "1k_bottun_45_1", "1k_bottun_45_2", "1k_bottun_45_3", "1k_bottun_45_4",  "1k_bottun_45_5"]
def handle_arm_motion_down(req):

  global arm
  global down_motion_list

  m = down_motion_list[req.number]
  print("down move", m)
  arm.set_named_target(m)
  arm.go()
  rospy.sleep(1)
  return 1

def main():
  global motion_list

  rospy.init_node("arm_motion_server")
  s = rospy.Service("arm_motion", ArmMotion, handle_arm_motion)
  s = rospy.Service("arm_motion_down", ArmMotionDown, handle_arm_motion_down)
  print("start arm motion server")
  print("-- motion list --")
  print(motion_list)
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
