#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import tf
from elevator_navigation_srv.srv import ArmMotion
import time

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("interaction_arm")
motion_list = ["home","up_bottun_45_1", "up_bottun_45_2", "up_bottun_45_3", "up_bottun_45_4", "up_bottun_45_5", "up_bottun_6", "up_bottun_7"]

def handle_arm_motion(req):

  global arm
  global motion_list

  m = motion_list[req.number]
  print("move", m)
  arm.set_named_target(m)
  arm.go()
  rospy.sleep(1)
  return 1

def main():
  global motion_list

  rospy.init_node("arm_motion_server")
  s = rospy.Service("arm_motion", ArmMotion, handle_arm_motion)
  print("start arm motion server")
  print("-- motion list --")
  print(motion_list)
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
