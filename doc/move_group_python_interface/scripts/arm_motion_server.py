#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import tf
from elevator_navigation_srv.srv import ArmMotion
import time

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("interaction_arm")

def handle_arm_motion(req):

  motion_list = ["home", "running"]
  global arm

  m = motion_list[req.number]
  print("move", m)
  arm.set_named_target(m)
  arm.go()
  rospy.sleep(1)
  return 1

def main():
  rospy.init_node("arm_motion_server")
  s = rospy.Service("arm_motion", ArmMotion, handle_arm_motion)
  print("start arm motion server")
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
