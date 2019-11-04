#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion

class Vector3:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

def euler_to_quaternion(euler):
  q = tf.transformations.quaternion_from_euler(euler.x, euler.y,euler.z)
  return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def main():
    rospy.init_node("moveit_command_sender")

    robot = moveit_commander.RobotCommander()

    arm = moveit_commander.MoveGroupCommander("interaction_arm")

    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
