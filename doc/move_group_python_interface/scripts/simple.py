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

    print "=" * 10, "Robot Groups:" 
    #print robot.get_gropu_names()
    print "=" * 10

    print "=" * 10, "Robot state:" 
    print robot.get_current_state()
    print "=" * 10

    arm = moveit_commander.MoveGroupCommander("interaction_arm")


    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1)

    arm.set_named_target("running")
    arm.go()
 
    
    while 1:
      arm.set_named_target("up_bottun_45")
      arm.go()
      print("push bottun")
      arm.set_named_target("running")
      arm.go()
      print("return running pose")
   

    arm_initial_pose.position.x += 0.01
    arm.set_pose_target(arm_initial_pose)
    arm.go()
    print "=" * 10, " Printing x+0.01 pose: "
    print arm_initial_pose


    rospy.sleep(1)
    arm.clear_pose_targets()

    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
