#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

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
    listener = tf.TransformListener()
    vel_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()

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

    arm.set_named_target("up_bottun")
    arm.go()

    trans = [0.0, 0.0, 0.0] 
    while(1):
      lock = 1
      try:
        #now = rospy.Time.now()
        #listener.waitForTransform("/ar_marker_187", "/base_link", now, rospy.Duration(3.0))
        (trans,rot) = listener.lookupTransform('/bottun' ,'/ar_marker_7', rospy.Time(0))

        print("ar->base_link",trans)
        lock = 0
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("tf error")
      if lock is 0:
        break

    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1)


'''
    arm_initial_pose = arm.get_current_pose().pose
    print "=" * 10, " Printing running pose: "
    print arm_initial_pose

    x_bias = -0.03
    z_bias = 0.05
    arm_initial_pose.position.x += x_bias
    arm_initial_pose.position.z += trans[1] + z_bias
    arm.set_pose_target(arm_initial_pose)
    arm.go()
    print "=" * 10, " Printing target pose: "
    print arm_initial_pose


    rospy.sleep(1)
    arm.clear_pose_targets()

    #push bottun
    x_bias = 0.04
    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_pose.position.x += x_bias
    arm.set_pose_target(arm_initial_pose)
    arm.go()
    rospy.sleep(1)
    arm.clear_pose_targets()
'''

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
