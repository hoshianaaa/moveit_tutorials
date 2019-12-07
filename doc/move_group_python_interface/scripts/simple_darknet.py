#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from std_msgs.msg import Empty
import time

ID_UP_ON = 1
UP_BOTTUN_LOCK = 1
START = 0


class Vector3:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

def euler_to_quaternion(euler):
  q = tf.transformations.quaternion_from_euler(euler.x, euler.y,euler.z)
  return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def bounding_boxes_callback(msg):
  print("bounding box call back")
  for i in range(len(msg.bounding_boxes)):
    if msg.bounding_boxes[i].id is ID_UP_ON:
      print("detect up_on bounding_box")
      global UP_BOTTUN_LOCK
      UP_BOTTUN_LOCK = 0

def arm_callback(msg):
  global START
  START = 1
  print("start arm callback")
  time.sleep(0.1)

def main():
    rospy.init_node("moveit_command_sender")
    rospy.Subscriber("start_arm", Empty, arm_callback, queue_size=1)
    
    global START
    while START is 0:
      print(START)
      time.sleep(0.1)


    print("start arm motion")

    bounding_box_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)

    robot = moveit_commander.RobotCommander()

    arm = moveit_commander.MoveGroupCommander("interaction_arm")


    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1)
    print("home")

    arm.set_named_target("running")
    arm.go()
    print("running")

    motion_list = ["up_bottun_45", "up_bottun_46", "up_bottun_46_2"]

    counter = 0
    global UP_BOTTUN_LOCK
    while UP_BOTTUN_LOCK:
      i = counter % 3
      arm.set_named_target(motion_list[i])
      arm.go()
      print("push bottun")
      arm.set_named_target("running")
      arm.go()
      print("return running pose")
      rospy.sleep(1)
      arm.clear_pose_targets()
      counter += 1

    arm.set_named_target("home")
    arm.go()
    print("home")
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
