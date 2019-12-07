#!/usr/bin/env python

import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from std_msgs.msg import Int32
import time


ID_UP_ON = 1
UP_BOTTUN_LOCK = 1
STATE = 0


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

def state_callback(msg):
  global STATE
  STATE = msg.data
  print("state is :", STATE)

def main():
  rospy.init_node("arm_motion_server")
  rospy.Subscriber("arm_motion", Int32, state_callback, queue_size=1)

  print("start arm motion server")

  robot = moveit_commander.RobotCommander()

  arm = moveit_commander.MoveGroupCommander("interaction_arm")


  motion_list = ["home", "running"]
  global STATE
  last_state = STATE

  m = motion_list[STATE]
  print("move", m)
  arm.set_named_target(m)
  arm.go()
  rospy.sleep(1)

  while 1:
    if last_state is not STATE:
      m = motion_list[STATE]
      print("move", m)
      arm.set_named_target(m)
      arm.go()
      rospy.sleep(1)
    else:
      print("")
      print("wait arm_motion topic")
      print("-- motion list --")
      for i in range(len(motion_list)):
        print(i, motion_list[i])
    last_state = STATE
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
