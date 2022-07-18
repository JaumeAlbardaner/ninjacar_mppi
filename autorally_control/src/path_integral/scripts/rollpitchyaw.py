#!/usr/bin/env python

"""command_republisher.py
Republishes another version of the computed command in the 
same frame as the state estimator.
"""

import numpy as np
import math

import rospy
import tf

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
class CommandRepublisher(object):
  
  def __init__(self, namespace='command_republisher'):
    """Initialize this _command_republisher"""
    rospy.init_node("command_republisher", anonymous = True)
    self.roll = rospy.Publisher('/roll', Float32, queue_size=1)
    self.pitch = rospy.Publisher('/pitch', Float32, queue_size=1)
    self.yaw = rospy.Publisher('/yaw', Float32, queue_size=1)
    self.sub = rospy.Subscriber("/odom_reframer/odom_chassis", Odometry, self.handle_pose)
  
  def handle_pose(self, msg):
    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z

    yaw = math.atan2(2.0*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3)
    pitch = math.sin(-2.0*(q1*q3 - q0*q2))
    roll = math.atan2(2.0*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)
    
    roll_msg=Float32()
    pitch_msg=Float32()
    yaw_msg=Float32()

    roll_msg.data = roll
    pitch_msg.data = pitch
    yaw_msg.data = yaw

    self.roll.publish(roll_msg)
    self.pitch.publish(pitch_msg)
    self.yaw.publish(yaw_msg)


if __name__ == "__main__":
  command_republisher = CommandRepublisher()
  rospy.spin()