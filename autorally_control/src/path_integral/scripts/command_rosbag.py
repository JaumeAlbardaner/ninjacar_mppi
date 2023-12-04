#!/usr/bin/env python

"""command_rosbag.py
Republishes another version of the computed command in the 
same frame as the state estimator for the rosbag.
Used to generate the neuralNet
"""

import numpy as np
import math

import rospy
import tf

from nav_msgs.msg import Odometry
from drive_control.msg import DriveCommand
from autorally_msgs.msg import chassisState

class CommandRepublisher(object):
  
  def __init__(self):
    """Initialize this _command_republisher"""
    rospy.init_node("command_rosbag", anonymous = True)
    self.pub = rospy.Publisher("/vehicle_controls", chassisState, queue_size = 1)
    self.sub = rospy.Subscriber("/drive/command", DriveCommand, self.handle_pose)
    self.sub2 = rospy.Subscriber("/odom_reframer/odom_chassis", Odometry, self.update_time)
    self.header = None

  def update_time(self,message):
    self.header = message.header
  
  def handle_pose(self, message):
    if self.header is not None:
      msg = chassisState()
      #Fucking msgs.
      msg.header = self.header
      msg.steering = message.front_steer_angle 
      msg.throttle = message.control_value

      self.pub.publish(msg)

command_republisher = CommandRepublisher()
rospy.spin()
