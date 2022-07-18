#!/usr/bin/env python

"""command_republisher.py
Republishes another version of the computed command in the 
same frame as the state estimator.
"""

import numpy as np
import math

import rospy
import tf

from autorally_msgs.msg import chassisState
from geometry_msgs.msg import PoseStamped
from drive_control.msg import DriveCommand

class CommandRepublisher(object):
  
  def __init__(self, namespace='command_republisher'):
    """Initialize this _command_republisher"""
    rospy.init_node("command_republisher", anonymous = True)
    self.header = None
    self.pub = rospy.Publisher("/vehicle_controls", chassisState, queue_size = 1)
    self.sub = rospy.Subscriber("/vrpn_client_node/N03/pose", PoseStamped , self.updateheader)
    self.sub2 = rospy.Subscriber("/drive/command", DriveCommand , self.handle_pose)

  def updateheader(self, msg):
    self.header = msg.header
  
  def handle_pose(self, message):
    if self.header != None:
      msg = chassisState()
      msg.header = self.header
      msg.steering = message.front_steer_angle
      msg.throttle = message.control_value   

      self.pub.publish(msg)

if __name__ == "__main__":
  command_republisher = CommandRepublisher()
  rospy.spin()