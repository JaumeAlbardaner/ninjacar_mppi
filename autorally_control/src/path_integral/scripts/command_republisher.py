#!/usr/bin/env python

"""command_republisher.py
Republishes another version of the computed command in the 
same frame as the state estimator.
"""

import numpy as np
import math

import rospy
import tf

from autorally_msgs.msg import chassisCommand
from drive_control.msg import DriveCommand

class CommandRepublisher(object):
  
  def __init__(self, namespace='command_republisher'):
    """Initialize this _command_republisher"""
    rospy.init_node("command_republisher", anonymous = True)
    self.ex_throttle = 0 
    self.pub = rospy.Publisher("/planner_command", DriveCommand, queue_size = 1)
    self.sub = rospy.Subscriber("/mppi_controller/chassisCommand", chassisCommand, self.handle_pose)
  
  def handle_pose(self, message):
    msg = DriveCommand()
    msg.rear_steer_angle = 0
    msg.front_steer_angle = message.steering
    if message.throttle <0:
      msg.control_value = 0.2
    else:
      msg.control_value = 0.2 
      self.ex_throttle = message.throttle 

    self.pub.publish(msg)

if __name__ == "__main__":
  command_republisher = CommandRepublisher()
  rospy.spin()