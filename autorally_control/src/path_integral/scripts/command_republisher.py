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
  
  def __init__(self):
    """Initialize this _command_republisher"""
    rospy.init_node("command_republisher", anonymous = True)
    self.pub = rospy.Publisher("/planner_command", DriveCommand, queue_size = 1)
    self.sub = rospy.Subscriber("/mppi_controller/chassisCommand", chassisCommand, self.handle_pose)
    self.throttle_scale = rospy.get_param("~throttle_scale",1.0)
    update_rate = 1.0
    self.update_param_timer = rospy.Timer(rospy.Duration(1.0/update_rate), self.update_param)

  def update_param(self, timer):
    self.throttle_scale = rospy.get_param("~throttle_scale",1.0)

  def handle_pose(self, message):
    msg = DriveCommand()
    msg.rear_steer_angle = 0
    msg.front_steer_angle = message.steering
    msg.control_value = message.throttle*self.throttle_scale

    self.pub.publish(msg)

if __name__ == "__main__":
  command_republisher = CommandRepublisher()
  rospy.spin()
