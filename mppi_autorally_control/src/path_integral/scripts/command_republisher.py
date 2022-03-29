#!/usr/bin/env python

"""command_republisher.py

Republishes another version of the computed command in the 
same frame as the state estimator.
"""

import numpy as np
import math

import rospy
import tf

from mppi_autorally_msgs.msg import chassisCommand
from carplanner_msgs.msg import Command

class CommandRepublisher(object):
  
  def __init__(self, namespace='command_republisher'):
    """Initialize this _command_republisher"""
    rospy.init_node("command_republisher", anonymous = True)
    self.pub = rospy.Publisher("/computedCommand", Command, queue_size = 1)
    self.sub = rospy.Subscriber("/mppi_controller/chassisCommand", chassisCommand, self.handle_pose)
  
  def handle_pose(self, message):
    msg = Command()
    msg.dt = 1./rospy.get_param('mppi_controller')['hz']
    msg.dphi = message.steering
    msg.force = message.throttle #Maybe multiply by mass?

    self.pub.publish(msg)

if __name__ == "__main__":
  command_republisher = CommandRepublisher()
  rospy.spin()
  