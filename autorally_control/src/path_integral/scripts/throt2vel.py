#!/usr/bin/env python

"""throt2vel.py

Reads the goal throttle output from the MPPI algorithm and transform it into
velocity for the ninjacar to process.
"""

import numpy as np
import rospy
import tf
from drive_control.msg import DriveCommand
from autorally_msgs.msg import chassisCommand
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_vector3


class ThrottleRepublisher(object):

    def __init__(self, namespace='throttle_republisher'):
        """Initialize this _throttle_republisher"""

        #Instantiate variables
        self.gain = 1
        self.oldTime = None
        self.oldSpeed = 0
        #Topics we'll listen to
        throttle_raw_topic = "/mppi_controller/chassisCommand"
        odometry_topic = "fuckmylife_1080p"

        rospy.init_node(namespace, anonymous=True)
        self.pub = rospy.Publisher("/mppi_controller/chassisCommand_vel", DriveCommand, queue_size=1)
        self.sub = rospy.Subscriber(throttle_raw_topic, chassisCommand, self.handle_throttle)
        self.sub2 = rospy.Subscriber(odometry_topic, Odometry, self.handle_pose)

    #I think this may be improved by accounting for the horizontal and vertical velociites
    def handle_pose(self,msg):
        self.oldSpeed = msg.twist.twist.linear.x
        # self.oldSpeed2 = msg.twist.twist.linear.y
        # self.oldSpeed3 = msg.twist.twist.linear.z

    # this is the callback
    def handle_throttle(self, msg):
        #Time the message was received
        newTime = msg.header.secs + msg.header.nsecs * 10**(-9)
        newSpeed = 0
        a = msg.throttle
        #If previous message, we operate on it
        if self.oldtime is not None:
            dT = newTime - self.oldTime
            newSpeed = self.oldSpeed + self.gain*dT*a
            
            #Send the new control message
            message = DriveCommand()
            message.speed = newSpeed
            message.front_steer_angle = msg.steering
            message.rear_steer_angle = 0
            self.pub.publish(message)

        #Update "old" message
        self.oldTime = newTime
        self.oldSpeed = newSpeed



if __name__ == "__main__":
    throttle_republisher = ThrottleRepublisher()
    rospy.spin()