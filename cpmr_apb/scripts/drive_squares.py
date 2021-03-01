#!/usr/bin/env python3

import rospy
import roslib
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import sin, cos, pi

class DriveSquares():
    def drive_squares(self, cmd_vel):
        # Start driving squares here; using drive_to_goal.py as inspiration.
        twist = Twist()
        twist.linear.x = 0.2
        rospy.loginfo(f"Starting to publish on {cmd_vel}...")
        for d in range(100): # Send forward command for 10 Hz * 10 seconds = 100 iterations
            self.pub.publish(twist)
            self.rate.sleep()
            
        twist = Twist()
        twist.angular.z = -0.5236 # deg2rad(30) clockwise; requires 3 seconds to turn 90 degrees.
        rospy.loginfo(f"Starting to turn at rate of {twist.angular.z} rad/s...")
        for a in range(34):
            self.pub.publish(twist)
            self.rate.sleep()

    def __init__(self):
        #self.sub = rospy.Subscriber("odom", Odometery, odom_callback) # Subscribe to perfect /odom data to correctly drive robot.
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # Instantiate /cmd_vel publisher for moving robot.
        self.rate = rospy.Rate(10)

if __name__ == '__main__':
    # Instantiate the drive_squares node and create Class object.
    rospy.init_node("drive_squares")
    driver = DriveSquares()

    # Grab the current command velocity topic and pass that to the drive_squares function.
    cmd_vel = rospy.get_param("~cmd_vel", "cmd_vel")
    for i in range(4):
        driver.drive_squares(cmd_vel)

    # Stop the robot after completing a full square.
    twist = Twist()
    driver.pub.publish(twist)
