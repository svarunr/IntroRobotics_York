#!/usr/bin/env python3

import rospy
import roslib
import math
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import sin, cos, pi

# Inspiration for this code from: https://answers.ros.org/question/336019/run-time-update-of-odometry-message-while-publishing-to-a-new-topic/
class NoisyOdometry:

    def odom_callback(self, msg):
        self.msg = msg

    # This section is inspired by code from Xuan Sang's article on adding noise to odom data at: https://blog.lxsang.me/post/id/16 
    def compute_noise(self, msg):
        # Determine the perfect quaternion data from /odom.
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        (r, p, thetac) = tf.transformations.euler_from_quaternion(quat)
       
        # First message published to /noisy_odom.
        if (self.last_odom == None):
            self.last_odom = msg 
            self.pose[0] = msg.pose.pose.position.x # Current x position.
            self.pose[1] = msg.pose.pose.position.y # Current y position.
            self.pose[2] = thetac # Current deviation angle. 
            self.pub.publish(self.last_odom)
        # Subsequent messages published to /noisy_odom.
        else:
            # Compute the distance travelled since previous reading.
            dist_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x 
            dist_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            dist_hyp = math.sqrt(dist_x ** 2 + dist_y ** 2)

            # Extract previous quaternion data; this is perfect /odom data used to calculate angular deviation since previous reading.
            quat = [self.last_odom.pose.pose.orientation.x,
                    self.last_odom.pose.pose.orientation.y,
                    self.last_odom.pose.pose.orientation.z,
                    self.last_odom.pose.pose.orientation.w]
            (r, p, thetap) = tf.transformations.euler_from_quaternion(quat)
            
            # Compute the angular deviation since previous reading using x and y distances.
            dist_rot1 = math.atan2(dist_y, dist_x) - thetap
            dist_rot2 = thetac - thetap - dist_rot1
            
            # Add noise to the distances computed above.
            dist_hyp  += np.random.normal(loc=0, 
                                          scale=math.fabs(dist_hyp * self.std))
            dist_rot1 += np.random.normal(loc=0, 
                                          scale=math.fabs(dist_rot1 * self.std))
            dist_rot2 += np.random.normal(loc=0, 
                                          scale=math.fabs(dist_rot2 * self.std))

            # Compute the individual channels from absolute data above.
            self.pose[0] += dist_hyp * cos(thetap + dist_rot1)
            self.pose[1] += dist_hyp * sin(thetap + dist_rot1)
            self.pose[2] += dist_rot1 + dist_rot2
            
            # Store current perfect /odom data for distance calculations in next iteration.
            self.last_odom = msg

            # Overwrite the original /odom message for publishing.
            msg.pose.pose.position.x = self.pose[0]
            msg.pose.pose.position.y = self.pose[1]
            
            eqt = tf.transformations.quaternion_from_euler(0, 0, self.pose[2]) 
            msg.pose.pose.orientation.x = eqt[0]
            msg.pose.pose.orientation.y = eqt[1]
            msg.pose.pose.orientation.z = eqt[2]
            msg.pose.pose.orientation.w = eqt[3]
            
        self.pub.publish(msg)

    def __init__(self):
        self.sub  = rospy.Subscriber("odom", Odometry, self.odom_callback) # Set the subscriber.
        self.pub  = rospy.Publisher('noisy_odom', Odometry, queue_size=10) # Set the publisher.
        self.msg  = Odometry() # Instantiate variable that stores the Odometry data from /odom.
        self.last_odom = None # Odometry variable.
        self.pose = [0.0, 0.0, 0.0] # x, y and rotational pose array.
        self.std = 0.10 # Standard deviation of noise per meter.
        self.rate = rospy.Rate(0.9) # /odom is published to at rate of 20 Hz.

if __name__ == '__main__':
    # Initialize the noisy_odom node.
    rospy.init_node('noisy_odom_node', anonymous=False)
    # Initialize an instance of the NoisyOdometry() class.
    odom = NoisyOdometry()
    # Run the compute_noise function while the node is active.
    try:
        # Wait a bit to get the first /odom message.
        odom.rate.sleep()
        while not rospy.is_shutdown():
            # Compute noise and publish to /noisy_odom.
            odom.compute_noise(odom.msg)
            odom.rate.sleep()
    except rospy.ROSInterruptException:
        pass
    
    # Unregister /noisy_odom.
    rospy.loginfo(f"Unregistering from /odom (Clean shutdown)")
    odom.sub.unregister()

