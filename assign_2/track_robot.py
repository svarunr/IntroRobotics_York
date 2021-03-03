#!/usr/bin/python3

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String


beacons = []
beacons.append(("beacon_0", 1, 1))
beacons.append(("beacon_1", 5, 1))
beacons.append(("beacon_2",-5, 1))

def distance(x1, y1, x2, y2):
    x_delta = x1 - x2
    y_delta = y1 - y2
    return math.sqrt(x_delta*x_delta + y_delta*y_delta)

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    publish_b0 = rospy.Publisher('beacon_0', String, queue_size=10)
    publish_b1 = rospy.Publisher('beacon_1', String, queue_size=10)
    publish_b2 = rospy.Publisher('beacon_2', String, queue_size=10)

    for beacon_name, beacon_x, beacon_y in beacons:
        dist = distance(x, y, beacon_x, beacon_y)

        #rospy.loginfo('current robot position x:{}, y:{}'.format(x, y))
        #rospy.loginfo('distance from {}, d = {}'.format(beacon_name, dist))

        data_str = 'source: {}, position: x:{} y:{}, distance: {}'.format(beacon_name, beacon_x, beacon_y, dist)

        if beacon_name == "beacon_0":
            publish_b0.publish(data_str)
        if beacon_name == "beacon_1":
            publish_b1.publish(data_str)
        else:
            publish_b2.publish(data_str)
        
def main():
   
    rospy.init_node('track_robot')
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()
    
if __name__ == '__main__':
    main()

