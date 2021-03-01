#!/usr/bin/python3
#
# A very simple ros node to populate the world
#
import rospy
import os

def make_beacon(id, x, y):
  os.system(f"rosrun gazebo_ros spawn_model -database beacon -sdf -model beacon_{id} -x {x} -y {y}") #
if __name__ == '__main__':
  rospy.init_node('populate')
  
  id = 0
  make_beacon(id, 1, 1)
  id = id+1
  make_beacon(id, 5, 1)
  id = id+1
  make_beacon(id,-5, 1)
  rospy.signal_shutdown("Placed beacons")
    

