#!/usr/bin/env python3
from keras.preprocessing.image import img_to_array
from keras.models import load_model
from imutils import paths
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import random
import rospy
import cv_bridge

"""FollowRoad is a class created to drive a robot autonomously.

FollowRoad recycles code from DriveByCamera and modifies it such that the output from the _image_callback()
is fed into the CNN created in train.py. The image is then matched to a certain label in the training
dataset, and if a match is found to a certain direction (forward, left or right), the robot is sent a cmd_vel
to move in the specified direction.
"""

class FollowRoad:
  _FORWARD = 0
  _TURNING_LEFT = 1
  _TURNING_RIGHT = 2
 
  def __init__(self, image_topic, cmd_topic, x_vel, theta_vel, output, image_size):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
    self._pub = rospy.Publisher(cmd_topic, Twist, queue_size = 1)
    self._x_vel = x_vel
    self._theta_vel = theta_vel
    self._output = output
    self._image_size = image_size
    self._recording = False
    self._curdir = None
    self._left_id = 0
    self._right_id = 0
    self._forward_id = 0
    self._model = load_model("model")
  
  def _deploy(self, image):
    return np.argmax(self._model.predict(image)) # Predict the direction based on image.

  def _image_callback(self, msg):
    # Initialize the image bridge and display the current camera feed in a window.
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
    cv2.imshow("Image", image) # Display the image in a window.
    key = cv2.waitKey(3) 
    
    """The following section provides an interface to send the robot commands via keyboard input."""
    if key == 106:
      self.turn_left()
    elif key == 107:
      self.go_straight()
    elif key == 108:
      self.turn_right()
    # Start autonomous driving with a keyboard input of 's', stop with 'q'.
    elif key == 115:
      print(f"Autonomous driving activated!")
      self._recording = True
    elif key == 32:
      self._recording = False
      self.stop()
      print(f"Autonomous driving deactivated!")
   
    """The  following section deploys the NN and drives the robot autonomously."""
    if self._recording:
      # Subsample the current image from the camera to scale it down into usable size for NN.
      dim = min(image.shape[0], image.shape[1])
      oy = int((image.shape[0] - dim) / 2)
      ox = int((image.shape[1] - dim) / 2)
      roi = image[oy:oy+dim, ox:ox+dim, 0:3]
      out = cv2.resize(image, (self._image_size, self._image_size))
      im = img_to_array(out) # Convert image to lxwx3 array.
      im = np.array(im, dtype="float") / 255.0 # Normalize the values to 255.
      im = im.reshape(-1, 28, 28, 3) # Reshape for processing.
      # Determine the robot's optimal direction based on predicted output from model.
      predict = self._deploy(im)
      if predict == FollowRoad._FORWARD:
        self.go_straight()
      elif predict == FollowRoad._TURNING_LEFT:
        self.turn_left()
      elif predict == FollowRoad._TURNING_RIGHT:
        self.turn_right()


  def _command(self, x_vel, theta_vel):
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = theta_vel
    self._pub.publish(twist)

  def go_straight(self):
    self._command(self._x_vel, 0)

  def turn_left(self):
    self._command(self._x_vel, self._theta_vel)

  def turn_right(self):
    self._command(self._x_vel, -self._theta_vel)

  def stop(self):
    self._command(0, 0)

if __name__ == '__main__':
  rospy.init_node('drive_by_camera')
  # Gather information on the diff_drive robot that is spawned using cpmr_ch5.
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")
  cmd_topic = rospy.get_param("~cmd", "cmd_vel")
  hz = int(rospy.get_param("~rate", 10))
  output = rospy.get_param("~output", "database")
  x_vel = float(rospy.get_param("~x_vel", 0.2))
  theta_vel = float(rospy.get_param("~theta_vel", 0.2))
  image_size = int(rospy.get_param("~size", 28))
  # Instantiate the FollowRoad class.
  camera = FollowRoad(image_topic, cmd_topic, x_vel, theta_vel, output, image_size)
  rospy.spin()
