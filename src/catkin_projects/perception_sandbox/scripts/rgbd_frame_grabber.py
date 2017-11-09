#!/usr/bin/env python

import roslib
import sys
import rospy
import time
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

from spartan.utils.cv_utils import *

class Grabber:
  def __init__(self, camera_name = "camera_1112170110"):
    self.bridge = CvBridge()
    prefix = "/" + camera_name + "/"
    
    self.rgb_image_sub = rospy.Subscriber(prefix + "rgb/image_rect_color", Image, self.rgb_callback)
    self.depth_image_sub = rospy.Subscriber(prefix + "depth_registered/sw_registered/image_rect", Image, self.depth_callback)

    self.latest_rgb_image = np.zeros((480, 640, 3))
    self.latest_rgb_image_ts = time.time() - 1000
    self.latest_depth_image = np.zeros((480, 640, 1))
    self.latest_depth_image_ts = time.time() - 1000

  def rgb_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.latest_rgb_image = cv_image
    self.latest_rgb_image_ts

  def depth_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
      print(e)

    self.latest_depth_image = cv_image
    self.latest_depth_image_ts


  def update(self):
    tiled_image = generateGridOfImages((self.latest_rgb_image, generateColorMap(self.latest_depth_image, 0.0, 10.0)), 2, 10)
    cv2.imshow("herp", tiled_image)

def main(args):
  print("Notice: until upgraded, this script does not produced *perfectly synchronized*")
  print("point clouds and frames. The grouped data is from close in time, but not exactly")
  print("the same frame, necessarily.")

  gr = Grabber("camera_1112170110")
  rospy.init_node('rgbd_frame_grabber', anonymous=True)
  try:
    while rospy.is_shutdown() is not True:
      gr.update()
      if cv2.waitKey(33) == ord('s'):
        print "SAVE REQUESTED BUT NOT YET IMPLEMENTED"

  except KeyboardInterrupt:
    print("Exception ", e)
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)