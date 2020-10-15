#!/usr/bin/env python

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import matplotlib.pyplot as plt

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("depth",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
      print(e)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    print("=========")
    print("max ", np.max(cv_image.data))
    print("min ", np.min(cv_image.data))

    plt.imshow(cv_image)
    plt.show()


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
