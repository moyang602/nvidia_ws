#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

source  = 'rtsp://192.168.50.50:554/Streaming/Channels/101?transportmode=unicast'

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cf_image_raw",Image, queue_size = 10)

    self.bridge = CvBridge()
    self.cap = cv2.VideoCapture(source) #Open camera one
    success, frame = self.cap.read() #Read one frame
    print("Camera open operation is: ", success)

  def capture(self):

    success, frame = self.cap.read() #Read one frame
    # cv2.imshow("Image window", frame)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)

  try:
    while not rospy.is_shutdown():
      ic.capture()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
