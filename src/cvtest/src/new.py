#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvtest')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
      #img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      cv_image_blur = cv2.GaussianBlur(cv_image,(5,5),0) #Aplicando un blur a la imagen 
      
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      
      redBajo1 = np.array([0, 100, 20], np.uint8)
      redAlto1 = np.array([8, 255, 255], np.uint8)
      redBajo2=np.array([175, 100, 20], np.uint8)
      redAlto2=np.array([179, 255, 255], np.uint8)
      
      lower_green = np.array([35, 30, 110])
      upper_green = np.array([60, 255, 255])
      
      maskRed1 = cv2.inRange(hsv, redBajo1, redAlto1)
      maskRed2 = cv2.inRange(hsv, redBajo2, redAlto2)
      maskRed = cv2.add(maskRed1, maskRed2)
      
      mask_green = cv2.inRange(hsv, lower_green, upper_green)
      
      res_green = cv2.bitwise_and(cv_image,cv_image, mask= mask_green)
      res_red = cv2.bitwise_and(cv_image,cv_image, mask= mask_Red)
      
      
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window green", res_green)
    cv2.imshow("Image window red", res_red)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print(e)

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