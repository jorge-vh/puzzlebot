#!/usr/bin/env python
#EQUIPO 5
from __future__ import print_function

import roslib
roslib.load_manifest('cvtest')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Clase utilizada para convertir la imagen
class image_converter:

  def __init__(self):
    #Iniciamos un publicador para ver la imagen que tenemos
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
    #Bridge para conseguir la imagen
    self.bridge = CvBridge()
    #Subscriber para obtener la imagen en tiempo real
    self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.callback)
    #Subscriber para la velocidad del robot
    self.pubVel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    self.t=Twist()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      #img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      cv_image_blur = cv2.GaussianBlur(cv_image,(5,5),0) #Aplicando un blur a la imagen
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #Cambiando el espacio del color
      circles_find = 0

      #Rangos de color rojo
      lower_red1 = np.array([0, 100, 20], np.uint8)
      upper_red1 = np.array([8, 255, 255], np.uint8)
      lower_red2=np.array([175, 100, 20], np.uint8)
      upper_red2=np.array([179, 255, 255], np.uint8)

      #Rangos de color verde
      lower_green1 = np.array([45, 90, 90])
      upper_green1 = np.array([75, 255, 255])
      lower_green2 = np.array([36, 52, 72])
      upper_green2 = np.array([86, 255, 255])

      #Union de mascaras de color rojo (mas preciso)
      maskRed1 = cv2.inRange(hsv, lower_red1, upper_red1)
      maskRed2 = cv2.inRange(hsv, lower_red2, upper_red2)
      maskRed = cv2.add(maskRed1, maskRed2)

      #Union de mascaras de color verde
      maskGreen1 = cv2.inRange(hsv, lower_green1, upper_green1)
      maskGreen2 = cv2.inRange(hsv, lower_green2, upper_green2)
      maskGreen = cv2.add(maskGreen1, maskGreen2)

      #Imagen con filtro para solo colores verdes
      res_green = cv2.bitwise_and(cv_image,cv_image, mask= maskGreen1)
      #Imagen con filtro para solo colores rojos
      res_red = cv2.bitwise_and(cv_image,cv_image, mask= maskRed)

      #Cambio de espacio de color de HVS A BGR
      res_green_bgr = cv2.cvtColor(res_green, cv2.COLOR_HSV2BGR)
      res_red_bgr = cv2.cvtColor(res_red, cv2.COLOR_HSV2BGR)

      #Cambio de espacio de color de BGR A GRAY
      res_green_gray = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
      res_red_gray = cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY)

      #Funcion de Hough para crear circulos con gradiente, este
      #detecta circulos mediante los parametros del radio
      circles_red = cv2.HoughCircles(res_red_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=60,maxRadius=100)
      circles_green = cv2.HoughCircles(res_green_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=60,maxRadius=100)

      #if para ver si se detectan circulos de color rojo
      if circles_red is not None:
          circles_find = circles_find + 1
          self.t.linear.x=0 #vamos un alto en la velocidad lineal
          print("Se ha detectado un circulo color rojo")
      #if para ver si se detectan circulos de color rojo
      if circles_green is not None:
          circles_find = circles_find + 1
          self.t.linear.x=0.3 #vamos una velocidad en la velocidad lineal
          print("Se ha detectado un circulo color verde")

    except CvBridgeError as e:
      print(e)

    self.pubVel.publish(self.t) #Publicamos la velocidad en base a la desicion

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window green", res_green) #Mostramos el filtrado de color verde
    cv2.imshow("Image window red", res_red) #Mostramos el filtrado de color rojo

    cv2.waitKey(3)

    try:
      #Mandamos la imagen en un publicador mediante el bridge
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True) #iniciamos el nodo de imagen
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
