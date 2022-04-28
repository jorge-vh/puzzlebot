#!/usr/bin/env python2
# Library that contains the basic files for ros to work
import rospy
from math import sin,cos,tan,atan2,pi
import math
import numpy
# Library that contains standard ROS msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D,Twist

class control:
    def __init__(self):
        self.wr=0
        self.wl=0
        self.r =0.05
        self.l =0.19
        self.loc=Pose2D()
        self.loc.x=0
        self.loc.y=0
        self.loc.theta=0
        self.dt=0
        self.goalY=0
        self.goalX=-0.8
        self.etheta=0
        self.ed=0
        self.transform=Twist()
        self.transform.linear.x=0
        self.transform.linear.y=0
        self.transform.linear.z=0
        self.transform.angular.x=0
        self.transform.angular.y=0
        self.transform.angular.z=0
        self.kv=0.3
        self.kw=0.1

    def calculatePosition(self):
        self.loc.theta=self.loc.theta+self.r*((self.wr-self.wl)/self.l)*self.dt
        if self.loc.theta >= numpy.pi:
            self.loc.theta=-numpy.pi
        if self.loc.theta < -numpy.pi:
            self.loc.theta= numpy.pi
        self.loc.x=self.loc.x+self.r*((self.wr+self.wl)/2)*self.dt*cos(self.loc.theta)
        self.loc.y=self.loc.y+self.r*((self.wr+self.wl)/2)*self.dt*sin(self.loc.theta)


    def calculateErrors(self):
        if self.loc.theta >= 0:
            self.etheta=numpy.arctan2(self.goalY,self.goalX)-self.loc.theta
        elif self.loc.theta < 0 :
            self.etheta=-(numpy.arctan2(self.goalY,self.goalX)+self.loc.theta)
        self.ed=numpy.sqrt(math.pow(self.goalX-self.loc.x,2)+math.pow(self.goalY-self.loc.y,2))

    def calculateVel(self):
        vel=self.ed*self.kv
        velW=self.etheta*self.kw
        self.transform.linear.x=vel
        self.transform.angular.z=velW

pb=control()

def callWl(data):
    pb.wl= data.data


def callWr(data):
    pb.wr=data.data

def localize():
    rospy.init_node('localization',anonymous=True)
    rospy.Subscriber('wl', Float32, callWl)
    rospy.Subscriber('wr', Float32, callWr)

    pubETheta=rospy.Publisher('etheta',Float32,queue_size=10)
    pubED=rospy.Publisher('ed',Float32,queue_size=10)
    pubCoord=rospy.Publisher('coord', Pose2D, queue_size=10)
    pubVel=rospy.Publisher('cmd_vel',Twist,queue_size=10)

    base=rospy.get_time()
    now=rospy.get_time()
    desp=numpy.sqrt(math.pow(pb.goalX,2)+math.pow(pb.goalY,2))
    while not rospy.is_shutdown():
        now=rospy.get_time()
        pb.dt=now-base
        if pb.dt >= 0.01:
            pb.calculatePosition()
            pb.calculateErrors()
            pb.calculateVel()
            print(pb.transform)
            print(pb.ed)

            if pb.etheta > 0.03:
                pb.transform.linear.x=0
            if pb.ed < desp*.05:
                pb.transform.linear.x=0
                pb.transform.angular.z=0
            pubETheta.publish(pb.etheta)
            pubED.publish(pb.ed)
            pubCoord.publish(pb.loc)
            pubVel.publish(pb.transform)
            base=rospy.get_time()

if __name__ == '__main__':
    localize()
