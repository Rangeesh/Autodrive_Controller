#!/usr/bin/env python

import rospy
import pickle # Working with binary files
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
from math import sqrt

class PublishWaypoints:
    def __init__(self):
        rospy.init_node("Publish_Novatel_waypoints", anonymous=True)
        self.pub=rospy.Publisher('/autodrive_sim/output/waypoints', Float32MultiArray, queue_size = 1) # Make sure you remove the other one before running
        self.CarPositionX=0
        self.CarPositionY=0
        self.List=[]
        
        with open('GeneratedRoute.data','rb') as filehandle:
            self.List=pickle.load(filehandle)
            # print(self.List)
        self.List1D=[]
        self.rate=rospy.Rate(20)
        self.publishwaypoints()

    def publishwaypoints(self):


        ArrayList=np.asarray(self.List)
        # print(ArrayList)
        # print("\n\n\n\n\n")
        # print(ArrayList.flatten())

        self.List1D=Float32MultiArray(data=ArrayList.flatten())
        while not rospy.is_shutdown():
# # I'm removing the Waypoints that are passed by the Car - Assuming a Rectangular area of 6m (for now it's infinity, i.e. I'm not checking this constraint) (the width of the track)* 20 cm - Center of it is the waypoint
#             # temp=np.asarray(self.List)
#             # x1=temp[0][0]
#             # y1=temp[0][1]
#             # x2=temp[1][0]
#             # y2=temp[1][1]
#             # if abs((y1-y2)*self.CarPositionX + (x2-x1)*self.CarPositionY + x1*y2 - x2*y1)/sqrt((y2-y1)**2+(x2-x1)**2) <= 0.1:
#             #     self.List.pop(0)
#             temp1=Float32MultiArray(data=np.asarray(self.List))
#             print(temp1)

            self.pub.publish(self.List1D)
            self.rate.sleep()

    def callback(self,msg):
        self.CarPositionX=msg.pose.pose.position.x
        self.CarPositionY=msg.pose.pose.position.y




if __name__=="__main__":
    PublishWaypoints()