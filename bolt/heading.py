#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import math

rospy.init_node("Heading_Degrees")
pub=rospy.Publisher("heading_angle_deg",Float32, queue_size=1)

def subscriber():
	rospy.Subscriber("autodrive_sim/output/heading", Float32,headingCallBack, queue_size=1)
	print("This will be printed once")
	rospy.spin() #prevents it from exiting this function

def headingCallBack(msg):
	a = Float32()
	a.data = msg.data*180/3.14
	pub.publish(a)



	
if __name__=="__main__":
	subscriber()
	print(" this won't be printed")



