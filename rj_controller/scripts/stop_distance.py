#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
class Distance:
    def __init__(self):
        self.pose_x =0.0
        self.pose_y =0.0
        self.max_distance = 50 # meter
        self.pub_distance = rospy.Publisher("/waypoint_planning/stop_distance",Float32,queue_size=1)
        self.sub_odom = rospy.Subscriber("odom",Odometry,self.odom_cb)
        while not rospy.is_shutdown():
            self.distance_pub()
            rospy.sleep(0.02)
            #rospy.spinOnce()
    def odom_cb(self,data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

    def distance_pub(self):
        travel_distance = math.sqrt(self.pose_x**2 + self.pose_y**2)
        msg = Float32()
        temp = self.max_distance - travel_distance
        print 'distance to stop: ',temp
    
        msg.data = temp
        self.pub_distance.publish(msg)
if __name__ =='__main__':
    rospy.init_node('stop_distance_node')
    dt = Distance()