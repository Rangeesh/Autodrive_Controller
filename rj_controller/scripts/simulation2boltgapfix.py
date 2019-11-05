#!/usr/bin/env python

import roslib
roslib.load_manifest('combined_controller')
import rospy
from bolt_msgs.msg import ThrottleCmd
from bolt_msgs.msg import BrakeCmd
from bolt_msgs.msg import SteeringCmd

from std_msgs.msg import Int64
from std_msgs.msg import Float32

from nav_msgs.msg import Odometry

rospy.init_node('gap_fix_node', anonymous = True)
#send data to simulation
throttle_pub = rospy.Publisher('/autodrive_sim/input/throttle',ThrottleCmd,queue_size = 1)
brake_pub = rospy.Publisher("/autodrive_sim/input/brake",BrakeCmd,queue_size = 1)
# steering_pub = rospy.Publisher("/autodrive_sim/input/steering_angle",SteeringCmd,queue_size = 1)
odom_pub = rospy.Publisher("/odom",Odometry, queue_size = 1)


def throttle_cb(data):
    throttle_data = ThrottleCmd()
    throttle_data.throttle_cmd = data.data
    throttle_pub.publish(throttle_data)
    #rospy.loginfo(throttle_data.throttle_cmd)

# def steering_cb(data):
#     #rospy.loginfo(data.data)
#     steering_data = SteeringCmd()
#     steering_data.steering_wheel_angle_cmd = data.data/15.4
#     steering_pub.publish(steering_data)


def brake_cb(data):
    #rospy.loginfo(data.data)
    brake_data = BrakeCmd()
    brake_data.brake_cmd = data.data
    brake_pub.publish(brake_data)

def speed_cb(data):
    odom_data = Odometry()
    odom_data.twist.twist.linear.x = data.data
    odom_data.twist.twist.linear.y = 0.0 #No need, just because
    odom_data.twist.twist.linear.z = 0.0
    odom_pub.publish(odom_data)

def gap_fix():

    #get data from controller
    rospy.Subscriber("/can_throttle_auto", Int64, throttle_cb)
    # rospy.Subscriber("/can_steering_auto", Int64, steering_cb)
    rospy.Subscriber("/can_brake_auto", Int64, brake_cb)
    rospy.Subscriber("/autodrive_sim/output/speed", Float32, speed_cb)

    rospy.spin()

if __name__ == '__main__':
        gap_fix()