#!/usr/bin/env python

import signal
import tf
from math import sqrt
import sys
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
#from teb_local_planner.msg import FeedbackMsg
from geometry_msgs.msg import Twist

# Created By: Ben Swain and Juan Vasquez

# Initialize ROS node
rospy.init_node('bolt_gps_node', anonymous = False)

# Initialize ROS publishers
heading_pub = rospy.Publisher("autodrive_sim/output/heading", Float32, queue_size = 1)
ang_vel_pub = rospy.Publisher("autodrive_sim/output/angular_velocity", Float32, queue_size = 1)
position_pub = rospy.Publisher("autodrive_sim/output/position", Float32MultiArray, queue_size = 1)
imu_pub = rospy.Publisher("/vehicle/imu/data_raw", Imu, queue_size = 1)
#desired_vel_pub = rospy.Publisher("/vehicle/desire_v", Float32, queue_size = 1)
forward_speed_pub = rospy.Publisher("/autodrive_sim/output/speed", Float32, queue_size=1)
#for old rosbags
#odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

# Initialize odom
odom_broadcaster = tf.TransformBroadcaster()

# Callback for odom message
def odom_to_heading_callback(odom_msg):
    # ensure we get the same instance of all of these variables
    global heading_pub, position_pub, forward_speed_pub


    # define the important variables
    orientation_q = odom_msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    heading = Float32(data = yaw)
    #heading.data = yaw

    # define the position
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    pos_array = [x,y]
    pos_msg = Float32MultiArray(data = pos_array)

    # define the forward speed
    v_x = odom_msg.twist.twist.linear.x
    v_y = odom_msg.twist.twist.linear.y
    v_norm = sqrt((v_x)**2 + (v_y)**2)
    speed = Float32()
    speed.data = v_norm

    '''
    ### ONLY FOR OLD ROSBAGS ###
    # tf odom broadcaster
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #odom_quat = odom_msg.pose.pose.orientation
    current_time = rospy.Time.now()
    odom_broadcaster.sendTransform(
    (y, x, 0.),
    odom_quat,
    current_time,
    "base_link",
    "odom"
    )

    # rotate by 90 deg
    x_backup = odom_msg.pose.pose.position.x
    odom_msg.pose.pose.position.x = odom_msg.pose.pose.position.y
    odom_msg.pose.pose.position.y = x_backup
    xt_backup = odom_msg.twist.twist.linear.x
    odom_msg.twist.twist.linear.x = odom_msg.twist.twist.linear.y
    odom_msg.twist.twist.linear.y = xt_backup
    odom_pub.publish(odom_msg)
    ### ^FOR OLD ROSBAGS^ ###
    '''

    # publish the messages
    heading_pub.publish(heading)
    position_pub.publish(pos_msg)
    forward_speed_pub.publish(speed)

# ROS callback for angular velocity
def imu_to_angular_velocity_callback(imu_msg):
    # ensure we get the same instance of all of these variables
    global count2

    # define the angular velocity
    #ang_vel_x = imu_msg.angular_velocity.x
    #ang_vel_y = imu_msg.angular_velocity.y
    ang_vel_z = imu_msg.angular_velocity.z
    #ang_vel_norm = sqrt((ang_vel_x)**2 + (ang_vel_y)**2)

    ang_vel_msg = Float32(data = ang_vel_z)
    #ang_vel_msg.data = ang_vel_z

    #define linear acceleration
    acc_x = imu_msg.linear_acceleration.x
    acc_y = imu_msg.linear_acceleration.y
    acc_norm = sqrt((acc_x)**2 + (acc_y)**2)

    lin_acc_msg = Float32()
    lin_acc_msg.data = acc_norm

    # publish the message
    ang_vel_pub.publish(ang_vel_msg)

# ROS callback for desired velocity
def cmdvel_to_desiredvel_callback(cmdvel_msg):
    # define cmd_vel
    cmdvel_x = cmdvel_msg.linear.x

    desired_vel_msg = Float32(data = desired_vel_msg)
    #desired_vel_msg.data = desired_vel_msg

    # publish the message
    #desired_vel_pub.publish(desired_vel_msg)

# ROS subscriber function
def subscriber():
	rospy.Subscriber("/navsat/odom", Odometry, odom_to_heading_callback)
	rospy.Subscriber("/imu/data", Imu, imu_to_angular_velocity_callback)
	#rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, feedback_to_desired_velocity_callback)
	#rospy.Subscriber("/cmd_vel", Twist, cmdvel_to_desiredvel_callback)
	print "GPS Converter - Subscribers Active!"

	rospy.spin()

# Developer signal interrupt handler, immediate stop of program
def signalInterruptHandler(signum, frame):
    print "GPS Converter -", signum, "- Interrupting Program..."
    sys.exit()

# Main function
if __name__ == '__main__':
    # register CONTROL+C signal
	signal.signal(signal.SIGINT, signalInterruptHandler)

    # start subscriber
	subscriber()
