#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray


# Initialize ROS publishers
wp_pub = rospy.Publisher('/autodrive_sim/output/waypoints', Float32MultiArray, queue_size = 1)
stop_dist_pub = rospy.Publisher('/waypoint_planner/stop_distance', Float32, queue_size = 1)

# Main function
if __name__ == '__main__':
	rospy.init_node('straight_wp_pub_node', anonymous = False)
	print('Publishing straight waypoints')
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		msg = Float32MultiArray()
		msg2 = Float32()

		wp_list = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
		points_length = 6
		wp_list[0] = [3,0]
		wp_list[1] = [6,0]
		wp_list[2] = [9,0]
		wp_list[3] = [12,0]
		wp_list[4] = [15,0]
		wp_list[5] = [18,0]

		# wp_list = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
		# points_length = 6
		# wp_list[0] = [2,0]
		# wp_list[1] = [4,0]
		# wp_list[2] = [5,-1]
		# wp_list[3] = [6,-2]
		# wp_list[4] = [6,-4]
		# wp_list[5] = [6,-6]
		# wp_list[6] = [7,-7]
		# wp_list[7] = [8,-8]
		# wp_list[8] = [10,-8]
		# wp_list[9] = [12,-8]



		for waypoint in wp_list:
			msg.data.append(waypoint[0])
			msg.data.append(waypoint[1])

		msg2.data = 25000
		stop_dist_pub.publish(msg2)
		wp_pub.publish(msg)

		rate.sleep()
