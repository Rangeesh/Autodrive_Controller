#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

#markerArray = MarkerArray()

def waypoint_cb(msg):
	markerArray = MarkerArray()
	for i in range((len(msg.data)/2)):
		marker = Marker()
		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.z = 0
		marker.pose.position.x = msg.data[2*i]
		marker.pose.position.y = msg.data[2*i+1]
		marker.id = i
		markerArray.markers.append(marker)
		last_i = i

	if len(msg.data) > 2:
		for i in range((len(msg.data)/2)-1):
			line_start = Point()
			line_start.x = msg.data[2*i]
			line_start.y = msg.data[2*i+1]
			line_end = Point()
			line_end.x = msg.data[2*i+2]
			line_end.y = msg.data[2*i+3]

			marker = Marker()
			marker.header.frame_id = "odom"
			marker.type = marker.LINE_STRIP
			marker.action = marker.ADD
			marker.scale.x = 0.05
			marker.scale.y = 0.05
			marker.scale.z = 0.05
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.points = [line_start, line_end]
			marker.pose.orientation.w = 1.0
			marker.id = i + 1 + last_i
			markerArray.markers.append(marker)


	# Publish the MarkerArray
	publisher.publish(markerArray)

def waypoint_cb2(msg):
	markerArray2 = MarkerArray()
	for i in range((len(msg.data)/2)):
		marker = Marker()
		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 1.1
		marker.scale.y = 1.1
		marker.scale.z = 1.1
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 0.0
		marker.color.b = 1.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.z = 0
		marker.pose.position.x = msg.data[2*i]
		marker.pose.position.y = msg.data[2*i+1]
		marker.id = i
		markerArray2.markers.append(marker)
		last_i = i

	if len(msg.data) > 2:
		for i in range((len(msg.data)/2)-1):
			line_start = Point()
			line_start.x = msg.data[2*i]
			line_start.y = msg.data[2*i+1]
			line_end = Point()
			line_end.x = msg.data[2*i+2]
			line_end.y = msg.data[2*i+3]

			marker = Marker()
			marker.header.frame_id = "odom"
			marker.type = marker.LINE_STRIP
			marker.action = marker.ADD
			marker.scale.x = 0.4
			marker.scale.y = 0.4
			marker.scale.z = 0.4
			marker.color.a = 1.0
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			marker.points = [line_start, line_end]
			marker.pose.orientation.w = 1.0
			marker.id = i + 1 + last_i
			markerArray2.markers.append(marker)


	# Publish the MarkerArray
	publisher2.publish(markerArray2)


if __name__ == "__main__":
	rospy.init_node('visualize_waypoints_node')
	publisher = rospy.Publisher('waypoint_marker_array', MarkerArray, queue_size=1)
	publisher2 = rospy.Publisher('nonactive_waypoint_marker_array', MarkerArray, queue_size=1)
	rospy.Subscriber("autodrive_sim/output/waypoints", Float32MultiArray, waypoint_cb)
	rospy.Subscriber("nonactive_autodrive_sim/output/waypoints", Float32MultiArray, waypoint_cb2)
	rospy.spin()
