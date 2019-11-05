#!/usr/bin/env python

import argparse
import numpy as np
import rospy
import logging
from std_msgs.msg import String, Float32, Float32MultiArray

class LateralNode:
    def __init__(self):
        self.parse_arguments()
        self.create_publisher()
        self.create_subscriber()

    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='Subscriber node for lateral control algorithm.')
        parser.add_argument('-v', '--verbose', dest='verbose', action='store_true')
        parser.set_defaults(verbose=False)
        self.args = parser.parse_args()

    def create_subscriber(self):
        """Creates a subscriber node"""
        logging.debug('initializing Lateral Node subscriber')
        rospy.init_node('lateral_node', anonymous=True)
        sub = rospy.Subscriber('/waypoint/speed', Float32, self.callback)

    def create_publisher(self):
        """Creates a publisher node"""
        logging.debug('initializing Lateral Node publisher')
        self.pub = rospy.Publisher('/lateral/output', String, queue_size=10)

    def callback(self, msg):
        print "Received message!"
        print msg.data
    
if __name__ == "__main__":
    node = LateralNode()
    rospy.spin()