#!/usr/bin/python3

# standard imports
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import time
import signal
import os
import argparse

# ros imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool

class ResizeImage:
    def __init__(self, topic, size):
        """default constructor"""

        # variables for output
        self.active = True
        self.bridge = CvBridge()
        self.size = size
        self.topic = topic

        if(self.topic[len(self.topic) - 1] != "/"):
            self.topic += "/"

        print("Resize Image - Resizing image topic " + self.topic + " to " + str(self.size) + " by " + str(self.size)) 
        print("Resize Image - Publishing resized image to " + self.topic + 'resized')

        # initialize ros subscriber and publishers
        rospy.init_node("Resize_Image_Node")
        rospy.Subscriber(self.topic, Image, self.image_callback, queue_size = 1)
        self.image_resize = rospy.Publisher(self.topic + 'resized', Image, queue_size = 1)

    def image_callback(self, msg):
        """ros callback function for image"""

        # check if image data exists
        if msg.data is None:
            return

        # convert to cv2 mat type
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # resize image and publish
        image = self.resize_image(image, self.size)
        self.image_resize.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def resize_image(self, img, target_size=800):
        """resize image to target_size by target_size"""

        # get image properties
        target_size = float(target_size)
        height = img.shape[0]
        width = img.shape[1]

        # resize image based on scaling factor
        if height > width:
            scale_factor = target_size / height
            img = cv2.resize(img, (int(width * scale_factor), int(height * scale_factor)))
        else:
            scale_factor = target_size / width
            img = cv2.resize(img, (int(width * scale_factor), int(height * scale_factor)))

        return img

    def show_single_image(self, name, image):
        """show cv2 mat image"""
        cv2.imshow(name, image)
        cv2.waitKey(1)

    def signalInterruptHandler(self, signum, frame):
        """developer signal interrupt handler, immediate stop of ros node"""

        print("Resize Image - Node Interrupted...")

        self.active = False
        sys.exit()

if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", default="/front_camera/image_raw")
    parser.add_argument("-s", "--size", default=800)
    args = parser.parse_args()   

    # start resizing the image given
    node = ResizeImage(args.image, args.size)
    
    # to stop the program from ending
    rospy.spin()
