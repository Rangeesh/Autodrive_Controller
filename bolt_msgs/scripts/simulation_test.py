#!/usr/bin/env python
import argparse
import time
import logging
import rospy
from std_msgs.msg import String, Float32, Float32MultiArray, Int32
from bolt_msgs.msg import BrakeCmd, BrakeReport, SteeringCmd, SteeringReport, ThrottleCmd, ThrottleReport, StopSign, TargetPath

class AutodriveSimulationTest:
    def __init__(self):
        self.parse_cmdline_arguments()

        # Define main topics
        self.input_topic = '/autodrive_sim/input'
        self.output_topic = '/autodrive_sim/output'

        # Setup logging
        logging.basicConfig(level=logging.DEBUG)
        logging.info('Initializing AutodriveSimulationTest')
        rospy.init_node('test_simulation', anonymous=True)

        # Load test image for image_in publisher
        if self.args.image:
            test_image_file = open('test_image.jpg', 'rb')
            image_string = test_image_file.read()
            self.encoded_image_string = image_string.encode('base64')
            print self.encoded_image_string

        # Define the publsh frequency
        self.publish_frequency = 10 # Hz
        self.publish_period = 1.0 / self.publish_frequency
        self.last_publish_time = 0

        # Create Publishers for simulation inputs
        if self.args.status:
            self.status_pub = rospy.Publisher(self.input_topic + '/status',
                String, queue_size=10)
        if self.args.image:
            self.image_pub = rospy.Publisher(self.input_topic + '/image_in',
                String, queue_size=10)
        if self.args.throttle:
            self.throttle_pub = rospy.Publisher(self.input_topic + '/throttle',
                ThrottleCmd, queue_size=10)
        if self.args.brake:
            self.brake_pub = rospy.Publisher(self.input_topic + '/brake',
                BrakeCmd, queue_size=10)
        if self.args.steering:
            self.steering_angle_pub = rospy.Publisher(self.input_topic + '/steering_angle',
                SteeringCmd, queue_size=10)

        # Create Subscribers for simulation outputs
        if self.args.status:
            self.status_sub = rospy.Subscriber(self.output_topic + '/status',
                String, callback=self.status_callback, queue_size=1)
        if self.args.image:
            self.camera_front_sub = rospy.Subscriber(self.output_topic + '/camera_front',
                String, callback=self.camera_front_callback, queue_size=1)
        if self.args.speed:
            self.speed_sub = rospy.Subscriber(self.output_topic + '/speed',
                Float32, callback=self.speed_callback, queue_size=1)
        if self.args.waypoints:
            self.waypoints_sub = rospy.Subscriber(self.output_topic + '/waypoints',
                Float32MultiArray, callback=self.waypoints_callback, queue_size=1)
        if self.args.position:
            self.position_sub = rospy.Subscriber(self.output_topic + '/position',
                Float32MultiArray, callback=self.position_callback, queue_size=1)
        if self.args.heading:
            self.heading_sub = rospy.Subscriber(self.output_topic + '/heading',
                Float32, callback=self.heading_callback, queue_size=1)
        if self.args.angular_velocity:
            self.angular_velocity_sub = rospy.Subscriber(self.output_topic + '/angular_velocity',
                Float32, callback=self.angular_velocity_callback, queue_size=1)
        if self.args.throttle:
            self.throttle_report_sub = rospy.Subscriber(self.output_topic + '/throttle_report',
                ThrottleReport, callback=self.throttle_report_callback, queue_size=1)
        if self.args.brake:
            self.brake_report_sub = rospy.Subscriber(self.output_topic + '/brake_report',
                BrakeReport, callback=self.brake_report_callback, queue_size=1)
        if self.args.steering:
            self.steering_angle_report_sub = rospy.Subscriber(self.output_topic + '/steering_report',
                SteeringReport, callback=self.steering_report_callback, queue_size=1)

    def parse_cmdline_arguments(self):
        parser = argparse.ArgumentParser(description='Autodrive simulation test node')

        parser.add_argument('-v', '--verbose', dest='verbose', action='store_true')
        parser.add_argument('-a', '--all', dest='all', action='store_true')
        parser.add_argument('--status', dest='status', action='store_true')
        parser.add_argument('--image', dest='image', action='store_true')
        parser.add_argument('--speed', dest='speed', action='store_true')
        parser.add_argument('--waypoints', dest='waypoints', action='store_true')
        parser.add_argument('--heading', dest='heading', action='store_true')
        parser.add_argument('--position', dest='position', action='store_true')
        parser.add_argument('--angular_velocity', dest='angular_velocity', action='store_true')
        parser.add_argument('--throttle', dest='throttle', action='store_true')
        parser.add_argument('--brake', dest='brake', action='store_true')
        parser.add_argument('--steering', dest='steering', action='store_true')

        parser.set_defaults(all=False) 
        parser.set_defaults(verbose=False)
        parser.set_defaults(status=False)
        parser.set_defaults(image=False)
        parser.set_defaults(speed=False)
        parser.set_defaults(waypoints=False)
        parser.set_defaults(heading=False)
        parser.set_defaults(position=False)
        parser.set_defaults(angular_velocity=False)
        parser.set_defaults(throttle=False)
        parser.set_defaults(brake=False)
        parser.set_defaults(steering=False)

        self.args = parser.parse_args()
        if self.args.all:
            self.args.status = True
            self.args.speed = True
            self.args.image = True
            self.args.waypoints = True
            self.args.heading = True
            self.args.position = True
            self.args.angular_velocity = True
            self.args.throttle = True
            self.args.brake = True
            self.args.steering = True

    def status_callback(self, data):
        if self.args.status:
            logging.debug('Got status')

    def camera_front_callback(self, data):
        if self.args.image:
            logging.debug('Got camera_front')

    def speed_callback(self, data):
        if self.args.speed:
            logging.debug('Got speed')

    def waypoints_callback(self, data):
        if self.args.waypoints:
            logging.debug('Got waypoints')
  
    def position_callback(self, data):
        if self.args.position:
            logging.debug('Got position')

    def heading_callback(self, data):
        if self.args.heading:
            logging.debug('Got heading')

    def angular_velocity_callback(self, data):
        if self.args.angular_velocity:
            logging.debug('Got angular_velocity')

    def throttle_report_callback(self, data):
        if self.args.throttle:
            logging.debug('Got throttle report')

    def brake_report_callback(self, data):
        if self.args.brake:
            logging.debug('Got brake report')

    def steering_report_callback(self, data):
        if self.args.steering:
            logging.debug('Got steering report')

    def should_publish(self):
        current_time = time.time()
        diff = current_time - self.last_publish_time
        if diff > self.publish_period:
            self.last_publish_time = current_time
            return True
        return False

    def start(self):
        while not rospy.is_shutdown():
            if self.should_publish():
                self.publishers()

    def publishers(self):
        if self.args.status:
            status_msg = String()
            status_msg.data = "testing status in"
            self.status_pub.publish(status_msg)

        if self.args.image:
            image_msg = String()
            image_msg.data = self.encoded_image_string
            self.image_pub.publish(image_msg)

        if self.args.throttle:
            throttle_msg = ThrottleCmd()
            throttle_msg.enable = True
            throttle_msg.throttle_cmd = 0.5
            self.throttle_pub.publish(throttle_msg)

        if self.args.brake:
            brake_msg = BrakeCmd()
            brake_msg.enable = True
            brake_msg.brake_cmd = 0.25
            self.brake_pub.publish(brake_msg)

        if self.args.steering:
            steering_msg = SteeringCmd()
            steering_msg.enabled = True
            steering_msg.steering_wheel_angle_cmd = 10
            self.steering_angle_pub.publish(steering_msg)

if __name__ == '__main__':
    try:
        sim_test = AutodriveSimulationTest()
        sim_test.start()
    except rospy.ROSInterruptException:
        pass