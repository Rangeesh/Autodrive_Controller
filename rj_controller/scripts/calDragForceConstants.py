#!/usr/bin/env python

import roslib
roslib.load_manifest('combined_controller')
import rospy
import math
import numpy as np
from numpy.linalg import inv
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from bolt_msgs.msg import SteeringReport
class dragForce:
    def __init__(self):
        self.throttle_topic = "can_throttle_auto"
        self.brake_topic = "can_brake_auto"
        self.steering_topic = "can_steering_auto"
        self.throttle_pub = rospy.Publisher(self.throttle_topic,Int64,queue_size = 1)
        self.brake_pub = rospy.Publisher(self.brake_topic,Int64,queue_size = 1)
        self.steering_pub = rospy.Publisher(self.steering_topic,Int64,queue_size = 1)

        rospy.Subscriber("/autodrive_sim/output/imu",Imu, self.imu_cb)
        #rospy.Subscriber("/autodrive_sim/output/steering_report", SteeringReport,self.steering_report_cb)
        rospy.Subscriber("/navsat/odom",Odometry,self.vel_cb )
        self.throttle_data = 1000
        self.brake_data = 0
        self.steering_data = 0
        self.is_slow_down = False
        self.starting_time = rospy.get_time() # in second

        
        self.vel = 0.0
        self.prev_vel = 0.0
        self.mass = 1605.0
        self.g = 9.81
        self.a = 1.135
        self.b = 1.465
        self.theta = 0.0 # grade angle get from IMU
        self.prev_theta = 0.0
        self.lat_acceleration = 0.0 # from IMU also
        self.prev_lat_acceleration = 0.0
        self.delta = 0.0  # wheel angle
        self.prev_delta = 0.0
        self.long_acceleration = 0.0 # from IMU
        self.prev_long_acceleration = 0.0
        while not rospy.is_shutdown():
            #drive for 
            #self.throttle_pub.publish(self.throttle_data)
            rospy.sleep(.5)
            #if rospy.get_time() - self.starting_time > 5: #seconds
            #    self.throttle_data = 0
            self.constants_cal()
            #    if self.is_slow_down == False:
            #        rospy.loginfo("Slowing down, start estimating constants")
            #        self.is_slow_down = True 
            self.values_update()
            #if rospy.get_time() - self.starting_time > 10:
            #        rospy.loginfo("Done calculation")
            #        return
    def values_update(self):
        self.prev_vel = self.vel
        self.prev_theta = self.theta
        self.prev_lat_acceleration = self.lat_acceleration
        self.prev_long_acceleration = self.long_acceleration
        self.prev_delta = self.prev_theta
    def rad2degree(self, rad):
        return rad *180/math.pi
    def constants_cal(self):
        if self.vel == self.prev_vel:
            rospy.loginfo("shit")
            return
        #A = np.matrix([[1 self.prev_vel*self.prev_vel],[1 self.vel*self.vel]])
        A = np.ones((2,2))
        A[0,1] = self.prev_vel**2
        A[1,1] = self.vel**2
        #rospy.loginfo(A)
        inv_A = inv(A)
        #rospy.loginfo(inv_A)
        B = np.zeros((2,1))
        B[0,0] = - self.mass * self.g * self.rad2degree(math.cos(self.prev_theta)) + self.mass*(self.b/(self.a+self.b))*( self.prev_lat_acceleration * self.rad2degree(math.tan(self.prev_delta))) - self.mass*self.prev_long_acceleration
        B[1,0] = - self.mass * self.g * self.rad2degree(math.cos(self.theta)) + self.mass*(self.b/(self.a+self.b))*( self.lat_acceleration* self.rad2degree(math.tan(self.delta))) - self.mass*self.long_acceleration
        X = np.dot(inv_A,B)
        rospy.loginfo(X)
        #update values
        self.values_update()

    def imu_cb(self, data):
        q_list =[data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        e = euler_from_quaternion(q_list)
        self.theta = e[1]*180/math.pi# pitch convert to degree
        #self.theta = 0.0
        self.lat_acceleration = data.linear_acceleration.x
        self.long_acceleration = data.linear_acceleration.y
        print("Lat: %0.2f", self.lat_acceleration)
        print("Long: %0.2f", self.long_acceleration)
        #rospy.loginfo("new IMU")
    #def steering_report_cb(self, data):
        #self.delta = 0.0
    #    self.delta = data.steering_wheel_angle
        #rospy.loginfo("new Steering")
    def vel_cb(self,data):
        self.vel = data.twist.twist.linear.y
if __name__ == '__main__':
    rospy.init_node('drag_constants_node')
    rospy.loginfo("Speeding up")
    df = dragForce()

