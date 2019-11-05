#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def vel_pub():
    pub = rospy.Publisher('vehicle/desire_v',Float32MultiArray, queue_size = 1)
    rospy.init_node('desire_v_node', anonymous=True)
    rate = rospy.Rate(10) #10Hz
    output = Float32MultiArray()
    output.data.append(11.0)
    while not rospy.is_shutdown():
        pub.publish(output)
        rate.sleep()

if __name__ == '__main__':
    try:
        vel_pub()
    except rospy.ROSInterruptException:
        pass