#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 115200)

def talker():
    pub_left = rospy.Publisher('left_rpm', String, queue_size=10)
    pub_right = rospy.Publisher('right_rpm', String, queue_size=10)
    rospy.init_node('rpm_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Read response from serial port
        data = ser.readline().decode('utf-8').strip()
        rospy.loginfo(data)

        # Parse the response and publish to ROS topics
        values = data.split(',')
        if len(values) == 2:
            left_rpm = values[0]
            right_rpm = values[1]
            pub_left.publish(left_rpm)
            pub_right.publish(right_rpm)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

ser.close()

