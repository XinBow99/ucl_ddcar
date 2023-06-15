#!/usr/bin/env python
#coding:utf-8

import rospy
import serial
from std_msgs.msg import Int32

ser = None

def callback_left(data):
    global ser
    # Get the PWM value from the message
    pwm = data.data

    # Send the PWM value to the serial port
    cmd = 'AT+setLpwm=' + str(pwm) + '\r\n'
    ser.write(cmd.encode('utf-8'))

def callback_right(data):
    global ser
    # Get the PWM value from the message
    pwm = data.data

    # Send the PWM value to the serial port
    cmd = 'AT+setRpwm=' + str(pwm) + '\r\n'
    ser.write(cmd.encode('utf-8'))

def talker():
    global ser
    # Get serial port and baud rate from parameters
    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', 115200)

    # Open serial port
    ser = serial.Serial(serial_port, baud_rate)

    pub_left = rospy.Publisher('left_rpm', Int32, queue_size=10)
    pub_right = rospy.Publisher('right_rpm', Int32, queue_size=10)
    rospy.init_node('car_motherboard', anonymous=True)
    rospy.Subscriber('setLpwm', Int32, callback_left)
    rospy.Subscriber('setRpwm', Int32, callback_right)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Read response from serial port
        data = ser.readline().decode('utf-8').strip()
        rospy.loginfo(data)

        # Parse the response and publish to ROS topics
        values = data.split(',')
        if len(values) == 2:
            left_rpm = int(values[0])
            right_rpm = int(values[1])
            pub_left.publish(left_rpm)
            pub_right.publish(right_rpm)

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

