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
    # 修改為AT Command
    # 設定左輪的PWM值
    # 注意: AT Command的格式為: AT+setLpwm=xxx
    # xxx 是一個整數值
    cmd = 填入AT命令 + '\r\n'
    ser.write(cmd.encode('utf-8'))

def callback_right(data):
    global ser
    # Get the PWM value from the message
    pwm = data.data

    # Send the PWM value to the serial port
    # 設定右輪的PWM值
    cmd = 填入AT命令 + '\r\n'
    ser.write(cmd.encode('utf-8'))

def talker():
    global ser
    # Get serial port and baud rate from parameters
    # 取得參數中的serial_port和baud_rate
    # 若參數中沒有設定，則使用預設值
    # 參數的設定可以在launch檔中進行
    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', 115200)

    # Open serial port
    ser = serial.Serial(serial_port, baud_rate)

    ##########################################
    # Publish to ROS topics
    # 請同學新增兩個Publisher
    # 一個是left_rpm，一個是right_rpm
    #　範例程式碼如下
    # pub_SOMETHING = rospy.Publisher('Topic Name', Int32, queue_size=10)
    # Int32是傳送的資料型態，queue_size是設定的buffer大小
    ##########################################
    "撰寫Publisher程式碼"
    # 初始化ROS節點，並設定為匿名節點，不會與其他節點衝突，因為同時只有一個車子
    rospy.init_node('car_motherboard', anonymous=True)
    ##########################################
    # Subscribe to ROS topics
    # 請同學新增兩個Subscriber
    # 一個是setLpwm，一個是setRpwm
    # 當接收到訊息時，會執行callback_left或callback_right
    # 範例程式碼如下
    # rospy.Subscriber('Topic Name', Int32, callback)
    "撰寫Subscriber程式碼"
    
    # 設定ROS的迴圈頻率
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Read response from serial port
        data = ser.readline().decode('utf-8').strip()
        rospy.loginfo(data)

        # Parse the response and publish to ROS topics
        values = data.split(',')
        # 若接收到的資料有兩個值，則分別發布到left_rpm和right_rpm
        # 因為值是left_rpm,right_rpm : 0,0
        if len(values) == 2:
            left_rpm = int(values[0])
            right_rpm = int(values[1])
            ##########################################
            # Publish to ROS topics
            # 請同學使用上述新增的Publisher發布資料，資料為left_rpm和right_rpm
            # 範例程式碼如下
            # pub_SOMETHING.publish(SOMETHING)
            ##########################################
            "撰寫Publisher程式碼以發布left_rpm和right_rpm"

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

