#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg
from std_msgs.msg import String


ser = serial.Serial(port="/dev/micro", baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

left_speed = '0'
right_speed = '0'

def controllerCallback(joyMsg):
	global left_speed
	global right_speed
	left_speed = str(abs(int(joyMsg.axes[1] * 35)))
	right_speed = str(abs(int(joyMsg.axes[5] * 35)))

def outputTimerCallback(event):
	ser.write('{' + left_speed.zfill(3) + ' ' + right_speed.zfill(3) + '}') #Format serial message and write it to the device on USB0


""" Synchronizes to the last character of the data sent, prevents issues related to 
incorrect serial data being sent"""
def sync():
    while not rospy.core.is_shutdown():
        if ser.read(1) == '}' :
            break
    
	
def listener():
    rospy.init_node('rc_listener')
    rospy.Subscriber("joy", msg.Joy, controllerCallback)
    rospy.Timer(rospy.Duration(0.2), outputTimerCallback)
    enc_pub = rospy.Publisher("/encoder", String, queue_size=2)


    sync()
    while not rospy.core.is_shutdown():
        enc_data = ser.read(8)	 # Number of bytes from the microcontroller
        if enc_data[-1] != '}':
            sync()
        else:
            enc_pub.publish(String(str(enc_data)))
            print(str(enc_data))


if __name__ == '__main__':
 	listener()
	ser.close()
