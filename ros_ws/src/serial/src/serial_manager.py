#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

left_speed = '0'
right_speed = '0'

def controllerCallback(joyMsg):
	global left_speed
	global right_speed
	left_speed = str(abs(int(joyMsg.axes[1] * 35)))
	right_speed = str(abs(int(joyMsg.axes[5] * 35)))

def outputTimerCallback(event):
	ser.write('{' + left_speed.zfill(3) + ' ' + right_speed.zfill(3) + '}') #Format serial message and write it to the device on USB0

	
def listener():
	rospy.init_node('rc_listener', anonymous=True)
	rospy.Subscriber("joy", msg.Joy, controllerCallback)
	rospy.Timer(rospy.Duration(0.2), outputTimerCallback)
	while not rospy.core.is_shutdown():
		bytesIn = ser.in_waiting
		if(bytesIn):
			dataIn = ser.read(bytesIn)
			print(dataIn)
		#rospy.rostime.wallsleep(0.2)
		time.sleep(0.2)


if __name__ == '__main__':
 	listener()
	ser.close()
