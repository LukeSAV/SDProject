#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg

ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)

def controllerCallback(joyMsg):
	ser.write('L' + str(int(-1 * joyMsg.axes[1] * 35) + 90) + '\n')
	ser.write('R' + str(int(-1 * joyMsg.axes[5] * 35) + 90) + '\n')


def listener():
	rospy.init_node('rc_listener', anonymous=True)
	rospy.Subscriber("joy", msg.Joy, controllerCallback)
	while not rospy.core.is_shutdown():
		bytesIn = ser.in_waiting
		if(bytesIn):
			dataIn = ser.read(bytesIn)
			print(dataIn)
		rospy.rostime.wallsleep(0.5)


if __name__ == '__main__':
	while True:
		time.sleep(1)
		ser.write('Test')

	
 	listener()
