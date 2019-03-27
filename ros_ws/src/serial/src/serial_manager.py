#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg

ser = serial.Serial(port="/dev/ttyTHS2", baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

def controllerCallback(joyMsg):
	ser.write('{' + str(int(-1 * joyMsg.axes[1] * 35)) + ' ' + str(int(-1 * joyMsg.axes[3] * 35)) + '}' + '\n')
	


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
		value=ser.read()
		if value is not None: 
			print value
		time.sleep(1)

	print('Ending serial connection')
 	listener()
	ser.close()
