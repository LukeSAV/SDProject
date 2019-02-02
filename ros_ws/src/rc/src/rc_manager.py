#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg

ser = serial.Serial("/dev/ttyS0", 57600, timeout=1)

def controllerCallback(joyMsg):
	rospy.loginfo("X: %d", joyMsg.buttons[14])
	rospy.loginfo("Left: %f Right: %f", joyMsg.axes[1], joyMsg.axes[3])
	ser.write('L' + str(int(joyMsg.axes[1] * 35) + 90) + '\n');
	ser.write('R' + str(int(joyMsg.axes[3] * 35) + 90) + '\n');


def listener():
	rospy.init_node('rc_listener', anonymous=True)
	rospy.Subscriber("joy", msg.Joy, controllerCallback)
	rospy.spin()

if __name__ == '__main__':
 	listener()
