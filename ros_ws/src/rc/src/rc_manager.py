#!/usr/bin/env python2.7
import rospy
import serial
from sensor_msgs import msg

def controllerCallback(joyMsg):
	rospy.loginfo("X: %d", joyMsg.buttons[14])
	rospy.loginfo("Left: %f Right: %f", joyMsg.axes[1], joyMsg.axes[3])


def listener():
	rospy.init_node('rc_listener', anonymous=True)
	rospy.Subscriber("joy", msg.Joy, controllerCallback)
	rospy.spin()

if __name__ == '__main__':
  ser = serial.Serial("ttyTHS2", 57600, timeout=1)
  listener()
