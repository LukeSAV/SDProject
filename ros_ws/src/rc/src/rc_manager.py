#!/usr/bin/env python2.7
import rospy
from sensor_msgs import msg

def gpsCallback(joyMsg):
	rospy.loginfo("X: %d", joyMsg.buttons[14])
	rospy.loginfo("Left: %f Right: %f", joyMsg.axes[1], joyMsg.axes[3])


def listener():
	rospy.init_node('rc_listener', anonymous=True)
	rospy.Subscriber("joy", msg.Joy, gpsCallback)
	rospy.spin()

if __name__ == '__main__':
  listener()

