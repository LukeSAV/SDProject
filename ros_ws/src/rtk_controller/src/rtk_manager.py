#!/usr/bin/env python2.7
import rospy
import serial
from std_msgs.msg import String

def gpsCallback(data):
	rospy.loginfo("Received string: %s", data.data)


def listener():
	rospy.init_node('rtk_listener', anonymous=True)
	rospy.Subscriber("rtk_gpgga", String, gpsCallback)
	rospy.spin()

if __name__ == '__main__':
  listener()
