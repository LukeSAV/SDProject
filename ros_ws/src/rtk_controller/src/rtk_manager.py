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
  #ser = serial.Serial("ttyTHS2", 57600, timeout=1)
  #while True:
	  #ser.write("test")
	  #time.sleep(1)
  listener()
