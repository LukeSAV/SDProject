#!/usr/bin/env python2.7
from firebase import firebase
import rospy
from std_msgs.msg import String

ref = firebase.FirebaseApplication('https://sd-d2u.firebaseio.com', None)

def gpsCallback(data):
	ref.put('', 'Current Position', data.data)
	rospy.loginfo("Received string: %s", data.data)

def listener():
	rospy.init_node('fb_listener', anonymous=True)
	rospy.Subscriber("rtk_gpgga", String, gpsCallback)
	rospy.spin()

if __name__ == '__main__':
  listener()

