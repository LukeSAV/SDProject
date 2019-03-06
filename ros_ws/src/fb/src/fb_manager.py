#!/usr/bin/env python2.7
from firebase import firebase
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rtk.msg import HeadingSpeed

ref = firebase.FirebaseApplication('https://sd-d2u.firebaseio.com', None)

def controllerCallback(data):
	data_list = data.data.split(',')
	if(data_list[0] == 'NEXT_WPT'):
		ref.put('', 'Next Waypoint', data.data)
	if(data_list[0] == 'Left' or data_list[0] == 'Right'):
		ref.put('', 'Side of Line', data.data)

def gpsCallback(data):
	ref.put('', 'Current Position', str(data.latitude) + ',' + str(data.longitude) + ',' + str(data.altitude) + ',' + str(data.status.status) + ',' + str(data.status.service))

def hsCallback(data):
	ref.put('', 'Current HS', str(data.heading) + ',' + str(data.speed))

def listener():
	rospy.init_node('fb_listener', anonymous=True)
	rospy.Subscriber("rtk_gpgga", NavSatFix, gpsCallback)
	rospy.Subscriber("rtk_gpvtg", HeadingSpeed, hsCallback)
	rospy.Subscriber("robot_cmd", String, controllerCallback)
	rospy.spin()

if __name__ == '__main__':
  listener()

