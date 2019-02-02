#!/usr/bin/env python2.7
from firebase import firebase
import rospy
from std_msgs.msg import String

ref = firebase.FirebaseApplication('https://sd-d2u.firebaseio.com', None)

def controllerCallback(data):
	data_list = data.data.split(',')
	if(data_list[0] == 'NEXT_WPT'):
		ref.put('', 'Next Waypoint', data.data)
	if(data_list[0] == 'Left' or data_list[0] == 'Right'):
		ref.put('', 'Side of Line', data.data)



def gpsCallback(data):
	ref.put('', 'Current Position', data.data)
	rospy.loginfo("Received string: %s", data.data)

def listener():
	rospy.init_node('fb_listener', anonymous=True)
	rospy.Subscriber("rtk_gpgga", String, gpsCallback)
	rospy.Subscriber("robot_cmd", String, controllerCallback)
	rospy.spin()

if __name__ == '__main__':
  listener()

