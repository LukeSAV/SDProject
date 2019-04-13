#! /usr/bin/python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from math import sin, cos

x = 0
y = 0
theta = 0
pub = None

def goal_string_callback(string_in):
  global x, y

  sign1 = 1 if string_in[2] == '+' else -1
  sign2 = 1 if string_in[7] == '+' else -1
  print("Sign1: " + sign1 + " Sign2 " + sign2)
  val1 = float(string_in[3:6]) / 100
  val2 = float(string_in[8:11]) / 100

  x = sign1 * val1
  y = sign2 * val2

  print("X: " + x + " Y: " y)


def ekf_filtered(nav_msg):
  goal_point_msg = NavSatFix()
  lat_adj = (sin(theta) * x + cos(theta)*y) / 111139
  long_adj = (cos(theta)*x + -1 * sin(theta)*y) / 111139

  goal_point_msg.latitude = nav_msg.latitude + lat_adj
  goal_point_msg.longitude= nav_msg.longitude + long_adj

  pub.publish(goal_point_msg)


def ekf_direction(orientation):
  global theta
  theta = orientation.y




if __name__ == "__main__":
  global pub
  rospy.init_node("GoalPointDisplay")
  sub = rospy.Subscriber("/goal_string", String, goal_string_callback)

  rospy.Subscriber("/ekf/filtered"), NavSatFix, ekf_filtered)
  rospy.Subscriber("/efk/imu/data"), NavSatFix, ekf_direction)
  rospy.Publisher("/goalpoint", NavSatFix, queue_size=10)
  rospy.spin()
