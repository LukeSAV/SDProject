#! /usr/bin/python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from math import sin, cos

x = 0
y = 0
theta = 0
cmd_string = ""
cmd_string_refreshed = False

def goal_string_callback(string_in):
  global x, y

  sign1 = 1 if string_in.data[2] == '+' else -1
  sign2 = 1 if string_in.data[7] == '+' else -1

  val1 = float(string_in.data[3:6][::-1]) / 100

  val2 = float(string_in.data[8:11][::-1]) / 100


  x = sign1 * val1
  y = sign2 * val2

  #print("X: " + str(x) + " Y: " + str(y))


def adj_points(x_in, y_in):
  lat_adj = (sin(theta)*x_in + cos(theta)*y_in)/111139
  long_adj= (cos(theta)*x_in + -1 * sin(theta)*y_in)/111139
  return lat_adj, long_adj
  

def ekf_filtered(nav_msg):
  global cmd_string_refreshed

  odd = False
  prev_point = None #This should be even

  if cmd_string_refreshed:
    for point in cmd_string[3:-1].split(","):
      if not odd:
        prev_point = float(point)
      if odd:
        
        print("Lukes Points: " + str(prev_point) + "," +  point)
        path_point_msg = NavSatFix()
        lat_adj, long_adj = adj_points(prev_point, float(point))
        path_point_msg.latitude = lat_adj + nav_msg.latitude
        path_point_msg.longitude = long_adj + nav_msg.longitude

        path_point_msg.header.frame_id = 'map'
        # Tell mapviz what the path point was
        path_point_msg.header.stamp = rospy.Time.now()
        cmd_pub.publish(path_point_msg)
      odd = not odd #Only output on the second point
    cmd_string_refreshed = False


  # Send MapViz what the goal point was
  goal_point_msg = NavSatFix()
  print("Troy's Points: " + str(x) + "," +  str(y))
  lat_adj, long_adj = adj_points(x,y)
  goal_point_msg.latitude = nav_msg.latitude + lat_adj

  goal_point_msg.header.frame_id = 'map'
  goal_point_msg.header.stamp = rospy.Time.now()
  goal_point_msg.longitude= nav_msg.longitude + long_adj
  pub.publish(goal_point_msg)


def cmd_callback(command_msg):
  global cmd_string,cmd_string_refreshed
  cmd_string = command_msg.data;
  cmd_string_refreshed = True
  

def ekf_direction(imu_msg):
  global theta
  theta = imu_msg.orientation.y




if __name__ == "__main__":
  global pub, cmd_pub
  rospy.init_node("GoalPointDisplay")
  pub =rospy.Publisher("/goalpoints", NavSatFix, queue_size=10)

  sub = rospy.Subscriber("/goal_string", String, goal_string_callback)

  sub1 = rospy.Subscriber("/ekf/filtered", NavSatFix, ekf_filtered)
  sub2 = rospy.Subscriber("/ekf/imu/data", Imu, ekf_direction)
  sub3 = rospy.Subscriber("/robot_cmd", String, cmd_callback)
  cmd_pub = rospy.Publisher("/path_points", NavSatFix, queue_size = 10)
    

  rospy.spin()
