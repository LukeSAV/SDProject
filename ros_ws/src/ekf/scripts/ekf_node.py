#! /usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from threading import Lock
from copy import deepcopy, copy
from EKF import EKF

# TODO import encoder message


ekf = EKF()
gps_meas_msg = NavSatFix()
enc_meas_msg = String()
imu_meas_msg = Imu()

gps_updated = False
enc_updated = False
imu_updated = False

# Locks so the threads don't step on eachother
gps_lock = Lock()
enc_lock = Lock()
imu_lock = Lock()


# Tick count for the encoders
theta_l_meas = 0
theta_r_meas = 0

# Initial gps coordinates
init_gps_meas = NavSatFix()
init_imu_meas = Imu()

is_gps_init = False
is_imu_init = False


# TODO what is the default orientation?
def gps_callback(gps_msg):
  global gps_updated, gps_meas_msg, init_gps_meas
  global is_gps_init, init_gps_msg
  gps_lock.acquire()
  gps_meas_msg = deepcopy(gps_msg)
  gps_updated = True
  if not is_gps_init:
      init_gps_meas = deepcopy(gps_msg)
      is_gps_init = True
  gps_lock.release()

def imu_callback(imu_msg):
  global imu_meas_msg, imu_updated
  global is_imu_init
  imu_lock.acquire()
  imu_meas_msg = deepcopy(imu_msg)
  imu_updated = True
  if not is_imu_init:
      init_imu_meas = deepcopy(imu_msg)
      is_imu_init = True
  imu_lock.release()


def encoder_callback(encoder_msg):
  global gps_updated
  global ekf
  gps_lock.acquire()
  gps_meas = deepcopy(gps_meas_msg)
  gps_valid = copy(gps_updated)
  gps_updated = False
  gps_lock.release()

  imu_lock.acquire()
  imu_meas = deepcopy(imu_meas_msg)
  imu_lock.release()

  enc_updated = True

  lat_lon = (gps_meas.latitude, gps_meas.longitude)

  # compute initial gps location and get init x,y
  init_lat_lon = (init_gps_meas.latitude,
    init_gps_meas.longitude)

  # y is north, x is east
  purdue_fountain = (40.428642, -86.913776) 
  init_x = None
  init_y = None

  # Convert from lat_lon to x_y
  x = (lat_lon[0] - init_lat_lon[0])/111139
  x = (lat_lon[1] - init_lat_lon[1])/111139

  # Convert IMU data into current reference frame
  imu_x = imu_meas.orientation.x = init_imu_meas.orientation.x

  ############# Mike takes the wheel ######
  theta_l = int(encoder_msg.data[2:4])
  theta_r = int(encoder_msg.data[5:7])

  ekf.update_gps_cov(gps_msg.status.service)
  ekf.update_encoder_cov(theta_r, theta_l)

  if(gps_valid):
      ekf.step(0.2, x_y[0], x_y[1], imu_x, theta_l, theta_r)
  else:
      ekf.step(0.2, imu_x, thetaL, thetaR)
    
  print("FILETERED OUTPUT:")
  print("GPS X (meters): " + str(float(ekf.x[0])))
  print("GPS Y (meters): " + str(float(ekf.x[1])))
  print("IMU Heading (radians): " + str(float(ekf.x[2])))
  print("Encoder Theta R (ticks): " + str(float(ekf.x[3])))
  print("Encoder THeta L (ticks): " + str(float(ekf.x[4])))
  print("\n")

  if(gps_valid):
    print(gps_meas)

if __name__ == "__main__":
  rospy.init_node('ekf')

  # TODO: Determine topic names
  rospy.Subscriber("/imu/data", Imu, imu_callback) 
  rospy.Subscriber("/rtk_gpgga", NavSatFix , gps_callback)
  rospy.Subscriber("/encoder", String, encoder_callback)

  rospy.spin()




