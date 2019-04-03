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

state_space_init = False


# TODO what is the default orientation?
def gps_callback(gps_msg):
  global gps_updated, gps_meas_msg
  gps_lock.acquire()
  gps_meas_msg = deepcopy(gps_msg)
  gps_updated = True
  gps_lock.release()

def imu_callback(imu_msg):
  global imu_meas_msg, imu_updated
  imu_lock.acquire()
  imu_meas_msg = deepcopy(imu_msg)
  imu_updated = True
  imu_lock.release()


def encoder_callback(encoder_msg):
  global gps_updated
  global ekf, state_space_init
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

  # y is north, x is east
  purdue_fountain = (40.428642, -86.913776) 
  x = (lat_lon[1]-purdue_fountain[1])/111139 # in meters
  y = (lat_lon[0]-purdue_fountain[0])/111139 # in meters

  # Find IMU Heading (True North)
  imuHeading = imu_meas.orientation.y

  if(gps_valid and not state_space_init):
      ekf.x[0] = x
      ekf.x[1] = y
      elf.x[2] = imuHeading
      state_space_init = True

  ############# Mike takes the wheel ######
  theta_l = int(encoder_msg.data[2:4])
  theta_r = int(encoder_msg.data[5:7])

  ekf.update_gps_cov(gps_msg.status.service)
  ekf.update_encoder_cov(theta_r, theta_l)

  if(gps_valid and state_space_init):
      ekf.step(0.2, x, y, imuHeading, theta_l, theta_r)
  elif (state_space_init):
      ekf.step(0.2, imuHeading, theta_l, theta_r)
    
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




