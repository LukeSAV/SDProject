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
  gps_lock.acquire()
  gps_meas_msg = deepcopy(gps_msg)
  gps_updated = True
  if(not is_gps_init):
      init_gps_meas = deepcopy(gps_msg)
      is_gps_init = True
  gps_lock.release()

def imu_callback(imu_msg):
  global imu_meas_msg, imu_updated
  imu_lock.acquire()
  imu_meas_msg = imu_msg
  imu_updated = True
  if(not is_imu_init):
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

  # HELP MIKE WUT TO DO
  # gps_meas is the gps message
  lat_lon = (gps_meas.latitude, gps_meas.longitude)
  init_lat_lon = (init_gps_meas.latitude, init_gps_longitude)
  purdue_fountain = () # to be determined later

  # Convert from lat_lon to x_y
  x_y = (lat_lon - init_lat_lon)/111139

  # Convert IMU data into current reference frame
  imu_x = imu_meas.orientation.x = init_imu_meas.orientation.x

  ############# Mike takes the wheel ######
  theta_l = int(encoder_msg.data[2:4])
  theta_r = int(encoder_msg.data[5:7])i

  ekf.update_gps_cov(gps_msg.status.service)
  ekf.update_encoder_cov(thetaR, thetaL)

  if(gps_valid):
      ekf.step(0.1, x_y[0], x_y[1], imu_x, thetaL, thetaR)
  else:
      ekf.step(0.1, imu_x, thetaL, thetaR)
    
  print("FILETERED OUTPUT:")
  print("GPS X (meters): " + str(float(ekf.x[0])))
  print("GPS Y (meters): " + str(float(ekf.x[1])))
  print("IMU Heading (radians): " + str(float(ekf.x[2])))
  print("Encoder Theta R (ticks): " + str(float(ekf.x[3])))
  print("Encoder THeta L (ticks): " str(float(ekf.x[4])))
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




