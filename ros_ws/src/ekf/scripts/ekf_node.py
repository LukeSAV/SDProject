#! /usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from threading import Lock
from copy import deepcopy, copy

# TODO import encoder message


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
  imu_meas_msg = imu_msg
  imu_updated = True
  imu_lock.release()


def encoder_callback(encoder_msg):
  global gps_updated
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
  purdue_fountain = ( 

  # IMU IS AVAILABLE AS imu_meas



  ############# Mike takes the wheel ######
  theta_l = int(encoder_msg.data[2:4])
  theta_r = int(encoder_msg.data[5:7])




  if(gps_valid):
    print(gps_meas)




if __name__ == "__main__":
  rospy.init_node('ekf')

  # TODO: Determine topic names
  rospy.Subscriber("/imu/data", Imu, imu_callback) 
  rospy.Subscriber("/rtk_gpgga", NavSatFix , gps_callback)
  rospy.Subscriber("/encoder", String, encoder_callback)

  rospy.spin()




