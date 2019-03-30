#! /usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from threading import Lock
from copy import deepcopy
# TODO import encoder message


gps_meas_msg = NavSatFix()
enc_meas_msg = String()
imu_meas_msg = Imu()

gps_lock = Lock()
enc_lock = Lock()
imu_lock = Lock()

# TODO what is the default orientation?
def gps_callback(gps_msg):
  gps_lock.acquire()
  gps_meas_msg = gps_msg
  gps_lock.release()

def imu_callback(imu_msg):
  imu_lock.acquire()
  imu_meas_msg = imu_msg
  imu_lock.release()


def encoder_callback(encoder_msg):
  # TODO: Set the encoder message type
  enc_lock.acquire()
  enc_meas_msg = encoder_msg
  enc_lock.release()


# This is the bulk of the program. This takes in 
# the ekf measurements and turns them into something
# the ekf module can handle
def update_ekf():

  # Get the locks for each of the measurments
  gps_lock.acquire()
  gps_meas = deepcopy(gps_meas_msg)
  gps_lock.release()

  imu_lock.acquire()
  imu_meas = deepcopy(imu_meas_msg)
  imu_lock.release()

  enc_lock.acquire()
  enc_meas = deepcopy(enc_meas_msg)
  enc_lock.release()

  # TODO pass these measurements into the ekf module

  pass


if __name__ == "__main__":
  rospy.init_node('ekf')

  # TODO: Determine topic names
  rospy.Subscriber("/imu", Imu, imu_callback) 
  rospy.Subscriber("/gps", Imu, gps_callback)
  rospy.Subscriber("/encoder", String, encoder_callback)

  # TODO need to set up a publisher for EKF output

  r = rospy.Rate(10) # 10 hz
  while not rospy.is_shutdown():
    print("Running!")
    update_ekf()
    r.sleep()



