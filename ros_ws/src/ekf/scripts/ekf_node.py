#! /usr/bin/python
import rospy,tf
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

elapsed_time = 0
avg_x = 0
avg_y = 0

state_space_init = False
yaw_init = 0

filtered_lat = 0
filtered_lon = 0

seq = 0
sec = 0
nsec = 0

r = rospy.Publisher("/ekf/filtered", NavSatFix, queue_size = 2)
r2 = rospy.Publisher("/ekf/imu/data", Imu, queue_size = 2)

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
  global ekf, state_space_init, yaw_init, elapsed_time, avg_x, avg_y
  global fitlered_lat, filtered_lon
  global seq, sec, nsec
  gps_lock.acquire()
  gps_meas = deepcopy(gps_meas_msg)
  gps_valid = copy(gps_updated)
  gps_updated = False
  gps_lock.release()

  imu_lock.acquire()
  imu_meas = deepcopy(imu_meas_msg)
  imu_lock.release()

  enc_updated = True

  purdue_fountain = (40.428642, -86.913776) 
  
  if gps_valid:
    # y is north, x is east
    lat_lon = (gps_meas.latitude, gps_meas.longitude)
    x = (lat_lon[1]-purdue_fountain[1])*111139 # in meters
    y = (lat_lon[0]-purdue_fountain[0])*111139 # in meters
    #print("GPS LAT: " + str(float(lat_lon[0])))
    #print("GPS LON: " + str(float(lat_lon[1])))

  # Find IMU Heading (True North)
  quaternion = (imu_meas.orientation.x,
    imu_meas.orientation.y,
    imu_meas.orientation.z,
    imu_meas.orientation.w)
  angles = tf.transformations.euler_from_quaternion(
    quaternion)
  yaw = angles[2]

      
  imuHeading = yaw
  if(gps_valid):
    elapsed_time = elapsed_time + 1

  if(gps_valid and not state_space_init):
      avg_x = avg_x + 0.1*x
      avg_y = avg_y + 0.1*y
      if(elapsed_time >= 10):
        ekf.x[0] = avg_x
        ekf.x[1] = avg_y
        ekf.x[2] = 2.094
        yaw_init = yaw
        state_space_init = True
  imuHeading = (yaw - yaw_init) + 2.094
  ############# Mike takes the wheel ######
  theta_l = int(encoder_msg.data[2:4])
  theta_r = int(encoder_msg.data[5:7])

  if gps_valid:
    ekf.update_gps_cov(gps_meas.status.status)

  ekf.update_encoder_cov(theta_r, theta_l)
  ekf.update_imu_cov(0.01+0.005*elapsed_time)

  if(gps_valid and state_space_init):
      ekf.step(0.2, x, y, imuHeading, theta_l, theta_r)
  elif (state_space_init):
      ekf.step(0.2, imuHeading, theta_l, theta_r)
  
  if(state_space_init):
    #print("FILETERED OUTPUT:")
    #print("GPS X (meters): " + str(float(ekf.x[0])))
    #print("GPS Y (meters): " + str(float(ekf.x[1])))
    #print("IMU Heading (radians): " + str(float(ekf.x[2])))
    #print("Encoder Theta R (ticks): " + str(float(ekf.x[3])))
    #print("Encoder Theta L (ticks): " + str(float(ekf.x[4])))
    #print("\n")

    filtered_nsf = NavSatFix()
    filtered_nsf.longitude = ekf.x[0] / 111139 + purdue_fountain[1]
    filtered_nsf.latitude  = ekf.x[1] / 111139 + purdue_fountain[0]

    seq = seq+1
    filtered_nsf.header.seq = seq
    filtered_nsf.header.stamp = gps_meas.header.stamp
    filtered_nsf.header.frame_id = gps_meas.header.frame_id
    
    #print("Filtered Lat: " + str(float(filtered_nsf.latitude)))
    #print("Filtered Lon: " + str(float(filtered_nsf.longitude)))
  
    r.publish(filtered_nsf)

    filtered_imu = deepcopy(imu_meas)
    filtered_imu.orientation.x = imuHeading
    filtered_imu.orientation.y = ekf.x[2]
    filtered_imu.orientation.z = yaw_init
    r2.publish(filtered_imu)

if __name__ == "__main__":
  rospy.init_node('ekf')

  # TODO: Determine topic names
  rospy.Subscriber("/imu/data", Imu, imu_callback) 
  rospy.Subscriber("/rtk_gpgga", NavSatFix , gps_callback)
  rospy.Subscriber("/encoder", String, encoder_callback)

  rospy.spin()


