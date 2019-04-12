#! /usr/bin/python
import rospy
import tf
import math
from sensor_msgs.msg import Imu
index = 0
def imu_callback(imu_msg):
    global index
    quat = (imu_msg.orientation.x, imu_msg.orientation.y,
        imu_msg.orientation.z, imu_msg.orientation.w)
    
    angles = tf.transformations.euler_from_quaternion(quat)
    if index % 10 == 0:
        index = 0
        print ("Yaw: " + str(angles[2] * 180.0/ math.pi))
        print(angles)
    index += 1



if __name__ == "__main__":
    rospy.init_node("euler_angles")
    sub = rospy.Subscriber("/imu/data", Imu, imu_callback)

    rospy.spin()
