#! /usr/bin/python

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Make sure Opencv is built for jetson GPU
# Blob then fill
# Find clustered small objects, then get ROI
# Then can run contouring and more processing
# then find contour & momens
# numpy.asfortranarray..  (DONT CHANGE TYPES)

# Send Scott a lidar image
# who tf is scott? - Luke

img_pub = rospy.Publisher("/perception/map", Image, queue_size=1)

bridge = CvBridge()
img = np.zeros((400, 400))

def lidar_callback(msg):
  img= bridge.imgmsg_to_cv2(msg, "bgr8")

  kernel = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE,(11,11))

  kernel2 = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE,(11,11))

  kernel3 = np.ones((5,5))

  final_img = np.zeros(img.shape)
  final_img[:,:,0] = cv2.dilate(img[:,:,0], kernel)
  final_img[:,:,0] = cv2.erode(final_img[:,:,0], kernel2)
  final_img[:,:,0] = cv2.dilate(final_img[:,:,0], kernel)

  final_img = cv2.circle(final_img, (200, 400), 3, (0, 255, 0))
  final_img = cv2.resize(final_img, (51, 51)) # resizing the image for path planning
  final_img[:,:,0] = cv2.dilate(final_img[:,:,0], kernel3)

  img_msg = bridge.cv2_to_imgmsg(final_img, encoding="passthrough")
  img_pub.publish(img_msg)

  cv2.namedWindow("Lidar", cv2.WINDOW_NORMAL)
  cv2.imshow("Lidar", final_img)
  cv2.waitKey(1)
  cv2.resizeWindow("Lidar",1000,1000)


if __name__ == "__main__":
  print("Launching lidar track")
  rospy.init_node('LidarTrack')
  rospy.Subscriber("scan_img", Image, lidar_callback)
  rospy.spin()
