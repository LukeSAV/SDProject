#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cmath>


using namespace std;
using namespace cv;
image_transport::Publisher imgPub;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
  int width = 400;
  int height = 400;
  cv::Mat img(width,height, CV_8UC3,CvScalar(0,0,0));
  cv::Mat a_star_img(51, 51, CV_8UC3, CvScalar(0,0,0));


  float ang_min = scan_in->angle_min;
  float ang_max = scan_in->angle_max;
  float ang_inc = scan_in->angle_increment;

  // Angle min = -3.12414
  // Angle max = 3.14159
  // Angle Inc = 0.0174533
  // ranges.size() = 360


  int max_range = 3;
  //img.at<CvScalar>(200,200) = CvScalar(255,255,0);

  for(int index = 0; index < 360; index++){
    float angle = scan_in->angle_min + index*scan_in->angle_increment;
    angle = -1.0f * (angle - M_PI / 2.0f);
    if(angle > 0.0f && angle < M_PI) {
      continue;
    }
    float range = scan_in->ranges[index];
    if(range > max_range) continue;
    
    float x = cos(angle) * range;
    float y = sin(angle) * range;

    int j = x*width  / (2*max_range) + width / 2;
    int i = y*height / (2*max_range) + height - 1;
    img.at<uint8_t>(i,j,0) = 255;
  }

  // Publish the created image
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  imgPub.publish(msg);
  return;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_perception");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  imgPub = it.advertise("scan_img", 1);


  ros::Subscriber sub = n.subscribe("/scan", 10, scan_callback);


  ros::spin();


  return EXIT_SUCCESS;
}
