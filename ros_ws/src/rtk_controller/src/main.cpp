#include <chrono>
#include <exception>
#include <thread>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "../include/Node.h"
#include "../include/MapData.h"
#include "../include/xml_reader.h"

static std::pair<double, double> cur_coord(0,0);
static enum MapData::positionStatus position_status = MapData::FixNotValid;
static int num_satellites = 0;
static double cur_heading;
static double cur_speed;

std::chrono::time_point<std::chrono::system_clock> last_gpgga_received_time;

void gpggaCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("%s", msg->data.c_str());
    cur_coord = MapData::getLatLon(msg->data); // Update the current coordinate when a new one is received
    num_satellites = MapData::getNumSatellites(msg->data);
    position_status = MapData::getPositionStatus(msg->data);
    std::string closestWaypointKey = MapData::getClosestWaypoint(cur_coord);
    ROS_INFO("%s", closestWaypointKey.c_str());
    ROS_INFO("Num Satellites: %d, Position Status: %d", num_satellites, position_status);
    last_gpgga_received_time = std::chrono::system_clock::now();
}

void gpvtgCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("%s", msg->data.c_str());
    cur_speed = MapData::getSpeed(msg->data);
    cur_heading = MapData::getHeading(msg->data);
    ROS_INFO("Current speed: %lf, Current Heading: %lf", cur_speed, cur_heading);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtk_controller_node");
	ros::NodeHandle nh;
    ros::Subscriber gpgga_sub = nh.subscribe("rtk_gpgga", 1000, gpggaCallback);
    ros::Subscriber gpvtg_sub = nh.subscribe("rtk_gpvtg", 1000, gpvtgCallback);
    auto start_landmark = std::chrono::system_clock::now();
    auto cur_time = start_landmark;
    std::chrono::duration<double> elapsed_time;
    xml_reader("/home/luke/SDProject/ros_ws/src/rtk_controller/purdue_mapv1.0.xml");
    ros::Rate r(10);
    while(ros::ok()) {
        cur_time = std::chrono::system_clock::now();
        elapsed_time = cur_time - last_gpgga_received_time;
        if(elapsed_time.count() > 3.0f || position_status != MapData::positionStatus::RTKFix) { // Went 3 seconds without a GPS message. Better stop.
            // STOP ROBOT
        }
        elapsed_time = cur_time - start_landmark;
        if(elapsed_time.count() > 5.0f) { // Check the nearest landmark every 5 seconds
            ROS_INFO("%lf, %lf", cur_coord.first, cur_coord.second);
            std::string closestLandmarkKey = MapData::getClosestLandmark(cur_coord);
            if(closestLandmarkKey != "") {
                try {
                    std::cout << "Nearest landmark: " << MapData::landmark_map.at(closestLandmarkKey).second << std::endl;
                } catch(std::out_of_range& oor) {
                    std::cerr << "Failed to get nearest landmark" << std::endl;
                }
            }
            start_landmark = std::chrono::system_clock::now();
        }

        ros::spinOnce();
        r.sleep();
    }
}