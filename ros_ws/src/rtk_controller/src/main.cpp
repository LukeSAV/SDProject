#include <chrono>
#include <exception>
#include <iomanip>
#include <map>
#include <sstream>
#include <thread>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "../include/MapNode.h"
#include "../include/MapData.h"
#include "../include/xml_reader.h"
#include "rtk/HeadingSpeed.h"

#define ACCEPTED_DISTANCE_TO_WAYPOINT 5
#define SERIES_LENGTH 8

static const float pi = 3.1415927;
static ros::Publisher cmd_pub;
static ros::Publisher wpt_pub;
static enum MapData::positionStatus position_status = MapData::FixNotValid;
static double cur_heading_gps;
static double cur_speed;
static std_msgs::String pub_msg;
static sensor_msgs::NavSatFix wpt_msg;

static std::chrono::time_point<std::chrono::system_clock> last_ekf_received_time = std::chrono::system_clock::now(); // Last time a message was received from the EKF
static std::chrono::time_point<std::chrono::system_clock> time_last_position_series_sent = std::chrono::system_clock::now(); // Time that the last string of points was sent to the micro
//static std::chrono::duration<double> time_since_last_position_series_sent;

static std::string next_waypoint_key = "";
static std::string prev_waypoint_key = "";
static MapNode prev_waypoint = MapNode(0.0f, 0.0f);
static float cur_heading_ekf = 0.0f; // Most recent heading received from the EKF

static float x_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float y_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void convertPubMsg() {
    pub_msg.data = "{K";
    for(int i = 0; i < SERIES_LENGTH; i++) {
        std::stringstream x_stream;
        std::stringstream y_stream;
        x_stream << std::fixed << std::setprecision(2) << x_series[i];
        y_stream << std::fixed << std::setprecision(2) << y_series[i];
        pub_msg.data += ",";
        pub_msg.data += x_stream.str();
        pub_msg.data += ",";
        pub_msg.data += y_stream.str();
    }
    pub_msg.data += "}";
}

void EKFHeadingCallback(const sensor_msgs::Imu::ConstPtr& msg) { 
    cur_heading_ekf = 2 * pi - msg->orientation.y; // Rotation is inverted from standard notation
}

void EKFPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    auto cur_time = std::chrono::system_clock::now();
    std::chrono::duration<double> time_since_last_position_series_sent = cur_time - time_last_position_series_sent;
    last_ekf_received_time = std::chrono::system_clock::now();
    if(time_since_last_position_series_sent.count() < 2.0f) { // Not sending messages closer than one second apart
        return;
    }
    time_last_position_series_sent = std::chrono::system_clock::now();

    std::pair<double, double> cur_coord(msg->latitude, msg->longitude);

    try {
        double distance = MapData::getDistance(cur_coord, MapData::path_map.at(next_waypoint_key)); // Check the current distance to the next waypoint
        if(distance < ACCEPTED_DISTANCE_TO_WAYPOINT) {
            auto temp_waypoint_key = next_waypoint_key;
            auto next_waypoint_it = MapData::path_map.find(next_waypoint_key);
            if(next_waypoint_it != MapData::path_map.end()) {
                next_waypoint_key = std::next(next_waypoint_it)->first;
                // Write the waypoint lat/lon to the UART pins
                std::cout << "NEXT_WPT," + std::to_string(MapData::path_map.at(next_waypoint_key).lat) + "," + std::to_string(MapData::path_map.at(next_waypoint_key).lon) << std::endl;
            }
            else {
                std::cout << "end reached" << std::endl;
            }
            prev_waypoint_key = temp_waypoint_key;
        }
        std::pair<double, double> next_wpt(MapData::path_map.at(next_waypoint_key).lat, MapData::path_map.at(next_waypoint_key).lon);
        // Calculate path to travel
        if(next_wpt.first == cur_coord.first && next_wpt.second == cur_coord.second) {
            return;
        }
        float angle = atan2(next_wpt.second - cur_coord.second, next_wpt.first - cur_coord.first);
        if(angle < 0.0f) {
            angle += 2 * pi;
        }

        wpt_msg.latitude = next_wpt.first;
        wpt_msg.longitude = next_wpt.second;
        wpt_pub.publish(wpt_msg);

        std::cout.precision(10);
        std::cout << "Current robot coordinates: " << cur_coord.first << ", " << cur_coord.second << std::endl;
        std::cout << "Next waypoint coordinates: " << next_wpt.first << ", " << next_wpt.second << std::endl;
        float angle_delta = angle - cur_heading_ekf;
        if(angle_delta > pi / 2.0f) {
            ROS_ERROR("INVALID HEADING");
            return;
        }
        std::cout << "Angle delta: " << angle_delta << std::endl;

        // Project points along line to next waypoint for 8 * 0.6 meters (4.8 meter projection)
        for(int i = 0; i < 8; i++) { 
            float new_x = 0.6f * (i + 1) * sin(angle_delta);
            float new_y = 0.6f * (i + 1) * cos(angle_delta);
            x_series[i] = new_x;
            y_series[i] = new_y;
            if(new_y < 0.0f) {
                ROS_ERROR("MISSED WAYPOINT");
                return;
            }
        }
        convertPubMsg(); // Convert the x_series and y_series points to a string to send to the microcontroller
        cmd_pub.publish(pub_msg);
    } catch(std::out_of_range& eor) {
        ROS_ERROR("Waypoint out of range");
    }
}

void gpvtgCallback(const rtk::HeadingSpeed::ConstPtr& msg) {
    cur_speed = msg->speed;
    cur_heading_gps = msg->heading;
    //ROS_INFO("Current speed: %lf, Current Heading: %lf", cur_speed, cur_heading);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtk_controller_node");
	ros::NodeHandle nh;
    cmd_pub = nh.advertise<std_msgs::String>("robot_cmd", 1000);
    wpt_pub = nh.advertise<sensor_msgs::NavSatFix>("next_waypoint", 1000);
    ros::Subscriber ekf_pos_sub = nh.subscribe("/ekf/filtered", 1000, EKFPosCallback);
    ros::Subscriber ekf_heading_sub = nh.subscribe("/ekf/imu/data", 1000, EKFHeadingCallback);
    ros::Subscriber gpvtg_sub = nh.subscribe("rtk_gpvtg", 1000, gpvtgCallback);
    auto start_landmark = std::chrono::system_clock::now();
    auto cur_time = start_landmark;
    std::chrono::duration<double> elapsed_time;
    //xml_reader("/home/luke/SDProject/purdue_mapv1.0.xml");
    xml_reader("/home/nvidia/workspace/SDProject/ros_ws/src/rtk_controller/purdue_mapv1.0.xml");

    // Initialize frst waypoint
    prev_waypoint_key = MapData::path_map.begin()->first;
    next_waypoint_key = std::next(MapData::path_map.begin())->first;

    // Poll for EKF messages and publish path points
    ros::Rate r(10);
    last_ekf_received_time = std::chrono::system_clock::now();
    while(ros::ok()) {
        cur_time = std::chrono::system_clock::now();
        elapsed_time = cur_time - last_ekf_received_time;
        if(elapsed_time.count() > 3.0f) { // Went 3 seconds without an EKF message. Better stop.
            ROS_ERROR("3 seconds without EKF data");
        }
        /*elapsed_time = cur_time - start_landmark;
        if(elapsed_time.count() > 5.0f) { // Check the nearest landmark every 5 seconds
            //ROS_INFO("%lf, %lf", cur_coord.first, cur_coord.second);
            std::string closestLandmarkKey = MapData::getClosestLandmark(cur_coord);
            if(closestLandmarkKey != "") {
                try {
                    //ROS_INFO("Nearest landmark: %s", MapData::landmark_map.at(closestLandmarkKey).second.c_str());
                } catch(std::out_of_range& oor) {
                    ROS_ERROR("Failed to get nearest landmark");
                }
            }
            start_landmark = std::chrono::system_clock::now();
        }*/

        ros::spinOnce();
        r.sleep();
    }
}
