#include <chrono>
#include <exception>
#include <map>
#include <thread>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "../include/Node.h"
#include "../include/MapData.h"
#include "../include/xml_reader.h"

#define ACCEPTED_DISTANCE_TO_WAYPOINT 0.5

static ros::Publisher cmd_pub;
static std::pair<double, double> cur_coord(0,0);
static enum MapData::positionStatus position_status = MapData::FixNotValid;
static int num_satellites = 0;
static double cur_heading;
static double cur_speed;

std::chrono::time_point<std::chrono::system_clock> last_gpgga_received_time;

static std::string next_waypoint_key = "";
static std::string prev_waypoint_key = "";
static Node prev_waypoint = Node(0.0f, 0.0f);

static int left_speed = 90;
static int right_speed = 90;

void gpggaCallback(const std_msgs::String::ConstPtr& msg) {
    cur_coord = MapData::getLatLon(msg->data); // Update the current coordinate when a new one is received
    num_satellites = MapData::getNumSatellites(msg->data);
    position_status = MapData::getPositionStatus(msg->data);
    std::string closestWaypointKey = MapData::getClosestWaypoint(cur_coord);
    ROS_INFO("Current lat/lon: %lf %lf", cur_coord.first, cur_coord.second);
    ROS_INFO("%s", closestWaypointKey.c_str());
    ROS_INFO("Num Satellites: %d, Position Status: %d", num_satellites, position_status);
    last_gpgga_received_time = std::chrono::system_clock::now();
    std_msgs::String pub_msg;
    if(position_status == MapData::positionStatus::RTKFix || position_status == MapData::positionStatus::RTKFloat) { // RTKFix/RTKFloat assuming robot position accurate to about 10cm
        // Move to next waypoint 
        if(prev_waypoint_key == "") { // Check if robot is within 0.5m of starting waypoint
            double distance = MapData::getDistance(cur_coord, MapData::path_map.begin()->second);
            ROS_INFO("Distance from starting point: %lf", distance);
            prev_waypoint_key = MapData::path_map.begin()->first;
            next_waypoint_key = std::next(MapData::path_map.begin())->first;
            pub_msg.data = "NEXT_WPT," + std::to_string(MapData::path_map.at(next_waypoint_key).lat) + "," + std::to_string(MapData::path_map.at(next_waypoint_key).lon);
            cmd_pub.publish(pub_msg);
        }
        else { // Previous and next waypoint have been established. Adhere to line.
            try {
                double distance = MapData::getDistance(cur_coord, MapData::path_map.at(next_waypoint_key));
                if(distance < ACCEPTED_DISTANCE_TO_WAYPOINT) {
                    auto temp_waypoint_key = next_waypoint_key;
                    auto next_waypoint_it = MapData::path_map.find(next_waypoint_key);
                    if(next_waypoint_it != MapData::path_map.end()) {
                        next_waypoint_key = std::next(next_waypoint_it)->first;
                        ROS_INFO("Next waypoint key: %s", next_waypoint_key.c_str());
                        // Write the waypoint lat/lon to the UART pins
                        pub_msg.data = "NEXT_WPT," + std::to_string(MapData::path_map.at(next_waypoint_key).lat) + "," + std::to_string(MapData::path_map.at(next_waypoint_key).lon);
                        cmd_pub.publish(pub_msg);
                    }
                    else {
                        std::cout << "end reached" << std::endl;
                    }
                    prev_waypoint_key = temp_waypoint_key;
                }
                MapData::linePos line_pos = MapData::getSideOfLine(MapData::path_map.at(prev_waypoint_key), MapData::path_map.at(next_waypoint_key), cur_coord);
                double line_distance = MapData::getDistanceFromLine(MapData::path_map.at(prev_waypoint_key), MapData::path_map.at(next_waypoint_key), cur_coord);
                ROS_INFO("Distance from waypoint line: %lf", line_distance);
                std::string out_msg = "Side of line: ";
                if(line_pos == MapData::linePos::Left) {
                    out_msg += "left";
                    pub_msg.data = "Left," + std::to_string(line_distance);
                    cmd_pub.publish(pub_msg);
                }
                else {
                    out_msg += "right";
                    pub_msg.data = "Right," + std::to_string(line_distance);
                    cmd_pub.publish(pub_msg);
                }
                ROS_INFO("%s", out_msg.c_str());
            } catch(std::out_of_range& eor) {
                ROS_ERROR("Waypoint out of range");
            }
        }
    }
    else {
        // Connection not accurate enough. Stop robot.
        pub_msg.data = "L90 R90";
        cmd_pub.publish(pub_msg);
        left_speed = right_speed = 90;
    }
}

void gpvtgCallback(const std_msgs::String::ConstPtr& msg) {
    cur_speed = MapData::getSpeed(msg->data);
    cur_heading = MapData::getHeading(msg->data);
    ROS_INFO("Current speed: %lf, Current Heading: %lf", cur_speed, cur_heading);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtk_controller_node");
	ros::NodeHandle nh;
    cmd_pub = nh.advertise<std_msgs::String>("robot_cmd", 1000);
    ros::Subscriber gpgga_sub = nh.subscribe("rtk_gpgga", 1000, gpggaCallback);
    ros::Subscriber gpvtg_sub = nh.subscribe("rtk_gpvtg", 1000, gpvtgCallback);
    auto start_landmark = std::chrono::system_clock::now();
    auto cur_time = start_landmark;
    std::chrono::duration<double> elapsed_time;
    //xml_reader("/home/luke/SDProject/ros_ws/src/rtk_controller/purdue_mapv1.0.xml");
    xml_reader("/home/ubuntu/SDProject/ros_ws/src/rtk_controller/lukes_test_path.xml");
    ros::Rate r(10);
    while(ros::ok()) {
        cur_time = std::chrono::system_clock::now();
        elapsed_time = cur_time - last_gpgga_received_time;
        if(elapsed_time.count() > 3.0f) { // Went 3 seconds without a GPS message. Better stop.
            std_msgs::String pub_msg;
            pub_msg.data = "L90 R90";
            cmd_pub.publish(pub_msg);
            left_speed = right_speed = 90;
        }
        elapsed_time = cur_time - start_landmark;
        if(elapsed_time.count() > 5.0f) { // Check the nearest landmark every 5 seconds
            ROS_INFO("%lf, %lf", cur_coord.first, cur_coord.second);
            std::string closestLandmarkKey = MapData::getClosestLandmark(cur_coord);
            if(closestLandmarkKey != "") {
                try {
                    ROS_INFO("Nearest landmark: %s", MapData::landmark_map.at(closestLandmarkKey).second.c_str());
                } catch(std::out_of_range& oor) {
                    ROS_ERROR("Failed to get nearest landmark");
                }
            }
            start_landmark = std::chrono::system_clock::now();
        }

        ros::spinOnce();
        r.sleep();
    }
}
