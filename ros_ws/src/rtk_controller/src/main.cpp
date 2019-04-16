#include <chrono>
#include <exception>
#include <iomanip>
#include <map>
#include <sstream>
#include <thread>

/* OpenCV */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "../include/MapNode.h"
#include "../include/MapData.h"
#include "../include/xml_reader.h"
#include "rtk/HeadingSpeed.h"

/* Local Navigation Includes */
#include "../include/Map.h"
#include "../include/Node.h"
#include "../include/LocalOp.h"

/* OpenGL */
#include "../include/GLDebug.h"
#include "GL/freeglut.h"
#include "GL/gl.h"

#define ACCEPTED_DISTANCE_TO_WAYPOINT 5
#define SERIES_LENGTH 8

#ifndef _USE_OBSTACLE_MAP
#define _USE_OBSTACLE_MAP 
#endif

#ifndef _USE_A_STAR_INTERPOLATION
#define _USE_A_STAR_INTERPOLATION 
#endif

#ifndef _USE_OPEN_GL
#define _USE_OPEN_GL
#endif

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

static std::pair<double, double> cur_coord(41.0f, -86.0f);

static sensor_msgs::ImageConstPtr img_msg;

/*
*  Convert the floating point series to strings to publish to micro
*/
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

/*
* Take in the lidar map and update stored image
*/
void LocalMapCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        img_msg = msg;
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

/*
*  Store the most recent heading from the EKF until a filtered NavSatFix message is received
*/
void EKFHeadingCallback(const sensor_msgs::Imu::ConstPtr& msg) { 
    cur_heading_ekf = 2 * pi - msg->orientation.y; // Rotation is inverted from standard notation
}

/*
*  Iterpolate next set of points to the goal after a specified amount of time when an EKF message is received and perform path planning.
*/
void EKFPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    auto cur_time = std::chrono::system_clock::now();
    std::chrono::duration<double> time_since_last_position_series_sent = cur_time - time_last_position_series_sent;
    last_ekf_received_time = std::chrono::system_clock::now();
    if(time_since_last_position_series_sent.count() < 2.0f) { // Not sending messages closer than one second apart
        return;
    }
    time_last_position_series_sent = std::chrono::system_clock::now();

    cur_coord.first = msg->latitude;
    cur_coord.second = msg->longitude;

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
        float angle_delta = angle - cur_heading_ekf;
        if(angle_delta > pi / 2.0f || angle_delta < -pi / 2.0f) {
            ROS_ERROR("INVALID HEADING - Current EKF: %f, Current Map: %f", cur_heading_ekf, angle);
            return;
        }
        // Obtain a desired endpoint based on the heading to the next waypoint
        int x = 0;
        int y = 0;
        if(angle_delta != 0.0f) {
            float b = 25.0f;
            float m = 1.0f / tan(angle_delta);

            float y_0 = (0.0f - b) * m;
            float y_50 = (50.0f - b) * m;
            float x_50 = (50.0f / m) + b;

            if(y_0 <= 50.0f && y_0 >= 0) {
                x = 0; 
                y = y_0;
            } else if(y_50 <= 50 && y_50 >= 0) {
                x = 50;
                y = y_50;
            } else if(x_50 >= 0 && x_50 <= 50) {
                x = x_50;
                y = 50;
            }
            int temp_y = y;
            y = 50 - x;
            x = temp_y;
        } else {
            x = 50;
            y = 25;
        }

        #ifdef _USE_OBSTACLE_MAP
            LocalOp::addMap(x, y, img_msg);
        #else
            LocalOp::addMap(x, y);
        #endif

        std::shared_ptr<Node> n = LocalOp::m->AStarSearch();
        if(n == NULL) {
            ROS_ERROR("Could not reach destination");
            return;
        }
        #ifdef _USE_A_STAR_INTERPOLATION
            std::shared_ptr<Node> cur_node = n;
            int path_length = 0;
            while(cur_node) {
                cur_node = cur_node->prevNode;
                path_length++;
            }
            int path_unit_step = path_length / 8;
            if(path_unit_step < 1) {
                ROS_ERROR("Interpolated points in A* algorithm were incorrect");
                return;
            } else {
                std::shared_ptr<Node> cur_node = n;
                for(unsigned int i = 0; i < 8; i++) {
                    if(cur_node) {
                        x_series[i] = (float)(25 - cur_node->y_index) * 0.1f;
                        y_series[i] = (float)(cur_node->x_index) * 0.1f;
                        int step = path_unit_step;
                        while(step > 0) {
                            cur_node = cur_node->prevNode;
                            step--;
                        }
                    } else { 
                        ROS_ERROR("Problem tracing back A* path");
                        break;
                    }
                }
                
            }

        #else
            float dx = (25.0f - LocalOp::m->end->y_index) * 0.1f;
            float dy = LocalOp::m->end->x_index * 0.1f;
            angle_delta = pi / 2.0f - atan2(dy, dx);
            if(angle_delta < 0.0f) {
                angle_delta += 2.0f * pi;
            }
            
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
        #endif
        convertPubMsg(); // Convert the x_series and y_series points to a string to send to the microcontroller
        cmd_pub.publish(pub_msg);
    } catch(std::out_of_range& eor) {
        ROS_ERROR("Waypoint out of range");
    }
}

void gpvtgCallback(const rtk::HeadingSpeed::ConstPtr& msg) {
    cur_speed = msg->speed;
    cur_heading_gps = msg->heading;
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
    ros::Subscriber local_map = nh.subscribe("perception/map", 1000, LocalMapCallback);
    auto start_landmark = std::chrono::system_clock::now();
    auto cur_time = start_landmark;
    std::chrono::duration<double> elapsed_time;
    xml_reader("/home/luke/SDProject/ros_ws/src/rtk_controller/purdue_mapv1.0_old.xml");

    // Initialize frst waypoint
    prev_waypoint_key = MapData::path_map.begin()->first;
    next_waypoint_key = std::next(MapData::path_map.begin())->first;

    // Poll for EKF messages and publish path points
    ros::Rate r(10);
    last_ekf_received_time = std::chrono::system_clock::now();

    LocalOp::addMap(50, 25);

    // Spin off thread to run glut window
    #ifdef _USE_OPEN_GL
        GLDebug::init(argc, argv);
        std::thread t1(glutMainLoop);
    #endif

    while(ros::ok()) {
        cur_time = std::chrono::system_clock::now();
        elapsed_time = cur_time - last_ekf_received_time;
        if(elapsed_time.count() > 3.0f) { // Went 3 seconds without an EKF message. Better stop.
            ROS_ERROR("3 seconds without EKF data");
        }
        elapsed_time = cur_time - start_landmark;

        if(elapsed_time.count() > 5.0f) { // Check the nearest landmark every 5 seconds
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

    #ifdef _USE_OPEN_GL
        t1.join();
    #endif
}
