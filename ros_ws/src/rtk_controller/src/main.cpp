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

#include <boost/math/constants/constants.hpp>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <limits>

#define ACCEPTED_DISTANCE_TO_WAYPOINT 5
#define SERIES_LENGTH 8

#ifndef _USE_OBSTACLE_MAP
//#define _USE_OBSTACLE_MAP 
#endif

#ifndef _USE_A_STAR_INTERPOLATION
//#define _USE_A_STAR_INTERPOLATION 
#endif

#ifndef _USE_STRAIGHT_LINE
#define _USE_STRAIGHT_LINE
#endif

#ifndef _USE_OPEN_GL
//#define _USE_OPEN_GL
#endif


static const float pi = 3.1415927;
static ros::Publisher cmd_pub;
static ros::Publisher wpt_pub;
static ros::Publisher goal_pub;
static ros::Publisher goal_math_pub;
static enum MapData::positionStatus position_status = MapData::FixNotValid;
static double cur_heading_gps;
static double cur_speed;
static std_msgs::String pub_msg;
static sensor_msgs::NavSatFix wpt_msg;
static sensor_msgs::NavSatFix goal_msg;
static sensor_msgs::NavSatFix goal_math_msg;

static std::chrono::time_point<std::chrono::system_clock> last_ekf_received_time = std::chrono::system_clock::now(); // Last time a message was received from the EKF
static std::chrono::time_point<std::chrono::system_clock> time_last_position_series_sent = std::chrono::system_clock::now(); // Time that the last string of points was sent to the micro
//static std::chrono::duration<double> time_since_last_position_series_sent;

static std::string next_waypoint_key = "";
static std::string prev_waypoint_key = "";
static MapNode prev_waypoint = MapNode(0.0f, 0.0f);
static float cur_heading_ekf = 0.0f; // Most recent heading received from the EKF

static float x_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float y_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static float lat_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float lon_series[SERIES_LENGTH] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


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
	    std::pair<double, double> prev_wpt(0.0f, 0.0f);
        if(prev_waypoint_key != "") {
		  prev_wpt.first = MapData::path_map.at(prev_waypoint_key).lat;
		  prev_wpt.second = MapData::path_map.at(prev_waypoint_key).lon;
        } else {
          return;
        }
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
            #ifdef _USE_STRAIGHT_LINE
                /*//Use old way of targeting waypoints
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
                }*/

                //change in lat over change in lon
                double ang = atan((next_wpt.first - prev_wpt.first)/(next_wpt.second - prev_wpt.second));


                //next path point in lat lon
                for(int i = 0; i < 8; i++) {
                  x_series[7-i] = next_wpt.second - (0.6/DEGREE_MULTI_FACTOR)*(i+1)*cos(ang);
                  y_series[7-i] = next_wpt.first - (0.6/DEGREE_MULTI_FACTOR)*(i+1)*sin(ang);
                }

                goal_msg.latitude = y_series[7];
                goal_msg.longitude = x_series[7];
                goal_pub.publish(goal_msg);

                /*//Trnaslate and rotate points into robot coordinate frame
                for(int i = 0; i < 8; i++) {
                  //Translation
                  x_series[i] = x_series[i] - cur_coord.second;
                  y_series[i] = y_series[i] - cur_coord.first;
                  
                  // Rotation
                  double delta_theta = cur_heading_ekf;
                  double robot_x = (x_series[i] * cos(delta_theta) - y_series[i] * sin(delta_theta)) * DEGREE_MULTI_FACTOR;
                  double robot_y = (x_series[i] * sin(delta_theta) + y_series[i] * cos(delta_theta)) * DEGREE_MULTI_FACTOR;

                  x_series[i] = robot_x;
                  y_series[i] = robot_y;
                }*/
            #else
                float dx = (25.0f - LocalOp::m->end->y_index) * 0.1f;
                float dy = LocalOp::m->end->x_index * 0.1f;
                angle_delta = pi / 2.0f - atan2(dy, dx);
                if(angle_delta < 0.0f) {
                    angle_delta += 2.0f * pi;
                }
                
                if(prev_waypoint_key == "") { // In the event that there is no previous waypoint, just draw path between robot and next waypoint
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
                }
                else {
                    auto next_waypoint = MapData::path_map.at(next_waypoint_key);
                    auto prev_waypoint = MapData::path_map.at(prev_waypoint_key);
                    boost::multiprecision::cpp_dec_float_50 a = prev_waypoint.lat;
                    boost::multiprecision::cpp_dec_float_50 b = next_waypoint.lat;
                    boost::multiprecision::cpp_dec_float_50 c = next_waypoint.lon;
                    boost::multiprecision::cpp_dec_float_50 d = prev_waypoint.lon;
                    boost::multiprecision::cpp_dec_float_50 e = cur_coord.second;
                    boost::multiprecision::cpp_dec_float_50 f = cur_coord.first;
                    boost::multiprecision::cpp_dec_float_50 tan_theta = tan(cur_heading_ekf);
                    /*cur_heading_ekf = 4.0;
                    boost::multiprecision::cpp_dec_float_50 a = 40.429155;
                    boost::multiprecision::cpp_dec_float_50 b = 40.429053;
                    boost::multiprecision::cpp_dec_float_50 c = -86.91313;
                    boost::multiprecision::cpp_dec_float_50 d = -86.912977;
                    boost::multiprecision::cpp_dec_float_50 e = -86.91297933;
                    boost::multiprecision::cpp_dec_float_50 f = 40.42914998;
                    boost::multiprecision::cpp_dec_float_50 tan_theta = tan(cur_heading_ekf);*/

                    double x_test = (c.convert_to<double>() - d.convert_to<double>()) / (b.convert_to<double>() - a.convert_to<double>()) * (f.convert_to<double>() - a.convert_to<double>()) + d.convert_to<double>(); //corrected last term

                    bool away;
                    double waypoint_heading = atan2(c.convert_to<double>() - d.convert_to<double>(), b.convert_to<double>() - a.convert_to<double>());
                    if(waypoint_heading < 0.0f) {
                        waypoint_heading += 2 * pi;
                    }
                    if(x_test > e) {
                        
                        std::cout << "Left side of line: " << x_test << std::endl;
                        double heading_diff = acos(cos(waypoint_heading) * cos(cur_heading_ekf) + sin(waypoint_heading)*sin(cur_heading_ekf)); 
                        if(heading_diff > 0 && heading_diff < pi/2) {
                            away = true;
                        } else if(heading_diff > pi/2 && heading_diff < pi) {
                            away = false;
                        } else if(heading_diff > pi && heading_diff < 3*pi/2) {
                            away = true;
                        } else {
                            away = false;
                        }
                    } else {
                        std::cout << "Right side of line: " << x_test << std::endl;
                        double heading_diff = acos(cos(waypoint_heading) * cos(cur_heading_ekf) + sin(waypoint_heading)*sin(cur_heading_ekf)); 
                        if(heading_diff > 0 && heading_diff < pi/2) {
                            away = false;
                        } else if(heading_diff > pi/2 && heading_diff < pi) {
                            away = true;
                        } else if(heading_diff > pi && heading_diff < 3*pi/2) {
                            away = false;
                        } else {
                            away = true;
                        }
                    }

                    // AWAY
                    boost::multiprecision::cpp_dec_float_50 intersect_pt_lon;
                    boost::multiprecision::cpp_dec_float_50 intersect_pt_lat;
                    std::cout << std::setprecision(10);
                    if(away) {
                        std::cout << "a: " << a << " b: " << b << " c: " << c << " d: " << d << " e: " << e << " f: " << f << " theta: " << cur_heading_ekf << std::endl;
                        intersect_pt_lon = (a * c - b * d - c * f + d * f - c * e * tan_theta + d * e * tan_theta) / (a - b - c * tan_theta + d * tan_theta);
                        intersect_pt_lat = (a * f - b * f - a * c * tan_theta + a * e * tan_theta + b * d * tan_theta - b * e * tan_theta) / (a - b - c * tan_theta + d * tan_theta);
                        std::cout << "Away" << std::endl;
                        std::cout << intersect_pt_lon << std::endl;
                        std::cout << intersect_pt_lat << std::endl;
                    } else {
                        // TOWARD
                        std::cout << "a: " << a << " b: " << b << " c: " << c << " d: " << d << " e: " << e << " f: " << f << " theta: " << cur_heading_ekf << std::endl;
                        boost::multiprecision::cpp_dec_float_50 toward_denom = boost::multiprecision::cpp_dec_float_50(a * a - 2 * a * b + b * b + c * c - 2 * c * d + d * d);
                        intersect_pt_lon = boost::multiprecision::cpp_dec_float_50(a * a * c - a * b * c - a * b * d - f * a * c + f * a * d + b * b * d + f * b * c - f * b * d + e * c * c - 2 * e * c * d + e * d * d) / toward_denom;
                        intersect_pt_lat = boost::multiprecision::cpp_dec_float_50(f * a * a - 2 * f * a * b + a * c * c - a * c * d - e * a * c + e * a * d + f * b * b - b * c * d + e * b * c + b * d * d - e * b * d) / toward_denom;
                        std::cout << "Toward" << std::endl;
                        std::cout << intersect_pt_lon << std::endl;
                        std::cout << intersect_pt_lat << std::endl;
                    }
                    std::cout << "Robot heading: " << cur_heading_ekf << " Waypoint heading: " << waypoint_heading << std::endl;
                    // have point of intersection use waypoint heading here to add points on line
                    for(int i = 0; i < 8; i++) { 
                        // These are the points interpolated along the line in latitude/longitude
                        double new_x_rot = 0.6f * (i + 1) * sin(waypoint_heading) / DEGREE_MULTI_FACTOR + intersect_pt_lon.convert_to<double>(); 
                        double new_y_rot = 0.6f * (i + 1) * cos(waypoint_heading) / DEGREE_MULTI_FACTOR + intersect_pt_lat.convert_to<double>();
                        if(i == 7) {
                          goal_math_msg.latitude = new_y_rot;
                          goal_math_msg.longitude = new_x_rot;
                          goal_math_pub.pubish(goal_math_msg);
                        }
                        std::cout << "x coord: " << new_x_rot << " y coord: " << new_y_rot << std::endl;


                        // Translation
                        double new_x = new_x_rot - e.convert_to<double>();
                        double new_y = new_y_rot - f.convert_to<double>();


                        // Rotation
                        double delta_theta = cur_heading_ekf;
                        double robot_x = (new_x * cos(delta_theta) - new_y * sin(delta_theta)) * DEGREE_MULTI_FACTOR;
                        double robot_y = (new_x * sin(delta_theta) + new_y * cos(delta_theta)) * DEGREE_MULTI_FACTOR;


                        std::cout << "x: " << robot_x << " y: " << robot_y << std::endl;
                        
                        x_series[i] = robot_x;
                        y_series[i] = robot_y;
                        if(robot_y < 0.0f) {
                            ROS_ERROR("Y value to send to microcontroller is negative");
                            return;
                        }
                    }
                }
            #endif
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
    goal_pub = nh.advertise<sensor_msgs::NavSatFix>("goal_pt", 1000);
    goal_math_pub = nh.advertise<sensor_msgs::NavSatFix>("goal_math_pt", 1000);
    ros::Subscriber ekf_pos_sub = nh.subscribe("/ekf/filtered", 1000, EKFPosCallback);
    ros::Subscriber ekf_heading_sub = nh.subscribe("/ekf/imu/data", 1000, EKFHeadingCallback);
    ros::Subscriber gpvtg_sub = nh.subscribe("rtk_gpvtg", 1000, gpvtgCallback);
    ros::Subscriber local_map = nh.subscribe("perception/map", 1000, LocalMapCallback);
    auto start_landmark = std::chrono::system_clock::now();
    auto cur_time = start_landmark;
    std::chrono::duration<double> elapsed_time;
    xml_reader("/home/nvidia/workspace/SDProject/ros_ws/src/rtk_controller/purdue_mapv1.0_old.xml");

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
