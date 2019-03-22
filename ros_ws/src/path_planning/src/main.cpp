/*******************************************************************************
 *  @package path_planning
 *  @brief Path planning code. Subscribes to the state output of the Kalman filter and
 *  outputs an optimal path using A*.
 *  @author Luke Armbruster 
 * 
 ******************************************************************************/

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "rtk/HeadingSpeed.h"

#include "../include/Map.h"
#include "../include/Node.h"
#include "../include/GlobalNode.h"
#include "../include/GlobalMap.h"
#include "../include/xml_reader.h"
#include "../include/GLDebug.h"
#include "../include/LocalOp.h"

#include "GL/freeglut.h"
#include "GL/gl.h"

#include <map>
#include <exception>
#include <chrono>
#include <thread>

ros::Subscriber state_sub; // Subscriber to occupation grid/current position

static ros::Publisher cmd_pub; // Publisher for output to robot
static std::pair<double, double> cur_coord(0,0); // Current GPS latitude/longitude of robot
static enum GlobalMap::positionStatus position_status = GlobalMap::FixNotValid; // Current position fix status
static int num_satellites = 0; // Current number of satellites in view
static double cur_heading; // Current heading of robot
static double cur_speed; // Current speed of robot
static std_msgs::String pub_msg; // Published message for serial output

std::chrono::time_point<std::chrono::system_clock> last_gpgga_received_time;  

static std::string next_waypoint_key = "";
static std::string prev_waypoint_key = "";
static GlobalNode prev_waypoint = GlobalNode(0.0f, 0.0f);

static int left_speed = 90;
static int right_speed = 90;


static void pathCallback(const std_msgs::String::ConstPtr& msg) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    state_sub = nh.subscribe<std_msgs::String>("state", 100, pathCallback);

    GLDebug::init(argc, argv);
    LocalOp::addMap(50, 25);
    std::shared_ptr<Node> n = LocalOp::m->AStarSearch();
    
    glutMainLoop();

    return 0;
}