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
#include "../include/Map.h"
#include "../include/Node.h"

ros::Subscriber state_sub;

static void pathCallback(const std_msgs::String::ConstPtr& msg) {
    int x_index;
    int y_index;
    Map current_map(x_index, y_index);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    state_sub = nh.subscribe<std_msgs::String>("state", 100, pathCallback);
    ros::spin();
    return 0;
}