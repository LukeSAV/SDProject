#include "../include/LocalOp.h"
#include "../include/Map.h"
#include <memory>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

std::shared_ptr<Map> LocalOp::m;

LocalOp::LocalOp() {

}
 
LocalOp::~LocalOp() {

}

void LocalOp::addMap(int end_x_index, int end_y_index) {
    LocalOp::m = std::make_shared<Map>(end_x_index, end_y_index);
}

void LocalOp::addMap(int end_x_index, int end_y_index, sensor_msgs::ImageConstPtr& img) {
    if(img == NULL) {
        LocalOp::addMap(end_x_index, end_y_index);
        return;
    }
    LocalOp::m = std::make_shared<Map>(end_x_index, end_y_index, img);
}