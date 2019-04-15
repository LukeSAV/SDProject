#pragma once

#include "../include/Map.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#include <memory>

class LocalOp {
public:
    LocalOp();
    virtual ~LocalOp();
    static std::shared_ptr<Map> m;
    static void addMap(int end_x_index, int end_y_index);
    static void addMap(int end_x_index, int end_y_index, std_msgs::ImageConstPtr& img);
};