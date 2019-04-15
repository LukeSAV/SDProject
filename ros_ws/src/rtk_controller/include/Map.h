#pragma once
#include "Node.h"
#include <memory>
#include <set>
#include <boost/numeric/ublas/matrix.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#define MAP_SIZE 51 // Dimensions of grid
#define GRID_RESOLUTION 20 // Resolution of grid tiles in cm

class Map {
public:
    Map(int x_index, int y_index);
    Map(int x_index, int y_index, sensor_msgs::ImageConstPtr& img);
    virtual ~Map();

    std::shared_ptr<Node> AStarSearch();
    void ApplyObstacles();
    void ApplyObstacles(sensor_msgs::ImageConstPtr& img);
    void DrawMap();
    void FixEndpoint(int x_index, int y_index);
    std::pair<int, int> CycleMapRight(int x_index, int y_index);
    std::pair<int, int> CycleMapLeft(int x_index, int y_index);

    static bool Compare(std::shared_ptr<Node>, std::shared_ptr<Node>);

    boost::numeric::ublas::matrix<std::shared_ptr<Node>> grid; // Grid of nodes
    boost::numeric::ublas::matrix<int> obstacle_grid; // Grid of obstacle weights (0-128)
    std::shared_ptr<Node> end; // End node in route
};