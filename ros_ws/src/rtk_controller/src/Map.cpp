#include "../include/Map.h"
#include "../include/Node.h"
#include <memory>
#include <queue>
#include <set>
#include "GL/freeglut.h"
#include "GL/gl.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

Map::Map(int x_index, int y_index) : grid(boost::numeric::ublas::matrix<std::shared_ptr<Node>>(MAP_SIZE, MAP_SIZE)), obstacle_grid(boost::numeric::ublas::matrix<int>(MAP_SIZE, MAP_SIZE)) {
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            std::shared_ptr<Node> next_node(new Node);
            grid(i, j) = next_node;
            grid(i, j)->H = sqrt((i - x_index) * (i - x_index) + (j - y_index) * (j - y_index)) + obstacle_grid(i, j); // Initialize heuristic to be the sum of the x and y deltas to the final point
            grid(i, j)->x_index = i;
            grid(i, j)->y_index = j;
            obstacle_grid(i, j) = 0;
        }
    }
    grid(0, (MAP_SIZE - 1) / 2)->G = 0; // Set the first point to have an initial cost of 0
    FixEndpoint(x_index, y_index);
}

Map::Map(int x_index, int y_index, sensor_msgs::ImageConstPtr& img) : grid(boost::numeric::ublas::matrix<std::shared_ptr<Node>>(MAP_SIZE, MAP_SIZE)), obstacle_grid(boost::numeric::ublas::matrix<int>(MAP_SIZE, MAP_SIZE)) {
    ApplyObstacles(img);
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            std::shared_ptr<Node> next_node(new Node);
            grid(i, j) = next_node;
            grid(i, j)->H = sqrt((i - x_index) * (i - x_index) + (j - y_index) * (j - y_index)) + obstacle_grid(i, j); // Initialize heuristic to be the sum of the x and y deltas to the final point
            grid(i, j)->x_index = i;
            grid(i, j)->y_index = j;
        }
    }
    grid(0, (MAP_SIZE - 1) / 2)->G = 0; // Set the first point to have an initial cost of 0
    FixEndpoint(x_index, y_index);
}

Map::~Map() {

}

std::pair<int, int> Map::CycleMapRight(int x_index, int y_index) {
    if(x_index == MAP_SIZE - 1) {
        if(y_index == 0) {
            x_index--;
        }
        else {
            y_index--;
        }
    } else if(y_index == MAP_SIZE - 1) {
        x_index++;
    } else if(y_index == 0) {
        x_index--;
    }
    return std::pair<int, int>(x_index, y_index);
}

std::pair<int, int> Map::CycleMapLeft(int x_index, int y_index) { // Return the 
    if(x_index == MAP_SIZE - 1) {
        if(y_index == MAP_SIZE - 1) {
            x_index--;
        }
        else {
            y_index++;
        }
    }
    else if(y_index == 0) {
        x_index++;
    }
    else if(y_index == MAP_SIZE - 1) {
        x_index--;
    }
    return std::pair<int, int>(x_index, y_index);
}

bool IsOutOfBounds(int x, int y) {
    if(x >= MAP_SIZE || x < 0 || y >= MAP_SIZE || y < 0) {
        return true; 
    }
    else {
        return false;
    }
}

void Map::FixEndpoint(int x_index, int y_index) {
    bool left_complete = false;
    bool right_complete = false;
    std::pair<int, int> left_point = std::pair<int, int>(x_index, y_index);
    std::pair<int, int> right_point = std::pair<int, int>(x_index, y_index);
    while(true) {
        if(!left_complete) {
            if(obstacle_grid(left_point.first, left_point.second) > 10) { // Desired endpoint occupied, choose another one
                left_point = CycleMapLeft(left_point.first, left_point.second);
                if(IsOutOfBounds(left_point.first, left_point.second)) {
                    left_complete = true;
                }
            } else {
                x_index = left_point.first;
                y_index = left_point.second;
                break;
            }
        }
        if(!right_complete) {
            if(obstacle_grid(right_point.first, right_point.second) > 10) {
                right_point = CycleMapRight(right_point.first, right_point.second);
                if(IsOutOfBounds(right_point.first, right_point.second)) {
                    right_complete = true;
                }
            } else {
                x_index = right_point.first;
                y_index = right_point.second;
                break;
            }
        }
        if(right_complete && left_complete) { // No point was found
            break;
        }
    }
    end = grid(x_index, y_index);
}

bool Map::Compare(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) { // Return true if the first arg has a lower cost
    if((n1->G + n1->H) < (n2->G + n2->H)) {
        return true;
    }
    return false;
}

std::shared_ptr<Node> Map::AStarSearch() {
    std::set<std::shared_ptr<Node>> closedSet; // Nodes already passed 
    std::set<std::shared_ptr<Node>> openSet; // Nodes not yet passed
    std::set<std::shared_ptr<Node>> pathSet; // Nodes in the final path
    openSet.insert(grid(0, (MAP_SIZE - 1) / 2)); // Add the first node to the set of nodes to be evaluated
    
    path_length = 0;
    while(!openSet.empty()) {
        std::shared_ptr<Node> min_node; // Least cost node in the open set
        int min_distance = -1;
        for(auto set_iterator = openSet.begin(); set_iterator != openSet.end(); set_iterator++) {
            int distance = (*set_iterator)->H + (*set_iterator)->G;
            if(distance < min_distance || set_iterator == openSet.begin()) {
                min_distance = distance;                 
                min_node = *set_iterator;
            }
        }
        openSet.erase(min_node); 

        if(min_node == end) { // Destination reached
            ROS_INFO("A* path found");
            return min_node; // Returning last node 
        }

        closedSet.insert(min_node);
        for(int i = -1; i <= 1; i++) { // Check each neighbor or the current node
            for(int j = -1; j <= 1; j++) { 
                if(i == 0 && j == 0) {  // Only looking for the surrounding tiles
                    continue;
                }
                if(!IsOutOfBounds(min_node->x_index + i, min_node->y_index + j)) {
                    if(closedSet.find(grid(min_node->x_index + i, min_node->y_index + j)) != closedSet.end()) {
                        continue; // Already looked at this tile
                    }
                    int new_G_val = min_node->G + 1; // Add distance between original and this neighboring node
                    if(j != 0) {
                        new_G_val += 1;
                    }
                    auto neighbor = grid(min_node->x_index + i, min_node->y_index + j);
                    if(openSet.find(neighbor) == openSet.end()) {
                        openSet.insert(neighbor);
                    }
                    else if(new_G_val >= neighbor->G) {
                        continue; // Shorter path was already found to this node
                    }
                    neighbor->prevNode = min_node;
                    neighbor->G = new_G_val;

                }
                else {
                    continue; // This neighboring node is out of bounds
                }
            }
        }
    }
}

void Map::ApplyObstacles(sensor_msgs::ImageConstPtr& msg) {
    for(unsigned int i = 0; i < msg->height; i++) {
        for(unsigned int j = 0; j < msg->width; j++) {
            int intensity = msg->data[i * msg->step + j * msg->step / msg->width];
            obstacle_grid(50 - i, 50 - j) = intensity / 2;
        }
    }
}

void Map::ApplyObstacles() {
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            obstacle_grid(i, j) = 0;
        }
    }
    //
    obstacle_grid(0, 0) = 128;
    obstacle_grid(1, 0) = 128;
    obstacle_grid(2, 0) = 128;
    obstacle_grid(3, 0) = 128;
    obstacle_grid(4, 0) = 128;
    obstacle_grid(5, 0) = 128;
    obstacle_grid(6, 0) = 128;
    obstacle_grid(7, 0) = 128;
    obstacle_grid(8, 0) = 128;
    obstacle_grid(9, 0) = 128;
    obstacle_grid(10, 0) = 128;
    obstacle_grid(11, 0) = 128;
    obstacle_grid(12, 0) = 128;
    obstacle_grid(13, 0) = 128;
    obstacle_grid(14, 0) = 128;
    obstacle_grid(15, 0) = 128;
    obstacle_grid(16, 0) = 128;
    obstacle_grid(17, 0) = 128;
    obstacle_grid(18, 0) = 128;
    obstacle_grid(19, 0) = 128;
    obstacle_grid(20, 0) = 128;
    obstacle_grid(21, 0) = 128;
    obstacle_grid(22, 0) = 128;
    obstacle_grid(23, 0) = 128;
    obstacle_grid(24, 0) = 128;
    obstacle_grid(25, 0) = 128;
    obstacle_grid(26, 0) = 128;
    obstacle_grid(27, 0) = 128;
    obstacle_grid(28, 0) = 128;
    obstacle_grid(29, 0) = 128;
    obstacle_grid(30, 0) = 128;
    obstacle_grid(31, 0) = 128;
    obstacle_grid(32, 0) = 128;
    obstacle_grid(33, 0) = 128;
    obstacle_grid(34, 0) = 128;
    obstacle_grid(35, 0) = 128;
    obstacle_grid(36, 0) = 128;
    obstacle_grid(37, 0) = 128;
    obstacle_grid(38, 0) = 128;
    obstacle_grid(39, 0) = 128;
    obstacle_grid(40, 0) = 128;
    obstacle_grid(41, 0) = 128;
    obstacle_grid(42, 0) = 128;
    obstacle_grid(43, 0) = 128;
    obstacle_grid(44, 0) = 128;
    obstacle_grid(45, 0) = 128;
    obstacle_grid(46, 0) = 128;
    obstacle_grid(47, 0) = 128;
    obstacle_grid(48, 0) = 128;
    obstacle_grid(49, 0) = 128;
    obstacle_grid(50, 0) = 128;

    obstacle_grid(50, 0) = 128;
    obstacle_grid(50, 1) = 128;
    obstacle_grid(50, 2) = 128;
    obstacle_grid(50, 3) = 128;
    obstacle_grid(50, 4) = 128;
    obstacle_grid(50, 5) = 128;
    obstacle_grid(50, 6) = 128;
    obstacle_grid(50, 7) = 128;
    obstacle_grid(50, 8) = 128;
    obstacle_grid(50, 9) = 128;
    obstacle_grid(50, 10) = 128;
    obstacle_grid(50, 11) = 128;
    obstacle_grid(50, 12) = 128;
    obstacle_grid(50, 13) = 128;
    obstacle_grid(50, 14) = 128;
    obstacle_grid(50, 15) = 128;
    obstacle_grid(50, 16) = 128;
    obstacle_grid(50, 17) = 128;
    obstacle_grid(50, 18) = 128;
    obstacle_grid(50, 19) = 128;
    obstacle_grid(50, 20) = 128;
    obstacle_grid(50, 21) = 128;
    obstacle_grid(50, 22) = 128;
    obstacle_grid(50, 23) = 128;
    obstacle_grid(50, 24) = 128;
    obstacle_grid(50, 25) = 128;
    obstacle_grid(50, 26) = 128;
    obstacle_grid(50, 27) = 128;
    obstacle_grid(50, 28) = 128;
    obstacle_grid(50, 29) = 128;
    obstacle_grid(50, 30) = 128;
    obstacle_grid(50, 31) = 128;
    obstacle_grid(50, 32) = 128;
    obstacle_grid(50, 33) = 128;
    obstacle_grid(50, 34) = 128;
    obstacle_grid(50, 35) = 128;
    obstacle_grid(50, 36) = 128;
    obstacle_grid(50, 37) = 128;
    obstacle_grid(50, 38) = 128;
    obstacle_grid(50, 39) = 128;
    obstacle_grid(50, 40) = 128;
    obstacle_grid(50, 41) = 128;
    obstacle_grid(50, 42) = 128;
    obstacle_grid(50, 43) = 128;
    obstacle_grid(50, 44) = 128;
    obstacle_grid(50, 45) = 128;
    obstacle_grid(50, 46) = 128;
    obstacle_grid(50, 47) = 128;
    obstacle_grid(50, 48) = 128;
    obstacle_grid(50, 49) = 128;
    obstacle_grid(50, 50) = 128;


}