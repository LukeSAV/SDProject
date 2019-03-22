#include "../include/Map.h"
#include "../include/Node.h"
#include <memory>
#include <queue>
#include <set>
#include "GL/freeglut.h"
#include "GL/gl.h"

Map::Map(int x_index, int y_index) : grid(boost::numeric::ublas::matrix<std::shared_ptr<Node>>(MAP_SIZE, MAP_SIZE)), obstacle_grid(boost::numeric::ublas::matrix<int>(MAP_SIZE, MAP_SIZE)) {
    ApplyObstacles();
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
    end = grid(x_index, y_index);
}

Map::~Map() {
}

bool IsOutOfBounds(int x, int y) {
    if(x >= MAP_SIZE || x < 0 || y >= MAP_SIZE || y < 0) {
        return true; 
    }
    else {
        return false;
    }
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
            std::cout << "Destination reached" << std::endl;
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

void Map::ApplyObstacles() {
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            obstacle_grid(i, j) = 0;
        }
    }
    //
    obstacle_grid(24, 26) = 128;
    obstacle_grid(25, 26) = 128;
    obstacle_grid(26, 26) = 128;
    obstacle_grid(24, 25) = 128;
    obstacle_grid(25, 25) = 128;
    obstacle_grid(26, 25) = 128;
    obstacle_grid(24, 24) = 128;
    obstacle_grid(25, 24) = 128;
    obstacle_grid(26, 24) = 128;

    obstacle_grid(23, 27) = 50;
    obstacle_grid(23, 26) = 50;
    obstacle_grid(23, 25) = 50;
    obstacle_grid(23, 24) = 50;
    obstacle_grid(23, 23) = 50;

    obstacle_grid(27, 27) = 50;
    obstacle_grid(27, 26) = 50;
    obstacle_grid(27, 25) = 50;
    obstacle_grid(27, 24) = 50;
    obstacle_grid(27, 23) = 50;

    obstacle_grid(24, 27) = 50;
    obstacle_grid(25, 27) = 50;
    obstacle_grid(26, 27) = 50;

    obstacle_grid(24, 23) = 50;
    obstacle_grid(25, 23) = 50;
    obstacle_grid(26, 23) = 50;
    //
    obstacle_grid(44, 26) = 128;
    obstacle_grid(45, 26) = 128;
    obstacle_grid(46, 26) = 128;
    obstacle_grid(44, 25) = 128;
    obstacle_grid(45, 25) = 128;
    obstacle_grid(46, 25) = 128;
    obstacle_grid(44, 24) = 128;
    obstacle_grid(45, 24) = 128;
    obstacle_grid(46, 24) = 128;
}