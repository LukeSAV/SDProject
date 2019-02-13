#pragma once
#include "Node.h"
#include <memory>
#include <set>
#include <boost/numeric/ublas/matrix.hpp>

#define MAP_SIZE 51 // Dimensions of grid
#define GRID_RESOLUTION 20 // Resolution of grid tiles in cm

class Map {
public:
    Map(int x_index, int y_index);
    virtual ~Map();

    std::shared_ptr<Node> AStarSearch();

    boost::numeric::ublas::matrix<std::shared_ptr<Node>> grid; // Grid of nodes 

    std::set<std::shared_ptr<Node>> closedSet; // Nodes already passed 
    std::set<std::shared_ptr<Node>> openSet; // Nodes not yet passed
    std::set<std::shared_ptr<Node>> pathSet; // Nodes in the final path
};