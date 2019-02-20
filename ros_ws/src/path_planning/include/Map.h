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
    void ApplyObstacles();
    void DrawMap();

    boost::numeric::ublas::matrix<std::shared_ptr<Node>> grid; // Grid of nodes
    boost::numeric::ublas::matrix<int> obstacle_grid; // Grid of obstacle weights (0-128)
    std::shared_ptr<Node> end; // End node in route
};