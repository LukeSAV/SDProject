#include "../include/Map.h"
#include "../include/Node.h"
#include <memory>
#include <set>

Map::Map(int x_index, int y_index) : grid(boost::numeric::ublas::matrix<std::shared_ptr<Node>>(MAP_SIZE, MAP_SIZE)) {
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            std::shared_ptr<Node> next_node(new Node);
            grid(i, j) = next_node;
            grid(i, j)->H = abs(i - x_index) + abs(j - y_index); // Initialize heuristic to be the sum of the x and y deltas to the final point
            grid(i, j)->x_index = i;
            grid(i, j)->y_index = j;
        }
    }
    grid(0, (MAP_SIZE - 1) / 2)->G = 0; // Set the first point to have an initial cost of 0
    openSet.insert(grid(0, (MAP_SIZE - 1) / 2)); // Add the first node to the set of nodes to be evaluated
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

std::shared_ptr<Node> Map::AStarSearch() {
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

        if(min_distance == 0) { // Destination reached
            return min_node;
        }

        openSet.erase(min_node); 
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