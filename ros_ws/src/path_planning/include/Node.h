#pragma once
#include <set>
#include <memory>
#include <bits/stdc++.h>

class Node {
public:
    Node();
    virtual ~Node();

    int G; // Cost from start to node
    int H; // Estimated cost to end node

    int x_index; // x index of node in map
    int y_index; // y index of ndoe in map

    std::shared_ptr<Node> prevNode; 
};