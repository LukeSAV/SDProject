#include <string>
#include "../include/MapNode.h"

MapNode::MapNode() {

}

MapNode::MapNode(double lat, double lon) {
	this->lat = lat;
	this->lon = lon;
}

MapNode::MapNode(std::string lat, std::string lon) {
	this->lat = std::stod(lat);
	this->lon = std::stod(lon);
}

MapNode::~MapNode() {

}

