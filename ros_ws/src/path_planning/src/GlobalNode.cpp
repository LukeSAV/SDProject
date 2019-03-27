#include <string>
#include "../include/GlobalNode.h"

GlobalNode::GlobalNode() {

}

GlobalNode::GlobalNode(double lat, double lon) {
	this->lat = lat;
	this->lon = lon;
}

GlobalNode::GlobalNode(std::string lat, std::string lon) {
	this->lat = std::stod(lat);
	this->lon = std::stod(lon);
}

GlobalNode::~GlobalNode() {

}

