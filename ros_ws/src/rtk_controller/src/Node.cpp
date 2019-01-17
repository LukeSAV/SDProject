#include <string>
#include "../include/Node.h"

Node::Node() {

}

Node::Node(double lat, double lon) {
	this->lat = lat;
	this->lon = lon;
}

Node::Node(std::string lat, std::string lon) {
	this->lat = std::stod(lat);
	this->lon = std::stod(lon);
}

Node::~Node() {

}

