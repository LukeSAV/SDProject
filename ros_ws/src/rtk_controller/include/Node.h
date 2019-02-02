#pragma once
#include <string>

class Node {
public:
	Node();
	Node(std::string, std::string);
	Node(double, double);
	virtual ~Node();
	double lat;
	double lon;
};
