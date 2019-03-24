#pragma once
#include <string>

class MapNode {
public:
	MapNode();
	MapNode(std::string, std::string);
	MapNode(double, double);
	virtual ~MapNode();
	double lat;
	double lon;
};
