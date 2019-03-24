#pragma once
#include <string>

class GlobalNode {
public:
	GlobalNode();
	GlobalNode(std::string, std::string);
	GlobalNode(double, double);
	virtual ~GlobalNode();
	double lat;
	double lon;
};
