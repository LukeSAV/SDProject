#pragma once
#include <map>
#include <string>
#include <vector>
#include "Node.h"
class MapData {
public: 
	MapData();
	virtual ~MapData();
	static std::string nearestLandmarkKey; // String key using unique identifier to the nearest landmark
	static std::map<std::string, Node> node_map;
	static std::map<std::string, std::pair<std::vector<std::string>, std::string>> way_map;
	
	static void setClosestLandmark();
};
