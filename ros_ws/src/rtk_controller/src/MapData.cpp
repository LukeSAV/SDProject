#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "../include/MapData.h"
#include "../include/Node.h"

std::map<std::string, Node> MapData::node_map; // Key: Unique identifier for node, Value: Node type with relevant data for a single point on the Map
std::map<std::string, Node> MapData::path_map; // Key: Unique identifier for path node, Value: Node type with relevant data for a single point on the Map
std::map<std::string, std::pair<std::vector<std::string>, std::string>> MapData::landmark_map; // Key: Unique identifier string for waypoint, Value: pair - first element = vector of unique node identifier strings, second element = name of waypoint

MapData::MapData() {

}

MapData::~MapData() {

}

static std::vector<std::string> getCommaDelineated(std::string comma_sep_msg) {
	std::vector<std::string> comma_delineated;
	std::stringstream ss(comma_sep_msg);
	while(ss.good()) {
		std::string substr;
		std::getline(ss, substr, ',');
		comma_delineated.push_back(substr);
	}
	return comma_delineated;
}

std::pair<double, double> MapData::getLatLon(std::string gpgga_msg) { // Get latitude and longitude from live gpgga message
	double cur_lat = 0.0;
	double cur_lon = 0.0;
	std::vector<std::string> gpgga_delineated = getCommaDelineated(gpgga_msg);
	if(gpgga_delineated.size() > 6) {
		std::string lat_gpgga = gpgga_delineated[2];
		std::string lon_gpgga = gpgga_delineated[4];
		
		double lat_degrees = std::stod(lat_gpgga.substr(0,2));
		double lat_minutes = std::stod(lat_gpgga.substr(2));
		
		double lon_degrees = std::stod(lon_gpgga.substr(0,3));
		double lon_minutes = std::stod(lon_gpgga.substr(3));
		
		cur_lat = lat_degrees + lat_minutes / 60.0; // Assuming always north
		cur_lon = -1.0 * (lon_degrees + lon_minutes / 60.0); // Cheating and assuming always west
	}
	return std::pair<double, double>(cur_lat, cur_lon);
}

std::string MapData::getClosestLandmark(std::pair<double, double> cur_coord) { // Set the closest landmark to the user position based on the current gpgga string
	std::map<std::string, std::pair<std::vector<std::string>, std::string>>::iterator ways_it;
	static std::string nearestLandmarkKey; // String key using unique identifier to the nearest landmark
	double cur_lat = cur_coord.first;
	double cur_lon = cur_coord.second;
	if(cur_lat == 0.0 && cur_lon == 0.0) {
		return "";
	}

	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = landmark_map.begin(); ways_it != landmark_map.end(); ways_it++) { // Check each of the landmarks
		for(std::string node_id : ways_it->second.first) { // Check each point that constitutes a landmark
			try {
				Node cur_node = node_map[node_id];
				double distance = sqrt((cur_node.lat - cur_lat) * (cur_node.lat - cur_lat) + (cur_node.lon - cur_lon) * (cur_node.lon - cur_lon));
				if(shortest_distance == -1) {
					nearestLandmarkKey = ways_it->first;
					shortest_distance = distance;
				}
				else {
					if(distance < shortest_distance) {
						nearestLandmarkKey = ways_it->first;
						shortest_distance = distance;
					}
				}
			} catch(std::out_of_range& oor) {
				std::cerr << "Failed to find node with given id: " << node_id << std::endl;
			}
		}	
	}
	return nearestLandmarkKey;
}

std::string MapData::getClosestWaypoint(std::pair<double, double> cur_coord) { // Set the closest landmark to the user position based on the current gpgga string
	std::map<std::string, Node>::iterator ways_it;
	static std::string nearestWaypointKey; // String key using unique identifier to the nearest landmark
	double cur_lat = cur_coord.first;
	double cur_lon = cur_coord.second;
	if(cur_lat == 0.0 && cur_lon == 0.0) {
		return "";
	}
	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = path_map.begin(); ways_it != path_map.end(); ways_it++) { // Check each of the landmarks
		Node cur_node = ways_it->second;
		double distance = sqrt((cur_node.lat - cur_lat) * (cur_node.lat - cur_lat) + (cur_node.lon - cur_lon) * (cur_node.lon - cur_lon));
		if(shortest_distance == -1) {
			nearestWaypointKey = ways_it->first;
			shortest_distance = distance;
		}
		else {
			if(distance < shortest_distance) {
				nearestWaypointKey = ways_it->first;
				shortest_distance = distance;
			}
		}
	}
	return nearestWaypointKey;
}

int MapData::getNumSatellites(std::string gpgga_msg) {
	std::vector<std::string> gpgga_delineated = getCommaDelineated(gpgga_msg);	
	if(gpgga_delineated.size() > 7) {
		return std::stoi(gpgga_delineated[7]);
	}
	return 0;
}

MapData::positionStatus MapData::getPositionStatus(std::string gpgga_msg) {
	std::vector<std::string> gpgga_delineated = getCommaDelineated(gpgga_msg);	
	if(gpgga_delineated.size() > 6) {
		return positionStatus(std::stoi(gpgga_delineated[6]));
	}
	return positionStatus(0);
}

double MapData::getSpeed(std::string gpvtg_msg) {
	std::vector<std::string> gpvtg_delineated = getCommaDelineated(gpvtg_msg);	
	if(gpvtg_delineated.size() > 7) {
		return std::stod(gpvtg_delineated[7]);
	}
}

double MapData::getHeading(std::string gpvtg_msg) {
	std::vector<std::string> gpvtg_delineated = getCommaDelineated(gpvtg_msg);	
	if(gpvtg_delineated.size() > 1) {
		return std::stod(gpvtg_delineated[1]);
	}
}