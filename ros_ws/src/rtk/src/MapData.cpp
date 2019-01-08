#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "../include/MapData.h"
#include "../include/Node.h"
#include "../include/NMEAData.h"

std::string MapData::nearestLandmarkKey;
std::map<std::string, Node> MapData::node_map; // Key: Unique identifier for node, Value: Node type with relevant data for a single point on the Map
std::map<std::string, std::pair<std::vector<std::string>, std::string>> MapData::way_map; // Key: Unique identifier string for waypoint, Value: pair - first element = vector of unique node identifier strings, second element = name of waypoint

MapData::MapData() {

}

MapData::~MapData() {

}

void MapData::setClosestLandmark() { // Set the closest landmark to the user position based on the current gpgga string
	std::map<std::string, std::pair<std::vector<std::string>, std::string>>::iterator ways_it;
	std::pair<double, double> latlon = NMEAData::getLatLon();
	double cur_lat = latlon.first;
	double cur_lon = latlon.second;
	if(cur_lat == 0.0 && cur_lon == 0.0) {
		return;
	}

	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = way_map.begin(); ways_it != way_map.end(); ways_it++) { // Check each of the landmarks
		for(std::string node_id : ways_it->second.first) { // Check each point that constitutes a landmark
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
		}	
	}
}

