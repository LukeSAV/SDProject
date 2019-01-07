#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "../include/Landmark.h"
#include "../include/Node.h"
#include "../include/NMEAData.h"

std::string Landmark::nearestLandmarkName;
std::map<std::string, Node> Landmark::node_map;
std::map<std::string, std::pair<std::vector<std::string>, std::string>> Landmark::way_map;

Landmark::Landmark() {

}

Landmark::~Landmark() {

}

void Landmark::setClosestLandmark() { // Set the closest landmark to the user position based on the current gpgga string
	std::map<std::string, std::pair<std::vector<std::string>, std::string>>::iterator ways_it;
	std::pair<double, double> latlon = NMEAData::getLatLon();
	double cur_lat = latlon.first;
	double cur_lon = latlon.second;
	if(cur_lat == cur_lon == 0.0) {
		return;
	}

	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = Landmark::way_map.begin(); ways_it != Landmark::way_map.end(); ways_it++) { // Check each of the landmarks
		for(std::string node_id : ways_it->second.first) { // Check each point that constitutes a landmark
			Node cur_node = Landmark::node_map[node_id];
			double distance = sqrt((cur_node.lat - cur_lat) * (cur_node.lat - cur_lat) + (cur_node.lon - cur_lon) * (cur_node.lon - cur_lon));
			if(shortest_distance == -1) {
				Landmark::nearestLandmarkName = ways_it->second.second; // Name of landmark
				shortest_distance = distance;
			}
			else {
				if(distance < shortest_distance) {
					shortest_distance = distance;
					Landmark::nearestLandmarkName = ways_it->second.second;
				}
			}
		}	
	}
	Landmark::nearestLandmarkName.append("\n\n");
	Landmark::nearestLandmarkName.insert(0, "LM:");
}

