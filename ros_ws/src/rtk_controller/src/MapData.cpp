#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "../include/MapData.h"
#include "../include/MapNode.h"

std::map<std::string, MapNode> MapData::node_map; // Key: Unique identifier for node, Value: MapNode type with relevant data for a single point on the Map
std::map<std::string, MapNode> MapData::path_map; // Key: Unique identifier for path node, Value: MapNode type with relevant data for a single point on the Map
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
				MapNode cur_node = node_map[node_id];
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
	std::map<std::string, MapNode>::iterator ways_it;
	static std::string nearestWaypointKey; // String key using unique identifier to the nearest landmark
	double cur_lat = cur_coord.first;
	double cur_lon = cur_coord.second;
	if(cur_lat == 0.0 && cur_lon == 0.0) {
		return "";
	}
	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = path_map.begin(); ways_it != path_map.end(); ways_it++) { // Check each of the landmarks
		MapNode cur_node = ways_it->second;
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

/*std::pair<std::string, std::string> MapData::getPrevAndNextWaypoints(std::pair<double, double> cur_coord) {
	std::map<std::string, MapNode>::iterator path_map_it; 
	for(path_map_it = std::next(path_map.begin()); path_map_it != path_map.end(); path_map_it++) { // Iterate through each node to find the 
		std::pair<double, double> prev_waypoint_coord = std::pair<double, double>(std::prev(path_map_it)->second.lat, std::prev(path_map_it)->second.lon);
		std::pair<double, double> next_waypoint_coord = std::pair<double, double>(path_map_it->second.lat, path_map_it->second.lon);
		std::pair<double, double> waypoint_connective = std::pair<double, double>(next_waypoint_coord.first - prev_waypoint_coord.first, next_waypoint_coord.second - prev_waypoint_coord.first);
		double distance = sqrt(waypoint_connective.first * waypoint_connective.first + waypoint_connective.second * waypoint_connective.second);
		std::pair<double, double> waypoint_connective_direction = std::pair<double, double> (waypoint_connective.first / distance, waypoint_connective.second / distance);	

	}
	return std::pair<std::string, std::string>("", "");
}*/

double MapData::getDistance(std::pair<double, double> cur_pos, MapNode waypoint) {
	return DEGREE_MULTI_FACTOR * sqrt((cur_pos.first - waypoint.lat) * (cur_pos.first - waypoint.lat) + (cur_pos.second - waypoint.lon) * (cur_pos.second - waypoint.lon));
}

MapData::linePos MapData::getSideOfLine(MapNode prev_waypoint, MapNode next_waypoint, std::pair<double, double> cur_pos) {
	double line_val = (cur_pos.first - prev_waypoint.lat) * (next_waypoint.lon - prev_waypoint.lon) - (cur_pos.second - prev_waypoint.lon) * (next_waypoint.lat - prev_waypoint.lat); // Use outer product
	if(line_val > 0) { 
		return MapData::linePos::Left;
	}
	else {
		return MapData::linePos::Right;
	}
}
double MapData::getDistanceFromLine(MapNode prev_waypoint, MapNode next_waypoint, std::pair<double, double> cur_pos) {
	// Do vector projection of current point onto line
	std::pair<double, double> line_vec = std::pair<double, double>(next_waypoint.lat - prev_waypoint.lat, next_waypoint.lon - prev_waypoint.lon);
	std::pair<double, double> point_vec = std::pair<double, double>(cur_pos.first - prev_waypoint.lat, cur_pos.second - prev_waypoint.lon);
	double line_distance = sqrt(line_vec.first * line_vec.first + line_vec.second * line_vec.second);
	std::pair<double, double> line_unit_vec = std::pair<double, double>(line_vec.first / line_distance, line_vec.second / line_distance);

	double proj_distance = point_vec.first * line_unit_vec.first + point_vec.second * line_unit_vec.second;
	std::pair<double, double> proj_vec = std::pair<double, double>(proj_distance * line_unit_vec.first + prev_waypoint.lat, proj_distance * line_unit_vec.second + prev_waypoint.lon);

	return DEGREE_MULTI_FACTOR * sqrt((proj_vec.first - cur_pos.first) * (proj_vec.first - cur_pos.first) + (proj_vec.second - cur_pos.second) * (proj_vec.second - cur_pos.second));
}