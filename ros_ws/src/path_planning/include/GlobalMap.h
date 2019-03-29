#pragma once
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "GlobalNode.h"
class GlobalMap {
public: 
	GlobalMap();
	virtual ~GlobalMap();
	enum positionStatus {FixNotValid = 0, GPSFix = 1, DiffGPSFix = 2, RTKFix = 4, RTKFloat = 5}; 
	enum linePos {Left = 0, Right = 1};

	static std::map<std::string, GlobalNode> node_map;
	static std::map<std::string, GlobalNode> path_map;
	static std::map<std::string, std::pair<std::vector<std::string>, std::string>> landmark_map;
	
	static std::string getClosestLandmark(std::pair<double, double> cur_coord); // Get the closest landmark to the current position
	static std::string getClosestWaypoint(std::pair<double, double> cur_coord); // Get the closest waypoint to the current position
	static std::pair<double, double> getLatLon(std::string gpgga_msg); // Get latitude and longitude from GPGGA string
	static int getNumSatellites(std::string gpgga_msg); // Get number of satellites from GPGGA string
	static positionStatus getPositionStatus(std::string gpgga_msg); // Get positionStatus from GPGGA string
	static double getHeading(std::string gpvtg_msg); // Get heading from GPVTG string
	static double getSpeed(std::string gpvtg_msg); // Get speed from GPVTG string
	static double getDistance(std::pair<double, double> cur_pos, GlobalNode waypoint);
	//static std::pair<std::string, std::string> getPrevAndNextWaypoints(std::pair<double, double> cur_coord); // Get pair of keys for previous and next waypoints based on current position.
	static double getDistanceFromLine(GlobalNode prev_waypoint, GlobalNode next_waypoint, std::pair<double, double> cur_pos);
	static linePos getSideOfLine(GlobalNode prev_waypoint, GlobalNode next_waypoint, std::pair<double, double> cur_pos);
};
