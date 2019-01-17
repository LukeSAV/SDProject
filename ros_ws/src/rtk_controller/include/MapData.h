#pragma once
#include <map>
#include <string>
#include <vector>
#include "Node.h"
class MapData {
public: 
	MapData();
	virtual ~MapData();
	enum positionStatus {FixNotValid = 0, GPSFix = 1, DiffGPSFix = 2, RTKFix = 4, RTKFloat = 5}; 

	static std::map<std::string, Node> node_map;
	static std::map<std::string, Node> path_map;
	static std::map<std::string, std::pair<std::vector<std::string>, std::string>> landmark_map;
	
	static std::string getClosestLandmark(std::pair<double, double> cur_coord);
	static std::string getClosestWaypoint(std::pair<double,double> cur_coord);
	static std::pair<double, double> getLatLon(std::string gpgga_msg);

	static int getNumSatellites(std::string gpgga_msg);
	static positionStatus getPositionStatus(std::string gpgga_msg); 
	static double getHeading(std::string gpvtg_msg);
	static double getSpeed(std::string gpvtg_msg);
};
