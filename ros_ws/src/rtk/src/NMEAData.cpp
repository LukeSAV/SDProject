#include <atomic>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "../include/NMEAData.h"

std::string NMEAData::gpgga_msg;
std::string NMEAData::gpvtg_msg;
std::mutex NMEAData::gpgga_mu;
std::atomic<bool> NMEAData::gpgga_ready;
std::mutex NMEAData::gpvtg_mu;
std::atomic<bool> NMEAData::gpvtg_ready;

NMEAData::NMEAData() {

}


NMEAData::~NMEAData() {

}

std::pair<double, double> NMEAData::getLatLon() { // Get latitude and longitude from live gpgga message
	double cur_lat = 0.0;
	double cur_lon = 0.0;
	std::vector<std::string> gpgga_delineated;
	gpgga_mu.lock();
	std::stringstream ss(gpgga_msg);
	gpgga_mu.unlock();
	while(ss.good()) {
		std::string substr;
		std::getline(ss, substr, ',');
		gpgga_delineated.push_back(substr);
	}
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

