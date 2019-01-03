#include <atomic>
#include <cmath>
#include <string>
#include <thread>
#include <iostream>
#include <mutex>
#include <sstream>
#include <map>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <stdlib.h>
#include <sys/reboot.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/xml_reader.h"
#include "../include/Node.h"
#include "../include/NMEAData.h"
#include "../include/ntrip_helper.h"
#define BUFFER_SIZE 3000

static std::map<std::string, Node> node_map;
static std::map<std::string, std::pair<std::vector<std::string>, std::string>> way_map;

static std::string closest_landmark;

std::mutex gpgga_mu;
std::atomic<bool> gpgga_ready;
std::mutex gpvtg_mu;
std::atomic<bool> gpvtg_ready;

std::string broken_gpgga = ""; // GPGGA that got split between two successive reads in the hardware buffer

std::pair<double, double> getLatLon() { // Get latitude and longitude from live gpgga message
	double cur_lat = 0.0;
	double cur_lon = 0.0;
	std::vector<std::string> gpgga_delineated;
	gpgga_mu.lock();
	std::stringstream ss(NMEAData::gpgga_msg);
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

void setClosestLandmark() { // Set the closest landmark to the user position based on the current gpgga string
	std::map<std::string, std::pair<std::vector<std::string>, std::string>>::iterator ways_it;
	std::pair<double, double> latlon = getLatLon();
	double cur_lat = latlon.first;
	double cur_lon = latlon.second;
	if(cur_lat == cur_lon == 0.0) {
		return;
	}

	double shortest_distance = -1; // Closest distance between a node and the current coordinate
	for(ways_it = way_map.begin(); ways_it != way_map.end(); ways_it++) { // Check each of the landmarks
		for(std::string node_id : ways_it->second.first) { // Check each point that constitutes a landmark
			Node cur_node = node_map[node_id];
			double distance = sqrt((cur_node.lat - cur_lat) * (cur_node.lat - cur_lat) + (cur_node.lon - cur_lon) * (cur_node.lon - cur_lon));
			if(shortest_distance == -1) {
				closest_landmark = ways_it->second.second; // Name of landmark
				shortest_distance = distance;
			}
			else {
				if(distance < shortest_distance) {
					shortest_distance = distance;
					closest_landmark = ways_it->second.second;
				}
			}
		}	
	}
	closest_landmark.append("\n\n");
	closest_landmark.insert(0, "LM:");
}

void parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub) { // Make sense of what's in the read buffer and pull out the GPGGA string if there is one
	/*if(FILE* fp = fopen("DataLog.txt", "w+")) { // Writing to data log
		fprintf(fp, read_buf);
		fclose(fp);
	}*/
	int index = 0;
	int gpgga_index = 0;
	std::string new_gpgga = "";
	std::string incoming_data = std::string(read_buf);
	std::size_t start_loc = incoming_data.find("$GPGGA");
	if(start_loc != std::string::npos) {
		std::string start_of_new_string = incoming_data.substr(start_loc);
		std::size_t end_loc = start_of_new_string.find_first_of('\n'); 
		broken_gpgga = ""; // Reset and ignore any currently tracked broken string
		if(end_loc != std::string::npos) {
			new_gpgga = start_of_new_string.substr(0, end_loc + 1);
		}
		else {
			//std::cout << "Broken GPGGA" << std::endl;
			broken_gpgga = start_of_new_string;
		}
	}
	else if(broken_gpgga != "") { // Grab remainder of last started string
		std::size_t next_msg_loc = incoming_data.find_first_of("$");
		std::size_t end_loc = incoming_data.find_first_of('\n'); 
		if(end_loc != std::string::npos && next_msg_loc > end_loc) {
			broken_gpgga.append(incoming_data.substr(0, end_loc + 1));
			new_gpgga = broken_gpgga;
			broken_gpgga = "";
		}
	}
	if(new_gpgga != "" && new_gpgga.substr(17, 4) != "0000" && new_gpgga.substr(30, 5) != "00000") { // Check that the message is at least feasible
		gpgga_mu.lock();
		NMEAData::gpgga_msg = new_gpgga;
		std_msgs::String msg;
		msg.data = new_gpgga;
		gpgga_pub.publish(msg);
		gpgga_ready.store(true);
		gpgga_mu.unlock();
	}
}

/*void nmeaHandler(const char* nmea_conn, struct serial* nmea_in) { // Handles reading NMEA data from RTK device
	if(nmea_conn) {
		std::cout << "Not running NMEA thread.\n" << std::endl;
		return;
	}
	else {
		std::cout << "Running NMEA thread.\n" << std::endl;
	}
	char read_buf[BUFFER_SIZE];
	int readBytes;
	std::string new_gpgga = "";
	while(true) {
		memset(read_buf, '\0', sizeof(read_buf));
		readBytes = SerialRead(nmea_in, read_buf, BUFFER_SIZE); // Reads next chunk of data from hardware buffer into read_buf
		parseNMEA(read_buf, readBytes);		
	}
}

void btHandler(const char* bt_conn, struct serial* bt_serial) { // Handles reading/writing from/to bluetooth device
	std::string readStr = "";	
	std::string writeStr = "";
	char readBuf[BUFFER_SIZE];
	char writeBuf[BUFFER_SIZE];
	int readBytes = 0;
	int writeBytes = 0;
	if(bt_conn) {
		std::cout << "Not running bluetooth thread.\n";
		return;
	}
	else {
		std::cout << "Running bluetooth thread.\n";
	}

	xml_reader("/home/pi/ntrip_self/purdue_mapv1.0.xml", node_map, way_map);
	int counter = 0;
	while(true) {
		readBytes = SerialRead(bt_serial, readBuf, BUFFER_SIZE);
		if(readBytes) {
			readStr = std::string(&readBuf[0], &readBuf[readBytes]); // Add necessary elements to the string
			std::cout << "*******" << std::endl;
			std::cout << readStr << std::endl;
			std::cout << "*******" << std::endl;
			readBytes = 0;
			if(readStr == "TERM") {
				system("sudo shutdown -h now");
			}
			else if(readStr == "REBOOT") {
				system("sudo reboot");
			}
		}
		if(gpgga_ready.load()) {
			gpgga_mu.lock();
			std::cout << "New GPGGA" << std::endl;
			std::cout << NMEAData::gpgga_msg << std::endl;
			SerialWrite(bt_serial, NMEAData::gpgga_msg.c_str(), NMEAData::gpgga_msg.size());
			gpgga_mu.unlock();
			gpgga_ready.store(false);
		}
		if(counter == 20) { // Poll for closest landmark about every 2 seconds
			setClosestLandmark();
			SerialWrite(bt_serial, closest_landmark.c_str(), closest_landmark.size());
			counter = 0;	
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		counter++;
		
	}
}*/
