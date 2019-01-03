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
#include "include/xml_reader.h"
#include "include/Node.h"
#include "include/ntrip_helper.h"
#define BUFFER_SIZE 1500

static std::map<std::string, Node> node_map;
static std::map<std::string, std::pair<std::vector<std::string>, std::string>> way_map;

static std::string closest_landmark;
std::mutex gpgga_mu;
//std::string gpgga_msg = "";
std::atomic<bool> gpgga_ready;

/*void setClosestLandmark() {
	std::map<std::string, std::pair<std::vector<std::string>, std::string>>::iterator ways_it;
	double cur_lat = 0.0;
	double cur_lon = 0.0;
	//std::shared_ptr<std::string> gpgga_sp = std::shared_ptr<std::string>(&gpgga_msg);
	//std::stringstream ss(*(gpgga_sp.get()));
	gpgga_mu.lock();
	std::stringstream ss(gpgga_msg);
	gpgga_mu.unlock();
	std::vector<std::string> gpgga_delineated;
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
	else {
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
}*/



std::string getGPGGA(char* read_buf, int readBytes) {
	int index = 0;
	int gpgga_index = 0;
	std::string new_gpgga = "";
	for(index = 0; index < readBytes; index++) {
		if(read_buf[index] == '$' &&  read_buf[index+1] == 'G' && read_buf[index + 2] == 'P' && read_buf[index + 3] == 'G' && read_buf[index + 4] == 'G' && read_buf[index + 5] == 'A') {
			//printf("GPGGA message found\n");
			if((index + 90) >= readBytes) {
					break; // no more sense in looking
			}
			for(gpgga_index = 0; gpgga_index + index < readBytes; gpgga_index++) {
				if(gpgga_index != 0 && read_buf[index + gpgga_index] == '$') {
					break;
				}
				new_gpgga.append(1, read_buf[index + gpgga_index]);
			}
			new_gpgga.append(1, 10); // Line feed
			new_gpgga.append(1, 0);
			return new_gpgga;
		}
	}
	//std::cout << "No GPGGA collected" << std::endl;
	//std::cout << read_buf << std::endl << std::endl;
	return "";
}

/*void nmeaHandler(const char* nmea_conn, struct serial* nmea_in) {
	std::string readStr = "";	
	std::string writeStr = "";
	
	char writeBuf[BUFFER_SIZE];
	int readBytes = 0;
	int writeBytes = 0;
	if(nmea_conn) {
		std::cout << "Not running NMEA thread.\n";
		return;
	}
	else {
		std::cout << "Running NMEA thread.\n";
	}
	while(true) {
		char read_buf[BUFFER_SIZE];
		int readBytes = SerialRead(nmea_in, read_buf, BUFFER_SIZE);	

		if(readBytes) {
			gpgga_mu.lock();
			std::string new_gpgga = getGPGGA(read_buf, readBytes);		
							
			if(new_gpgga != "" && new_gpgga[20] != '0') {
				//std::shared_ptr<std::string> gpgga_sp = std::shared_ptr<std::string>(&gpgga_msg)
				//(gpgga_sp.get()) = new_gpgga;
				gpgga_msg = new_gpgga.c_str();
				std::cout << new_gpgga << std::endl;
				gpgga_ready.store(true);
			}
			gpgga_mu.unlock();
		}
	}
}*/

void btHandler(const char* bt_conn, struct serial* bt_serial) {
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
		/*if(gpgga_ready.load()) {
			gpgga_mu.lock();
			SerialWrite(bt_serial, gpgga_msg.c_str(), gpgga_msg.size());
			gpgga_mu.unlock();
			gpgga_ready.store(false);
		}*/
		/*if(counter == 20) { // Poll for closest landmark about every 2 seconds
			setClosestLandmark();
			SerialWrite(bt_serial, closest_landmark.c_str(), closest_landmark.size());
			counter = 0;	
		}*/
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		counter++;
		
	}
}
