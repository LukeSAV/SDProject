#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <exception>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "../include/NMEAData.h"
#include "rtk/HeadingSpeed.h"

std::string NMEAData::gpgga_msg;
std::string NMEAData::gpvtg_msg;
std::mutex NMEAData::gpgga_mu;
std::atomic<bool> NMEAData::gpgga_ready;
std::mutex NMEAData::gpvtg_mu;
std::atomic<bool> NMEAData::gpvtg_ready;
std::string NMEAData::broken_gpgga; 
std::string NMEAData::broken_gpvtg;


NMEAData::NMEAData() {
}


NMEAData::~NMEAData() {
}

void NMEAData::popMsg(sensor_msgs::NavSatFix &msg) { // Populate NavSatFix message from GPGGA
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
	if(gpgga_delineated.size() >= 10) {
		std::string lat_gpgga = gpgga_delineated[2];
		std::string lon_gpgga = gpgga_delineated[4];
		std::string alt_gpgga = gpgga_delineated[9];
		
		double lat_degrees = std::stod(lat_gpgga.substr(0,2));
		double lat_minutes = std::stod(lat_gpgga.substr(2));
		
		double lon_degrees = std::stod(lon_gpgga.substr(0,3));
		double lon_minutes = std::stod(lon_gpgga.substr(3));
		
		msg.latitude = lat_degrees + lat_minutes / 60.0; // Cheating and assuming always northern hemisphere
		msg.longitude = -1.0 * (lon_degrees + lon_minutes / 60.0); // Cheating and assuming always western hemisphere
		msg.altitude = std::stod(alt_gpgga); 
		msg.status.status = std::stoi(gpgga_delineated[6]); // Position fix status
		msg.status.service = std::stoi(gpgga_delineated[7]); // Number of satellites in view
		msg.header.stamp.sec = std::stoi(gpgga_delineated[1]);
	}
	else { 
		std::cout << "Missed message" << std::endl;
	}
}

void NMEAData::popHSMessage(rtk::HeadingSpeed &msg) {
	double heading = 0.0;
	double speed = 0.0;
	std::vector<std::string> gpvtg_delineated;
	gpvtg_mu.lock();
	std::stringstream ss(gpvtg_msg);
	gpvtg_mu.unlock();
	while(ss.good()) {
		std::string substr;
		std::getline(ss, substr, ',');
		gpvtg_delineated.push_back(substr);
	}
	if(gpvtg_delineated.size() >= 8) {
		if(gpvtg_delineated[1] != "" && gpvtg_delineated[7] != "") {
			try {
				double heading = std::stod(gpvtg_delineated[1]);
				double speed = std::stod(gpvtg_delineated[7]);
				msg.heading = heading;
				msg.speed = speed;
			} catch(const std::invalid_argument &ia) {
				ROS_ERROR("Invalid argument for GPVTG message");
			}
		}
	}
}

void NMEAData::parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub, ros::Publisher gpvtg_pub) { // Make sense of what's in the read buffer and pull out the GPGGA string if there is one
	std::string new_gpgga = "";
	std::string incoming_data = std::string(read_buf);
	std::size_t start_loc = incoming_data.find("$GPGGA");
	if(start_loc != std::string::npos) {
		std::string start_of_new_string = incoming_data.substr(start_loc);
		std::size_t end_loc = start_of_new_string.find_first_of('\n'); 
		if(broken_gpgga != "") {
			std::cout << "Broken GPGGA message was lost" << std::endl;
		}
		broken_gpgga = ""; // Reset and ignore any currently tracked broken string
		if(end_loc != std::string::npos) {
			new_gpgga = start_of_new_string.substr(0, end_loc + 1);
		}
		else {
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

	std::string new_gpvtg = "";
	start_loc = incoming_data.find("$GPVTG");
	if(start_loc != std::string::npos) {
		std::string start_of_new_string = incoming_data.substr(start_loc);
		std::size_t end_loc = start_of_new_string.find_first_of('\n'); 
		broken_gpvtg = ""; // Reset and ignore any currently tracked broken string
		if(end_loc != std::string::npos) {
			new_gpvtg = start_of_new_string.substr(0, end_loc + 1);
		}
		else {
			broken_gpvtg = start_of_new_string;
		}
	}
	else if(broken_gpvtg != "") { // Grab remainder of last started string
		std::size_t next_msg_loc = incoming_data.find_first_of("$");
		std::size_t end_loc = incoming_data.find_first_of('\n'); 
		if(end_loc != std::string::npos && next_msg_loc > end_loc) {
			broken_gpvtg.append(incoming_data.substr(0, end_loc + 1));
			new_gpvtg = broken_gpvtg;
			broken_gpvtg = "";
		}
	}

	if(new_gpgga != "" && new_gpgga.size() == 88 && new_gpgga.substr(17, 4) != "0000" && new_gpgga.substr(30, 5) != "00000") { // Check that the message is at least feasible
		gpgga_mu.lock();
		gpgga_msg = new_gpgga;
		gpgga_mu.unlock();
		sensor_msgs::NavSatFix msg;
		popMsg(msg);
		gpgga_mu.lock();
		gpgga_pub.publish(msg);
		std::cout << gpgga_msg << std::endl;
		gpgga_ready.store(true); // Signal that a new GPGGA string was added
		gpgga_mu.unlock();
	}
	if(new_gpvtg != "") { // Check that the message is at least feasible
		gpvtg_mu.lock();
		gpvtg_msg = new_gpvtg;
		gpvtg_mu.unlock();
		rtk::HeadingSpeed msg;
		popHSMessage(msg);
		gpvtg_pub.publish(msg);
		gpvtg_ready.store(true); // Signal that a new GPVTG string was added
		gpvtg_mu.unlock();
	}
}
// Old bluetooth code might be useful here
/*void btHandler(const char* bt_conn, struct serial* bt_serial) { // Handles reading/writing from/to bluetooth device
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
			NMEAData::gpgga_mu.lock();
			std::cout << "New GPGGA" << std::endl;
			std::cout << NMEAData::gpgga_msg << std::endl;
			SerialWrite(bt_serial, NMEAData::gpgga_msg.c_str(), NMEAData::gpgga_msg.size());
			NMEAData::gpgga_mu.unlock();
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