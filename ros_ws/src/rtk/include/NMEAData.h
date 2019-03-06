#pragma once
#include "rtk/HeadingSpeed.h"
#include "rtk/HeadingSpeed.h"
#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <utility>
#include "sensor_msgs/NavSatFix.h"
#include "rtk/HeadingSpeed.h"
class NMEAData {
public:
	NMEAData();
	virtual ~NMEAData();
	static std::string gpgga_msg; // GPGGA string 
	static std::string gpvtg_msg; // GPVTG string
	static std::string broken_gpgga; // GPGGA that got split between two successive reads in the hardware buffer
	static std::string broken_gpvtg; // GPVTG that got split between two successive reads in the hardware buffer
	static std::mutex gpgga_mu; // Mutex lock on GPGGA string
	static std::atomic<bool> gpgga_ready; // Semaphore on GPGGA
	static std::mutex gpvtg_mu; // Mutex lock on GPVTG string
	static std::atomic<bool> gpvtg_ready; // Semaphore on GPVTG

	static void parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub, ros::Publisher gpvtg_pub);
	static void popMsg(sensor_msgs::NavSatFix &msg); // Populates the given message based on the GPGGA string received
	static void popHSMessage(rtk::HeadingSpeed &msg); // Populates the given message based on the GPVTG string received
};
