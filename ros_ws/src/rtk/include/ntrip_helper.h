#pragma once
#include <string>
#include <mutex>
#include <atomic>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../src/serial.cpp"

extern std::mutex gpgga_mu;
extern std::atomic<bool> gpgga_ready;

extern std::mutex gpvtg_mu;
extern std::atomic<bool> gpvtg_ready;

void parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub);
//void btHandler(const char* bt_conn, struct serial* bt_serial);
//void nmeaHandler(const char* nmea_conn, struct serial* nmea_in);
