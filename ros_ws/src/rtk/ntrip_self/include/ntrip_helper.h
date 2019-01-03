#pragma once
#include <string>
#include <mutex>
#include <atomic>
#include "../serial.c"

extern std::mutex gpgga_mu;
//extern std::string gpgga_msg;
extern std::atomic<bool> gpgga_ready;

std::string getGPGGA(char* read_buf, int readBytes);
void btHandler(const char* bt_conn, struct serial* bt_serial);
//void nmeaHandler(const char* nmea_conn, struct serial* nmea_in);
