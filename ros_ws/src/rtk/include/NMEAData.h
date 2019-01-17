#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <utility>
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

	static std::pair<double, double> getLatLon(); // Get latitude and longitude from live gpgga message
	static void parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub, ros::Publisher gpvtg_pub);
};
