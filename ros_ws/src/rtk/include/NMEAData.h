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
	static std::mutex gpgga_mu;
	static std::atomic<bool> gpgga_ready;
	static std::mutex gpvtg_mu;
	static std::atomic<bool> gpvtg_ready;

	static std::pair<double, double> getLatLon(); // Get latitude and longitude from live gpgga message
	static void parseNMEA(char* read_buf, int readBytes, ros::Publisher gpgga_pub);
};
