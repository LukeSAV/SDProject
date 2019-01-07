#include <atomic>
#include <mutex>
#include <string>
#include <utility>
class NMEAData {
public:
	NMEAData();
	virtual ~NMEAData();
	static std::string gpgga_msg; // GPGGA string 
	static std::string gpvtg_msg; // GPVTG string

	static std::mutex gpgga_mu;
	static std::atomic<bool> gpgga_ready;
	static std::mutex gpvtg_mu;
	static std::atomic<bool> gpvtg_ready;

	static std::pair<double, double> getLatLon(); // Get latitude and longitude from live gpgga message
};
