#include <string>
class NMEAData {
public:
	NMEAData();
	virtual ~NMEAData();
	static std::string gpgga_msg;
	static std::string gpvtg_msg;
};
