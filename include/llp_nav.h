#include "llp_port.h"

class LLP_NAV : public LLP_PORT {
	
public:
	LLP_NAV();
	LLP_NAV(termios, string, int, int);
	IMU_CALCDATA read_imu();
	RC_DATA read_rc();
};