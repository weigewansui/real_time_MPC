#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "../lcmtypes/state_t.h"

#include "../include/util.h"
#include <time.h>
#include <stdlib.h>

time_t rawtime;
struct tm* info;
char imu_fname[50];
void imu_state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const state_t * msg, void * user) {

	FILE* logger;
	logger = fopen(imu_fname, "a");
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%" PRId64 ",%f, %f, %f\n", 
		msg->timestamp, msg->accel[0], msg->accel[1], msg->accel[2]);
	fclose(logger);

}

int main(int argc, char* argv[]) {

	time(&rawtime);
	info = localtime(&rawtime);
	strftime(imu_fname, 50, "../data/imu_%d%H%M%S.txt", info);

	lcm_t* lcm = lcm_create(NULL);

	state_t_subscribe(lcm, "state", imu_state_handler, NULL);



	while(1) {
		lcm_handle(lcm);
	}

	lcm_destroy(lcm);
	return 0;
}