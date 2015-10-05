#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "../lcmtypes/state_t.h"
#include "../include/util.h"
#include <time.h>
#include <stdlib.h>

time_t rawtime;
struct tm* info;
char state_fname[50];

void state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const state_t * msg, void * user) {

	FILE* logger;
	logger = fopen(state_fname, "a");
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%" PRId64 ",%f, %f, %f, %f, %f, %f, %f, %f, %f,   %hu, %hu, %hu, %hu\n", 
		msg->timestamp, 10/1e4*msg->accel[0], 10/1e4*msg->accel[1], 10/1e4*msg->accel[2], 0.001*msg->angle[0], 0.001*msg->angle[1], 0.001*msg->angle[2],
		0.0154*msg->angular_vel[0], 0.0154*msg->angular_vel[1], 0.0154*msg->angular_vel[2], msg->rpm[0], msg->rpm[1], msg->rpm[2], msg->rpm[3]);
	fclose(logger);

}

int main(int argc, char* argv[]) {

	time(&rawtime);
	info = localtime(&rawtime);
	strftime(state_fname, 50, "../data/state/state_%d%H%M%S.txt", info);
	printf("%s\n",state_fname);

	lcm_t* lcm = lcm_create(NULL);

	state_t_subscribe(lcm, "state", state_handler, NULL);

	while(1) {
		lcm_handle(lcm);
	}

	lcm_destroy(lcm);
	return 0;
}