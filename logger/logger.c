#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "../lcmtypes/vicon_state_t.h"
#include "../lcmtypes/accel_t.h"
#include "../lcmtypes/command_t.h"

#include "../include/util.h"
#include <time.h>
#include <stdlib.h>

time_t rawtime;
struct tm* info;
char vicon_state_fname[50], accel_fname[50], cmmd_fname[50];

void vicon_state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const vicon_state_t * msg, void * user) {

	FILE* logger;
	logger = fopen(vicon_state_fname, "a");
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%" PRId64 ",%f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
		msg->timestamp, msg->position[0], msg->position[1], msg->position[2],
		msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->attitude[0], msg->attitude[1], msg->attitude[2]);
	fclose(logger);

}

void accel_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const accel_t * msg, void * user) {

	FILE* logger;
	logger = fopen(accel_fname, "a");
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%f, %f, %f\n", msg->accel[0], msg->accel[1], msg->accel[2]);
		fclose(logger);

}

void cmmd_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const command_t * msg, void * user) {

	FILE* logger;
	logger = fopen(cmmd_fname, "a");
	
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%" PRId64 ",%d,%d,%d,%d\n", msg->timestamp, msg->thrust, msg->roll, msg->pitch, msg->yaw);	
	fclose(logger);

}

int main(int argc, char* argv[]) {

	time(&rawtime);
	info = localtime(&rawtime);
	strftime(vicon_state_fname, 50, "../data/vicon_state_%d%H%M%S.txt", info);
	strftime(accel_fname, 50, "../data/accel_%d%H%M%S.txt", info);
	strftime(cmmd_fname, 50, "../data/cmmd_%d%H%M%S.txt", info);

	lcm_t* lcm = lcm_create(NULL);

	vicon_state_t_subscribe(lcm, "vicon_state", vicon_state_handler, NULL);
	accel_t_subscribe(lcm, "acceleration", accel_handler, NULL);
	command_t_subscribe(lcm, "command_msg", cmmd_handler, NULL);



	while(1) {
		lcm_handle(lcm);
	}

	lcm_destroy(lcm);
	return 0;
}
