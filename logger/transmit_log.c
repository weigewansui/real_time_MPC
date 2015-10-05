#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "../lcmtypes/transmitter_t.h"
#include "../include/util.h"
#include <time.h>
#include <stdlib.h>

time_t rawtime;
struct tm* info;
char transmitter_fname[50];

void transmitter_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const transmitter_t * msg, void * user) {

	FILE* logger;
	logger = fopen(transmitter_fname, "a");
	if(logger == NULL) {
		perror("File open error! \n");
		exit(0);
	}
	//timestamp, accel[0]~[2], angle[0]~[2], angular_vel[0]~[2]
	fprintf(logger, "%" PRId64 ",%d, %d, %d, %d, %d, %d, %d, %d\n", 
		msg->timestamp, msg->ch0, msg->ch1, msg->ch2, msg->ch3, msg->ch4, msg->ch5, msg->ch6, msg->ch7);
	fclose(logger);
}

int main(int argc, char* argv[]) {

	time(&rawtime);
	info = localtime(&rawtime);
	strftime(transmitter_fname, 50, "../data/transmitter/transmitter_%d%H%M%S.txt", info);
	printf("%s\n",transmitter_fname);

	lcm_t* lcm = lcm_create(NULL);

	transmitter_t_subscribe(lcm, "transmitter", transmitter_handler, NULL);

	while(1) {
		lcm_handle(lcm);
	}

	lcm_destroy(lcm);
	return 0;
}