#include <lcm/lcm-cpp.hpp>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include "lcmtypes/vicon_state_t.hpp"

int64_t utime_now (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int main(int argc, char ** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    vicon_state_t vicon_state;

    while(1) {

    	srand(time(NULL));

        vicon_state.timestamp = utime_now();

    	vicon_state.position[0] = rand()%40000/10.0 - 2000;
    	vicon_state.position[1] = rand()%40000/10.0 - 2000;
    	vicon_state.position[2] = rand()%20000/10.0;

    	vicon_state.velocity[0] = rand()%20/10.0 - 1.0; 
    	vicon_state.velocity[1] = rand()%20/10.0 - 1.0; 
    	vicon_state.velocity[2] = rand()%20/10.0 - 1.0; 

    	lcm.publish("vicon_state", &vicon_state);

    	usleep(1.6e4);
    }



    return 0;
}