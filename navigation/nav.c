#include <stdio.h>
#include <lapacke.h>
#include <inttypes.h>
#include <pthread.h>
#include <lcm/lcm.h>
#include <math.h>
#include "../lcmtypes/state_t.h"
#include "../lcmtypes/vicon_state_t.h"
#include "../lcmtypes/real_accel_t.h"

#define dT = 0.01


lcm_t * lcm;
state_t imu_msg;
vicon_state_t vicon_msg;
real_accel_t real_accel_msg;

double ax, ay, az;
double phi, theta, psi;
double g = 988.55;
void* updateState(void);
void* getWorldAccel(void);

static void imu_msg_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const state_t * msg, void * user)
{
	imu_msg = *msg;
}

static void vicon_msg_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const vicon_state_t * msg, void * user)
{
	vicon_msg = *msg;
}

int main(int argc, char ** argv)
{

    lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    state_t_subscribe(lcm, "state", &imu_msg_handler, NULL);
    // printf("%d\n", __LINE__);
    vicon_state_t_subscribe(lcm, "vicon_state", &vicon_msg_handler, NULL);  
    // printf("%d\n", __LINE__);

    pthread_t p_updateState, p_getWorldAccel;
    // printf("%d\n", __LINE__);

    pthread_create(&p_updateState, NULL, updateState, NULL);
    // printf("%d\n", __LINE__);
    
    pthread_create(&p_getWorldAccel, NULL, getWorldAccel, NULL);
    // printf("%d\n", __LINE__);

    pthread_join(p_updateState, NULL);
    pthread_join(p_getWorldAccel, NULL);      

    lcm_destroy(lcm);
    return 0;

}

void* updateState(void) {

	while(1) {
		lcm_handle(lcm);
	}

}

void* getWorldAccel(void) {
    // printf("%d\n", __LINE__);

	while(1) {
    // printf("%d\n", __LINE__);

		// ax = vicon_msg.DCM[0]*imu_msg.accel[0] + vicon_msg.DCM[3]*imu_msg.accel[1] + vicon_msg.DCM[6]*imu_msg.accel[2];		
		// ay = vicon_msg.DCM[1]*imu_msg.accel[0] + vicon_msg.DCM[4]*imu_msg.accel[1] + vicon_msg.DCM[7]*imu_msg.accel[2];
		// az = vicon_msg.DCM[2]*imu_msg.accel[0] + vicon_msg.DCM[5]*imu_msg.accel[1] + vicon_msg.DCM[8]*imu_msg.accel[2] + g; 
		ax = vicon_msg.DCM[0]*imu_msg.accel[0] + vicon_msg.DCM[1]*imu_msg.accel[1] + vicon_msg.DCM[2]*imu_msg.accel[2];		
		ay = vicon_msg.DCM[3]*imu_msg.accel[0] + vicon_msg.DCM[4]*imu_msg.accel[1] + vicon_msg.DCM[5]*imu_msg.accel[2];
		az = vicon_msg.DCM[6]*imu_msg.accel[0] + vicon_msg.DCM[7]*imu_msg.accel[1] + vicon_msg.DCM[8]*imu_msg.accel[2] - g;
		// phi = vicon_msg.attitude[0];
		// theta = vicon_msg.attitude[1];
		// psi = vicon_msg.attitude[2];
		// ax = cos(theta)*cos(psi)*imu_msg.accel[0] - cos(theta)*sin(psi)*imu_msg.accel[1] + sin(theta)*imu_msg.accel[2];
		// ay = (-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi))*imu_msg.accel[0] + 
		// 		(cos(phi) * cos(psi) - sin(phi)*sin(theta)*sin(psi))*imu_msg.accel[1] + sin(phi)*cos(theta)*imu_msg.accel[2];
		// az = (sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi))*imu_msg.accel[0] + 
		// 		(-sin(phi)*cos(psi) - cos(phi)*sin(theta)*sin(psi))*imu_msg.accel[1] + cos(phi)*cos(theta)*imu_msg.accel[2];
		
		// ax = imu_msg.accel[0]*cos(psi) - imu_msg.accel[1]*sin(psi);
		// ay = -imu_msg.accel[0]*sin(psi) - imu_msg.accel[1]*cos(psi);
		// az = imu_msg.accel[2] + 9910.0;

		real_accel_msg.accel[0] = ax;
    // printf("%d\n", __LINE__);

		real_accel_msg.accel[1] = ay;
    // printf("%d\n", __LINE__);

		real_accel_msg.accel[2] = az;	
    // printf("%d\n", __LINE__);

		real_accel_t_publish(lcm, "real_accel", &real_accel_msg);
		usleep(10000);
	}

}