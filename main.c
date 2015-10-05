/**
 * Hovering for hummingbird with PD controller
 */

#include "include/asctecCommIntf.h"

#include <stdio.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>
#include <stdint.h>

#include <lcm/lcm.h>
#include "lcmtypes/vicon_state_t.h"
#include "lcmtypes/accel_t.h"
#include "lcmtypes/command_t.h"

#include "include/util.h"
#include "include/filter_util.h"
#define NUM_SAMPLES_MED 3
#define NUM_SAMPLES_AVG 3

//////////////////////////////////
///control stuffs
float HEIGHT = 4.3;


// #define THRUST_0 1925
#define THRUST_0 1830
#define COEFF_H 165
#define GAIN_P_H -2.4142135
#define GAIN_D_H -2.210
#define saturation_up 2400
#define saturation_down 1700

float YAW_DES = 0;
#define COEFF_YAW 717
#define yaw_0 0
#define GAIN_P_YAW -1.5
#define yaw_saturation 1500


float X_DES = 0.0;

#define COEFF_ROLL 2200
#define roll_0 0
#define GAIN_P_X -2.4142135
#define GAIN_D_X -2.210
#define roll_saturation 1500

float Y_DES = 0.0;

#define COEFF_PITCH 2200
#define pitch_0 0
#define GAIN_P_Y -2.4142135
#define GAIN_D_Y -2.210
#define pitch_satuarion 1500



///////////////////////////////////////

int fd;

unsigned char ctrl_mode = 0x02;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;

float x, y, z;
float vx, vy, vz;
float phi, theta, psi;
float v_phi, v_theta, v_psi;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;
int32_t pitch_vel, roll_vel, yaw_vel;
int32_t acc_x, acc_y, acc_z;
int16_t ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7;

unsigned char var_getted;

int16_t pitch=0;
int16_t roll=0;
int16_t yaw=0;
int16_t thrust=0;
int16_t ctrl = 0xF;

unsigned char cmd_ready = 0;
int16_t motor_start=1;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished();
void cmdListUpdateFinished();
void paramListUpdateFinished();
void *aciThread(void);
void *subscribeAccelThread(void);
void startStopMotor();
void startMotors();
void *sendCmmdThread(void);
void *publishCmmdThread(void);


lcm_t * lcm;
command_t cmmdMsg;
accel_t accel_msg;
vicon_state_t vicon_state;

float desired_x_accel, desired_y_accel, desired_z_accel;
float desired_yaw_rate, desired_pitch_angle, desired_roll_angle;

int main(int argc, char *argv[]) {

	pthread_t p_subscribeAccelThread, p_sendCmmdThread, p_publishCmmdThread;
	pthread_t p_acithread;

	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B57600); // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	aciInit();
	aciSetSendDataCallback(&transmit);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	aciSetParamListUpdateFinishedCallback(&paramListUpdateFinished);
	aciSetEngineRate(100, 10);


	pthread_create(&p_acithread, NULL, aciThread, NULL);
	aciGetDeviceCommandsList();
	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);

	printf("starting motors!\n");
	//turn on motors()
	usleep(1000000);
	
	lcm = lcm_create(NULL);
 	
 	if(!lcm) return 1;
	//start getting data from Vicon
	pthread_create(&p_subscribeAccelThread, NULL, subscribeAccelThread, NULL);
	pthread_create(&p_sendCmmdThread, NULL, sendCmmdThread, NULL);
	pthread_create(&p_publishCmmdThread, NULL, publishCmmdThread, NULL);
	
	pthread_join(p_sendCmmdThread, NULL);
  	pthread_join(p_publishCmmdThread, NULL);
  	pthread_join(p_subscribeAccelThread, NULL);
  	pthread_join(p_acithread, NULL);

}


static void accel_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const accel_t * msg, void * user)
{
    
	accel_msg = *msg;

}

static void state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const vicon_state_t * msg, void * user)
{
    
	vicon_state = *msg;

}

void *viconStateThread(void) {

	vicon_state_t_subscribe(lcm, "vicon_state", &state_handler, NULL);

	while(1) {
	
        lcm_handle(lcm);
		usleep(10000);

	}

	return NULL;

}

void *subscribeAccelThread(void) {

	accel_t_subscribe(lcm, "acceleration", &accel_handler, NULL);
	vicon_state_t_subscribe(lcm, "vicon_state", &state_handler, NULL);

	while(1) {
	
        lcm_handle(lcm);
		usleep(10000);

	}

	return NULL;

}

void *aciThread(void)
{
	int result = 0;
	unsigned char data = 0;

	while(1)
	{

		result = read(fd, &data, 1);

		while (result > 0) {
			aciReceiveHandler(data);
			result = read(fd, &data, 1);
		}

		aciEngine();
		usleep(10000);
	}

	return NULL;
}


void startMotors(int hi) {

	ctrl = 0xF;
	thrust = 0;
	roll = 0;
	pitch = 0;
	yaw = -2047;
	aciUpdateCmdPacket(0);

}


void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *) byte;

	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}

}


void *publishCmmdThread(void) {

	while(1){

		cmmdMsg.timestamp = utime_now();
		cmmdMsg.thrust = thrust;
		cmmdMsg.roll = roll;
		cmmdMsg.pitch = pitch;
		cmmdMsg.yaw = yaw;

		command_t_publish(lcm, "command_msg", &cmmdMsg);
		usleep(10000);

	}

	return NULL;

}

int THRUST_JUMP_THRES = 100, ROLL_JUMP_THRES = 50, PITCH_JUMP_THRES = 100, YAW_JUMP_THRES = 50;

void *sendCmmdThread(void) {

	while(1) {
    	
    	// acceleration in z direction is minus
    	desired_z_accel = - accel_msg.accel[2];
        thrust = (int)(COEFF_H * desired_z_accel + THRUST_0);

        if(thrust >= saturation_up) thrust = saturation_up;
        if(thrust <= saturation_down) thrust = saturation_down;


        // get the desired acceleration through LQR gains
        desired_x_accel = accel_msg.accel[0];
        desired_y_accel = accel_msg.accel[1];
        psi = vicon_state.attitude[2];

        //get angle through small angle approximation
        //
        // for x direction, pitch angle is the opposite sign as acceleration
        // for y direction, roll angle is the same sign as acceleration
        // 
        // //////////////////////////////////////
        
        desired_roll_angle = -desired_y_accel / (desired_z_accel + 9.8); 
        desired_pitch_angle = -desired_x_accel / (desired_z_accel + 9.8);

        //generation roll pitch command based on desired angle
        pitch = (int)(pitch_0 + desired_pitch_angle * COEFF_PITCH);
        roll  = (int)(roll_0 + desired_roll_angle * COEFF_ROLL);

        //yaw angle is controlled by velocity
        desired_yaw_rate = (psi - deg2rad(YAW_DES))*GAIN_P_YAW;
        yaw = (int)(yaw_0 + COEFF_YAW * desired_yaw_rate);

        // set command saturation 
        if (roll >= roll_saturation) roll = roll_saturation;
        if (roll <= -roll_saturation) roll = -roll_saturation;

        if (pitch >= pitch_satuarion) pitch = pitch_satuarion;
        if (pitch <= -pitch_satuarion) pitch = -pitch_satuarion;

        if (yaw >= yaw_saturation) yaw = yaw_saturation;
        if (yaw <= -yaw_saturation) yaw = -yaw_saturation;

        ////////////////////
        //set control mode here
        //
        //0x4 for yaw only
        //0x8 for thrust only
        //0x1 for pitch only
        //0x2 for roll only
        //
        ////////////////////
        
        ctrl = 0xF;

        //send command out
        aciUpdateCmdPacket(0);
        usleep(10000);

    }

    return NULL;
}
 
void cmdListUpdateFinished() {
	
	printf("command list getted!\n");
	aciAddContentToCmdPacket(0, 0x050A, &pitch);
	aciAddContentToCmdPacket(0, 0x050B, &roll);
	aciAddContentToCmdPacket(0, 0x050C, &yaw);
	aciAddContentToCmdPacket(0, 0x050D, &thrust);
	aciAddContentToCmdPacket(0, 0x050E, &ctrl);
	aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
	aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
	aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
	aciSendCommandPacketConfiguration(0, 0);
	// second variable, if set this not zero, it will send the last command until it gets an acknowledge
	aciSendCommandPacketConfiguration(1, 1);

	pitch = 0;
	roll = 0;
	yaw = 0;
	thrust = 0;
	ctrl = 0xF;

	ctrl_mode=0x02;
	ctrl_enabled=1;
	// disable_motor_onoff_by_stick=1;
	aciUpdateCmdPacket(0);
	aciUpdateCmdPacket(1);
	cmd_ready=1;

}

void paramListUpdateFinished() {
 
}


void varListUpdateFinished() {

}