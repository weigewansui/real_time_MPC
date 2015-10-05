#include "../include/asctecCommIntf.h"

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <stdint.h>
#include <lcm/lcm.h>
#include "../lcmtypes/state_t.h"
#include "../include/util.h"
#include "../include/filter_util.h"

#define NUM_SAMPLES_MED 5
#define NUM_SAMPLES_AVG 5
#define bias_x 498.9157-646.8957
#define bias_y 81.2244 + 149.4496

int fd;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;
int32_t pitch_vel, roll_vel, yaw_vel;
int16_t acc_x, acc_y, acc_z;
int16_t ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7;
int16_t rpm1, rpm2, rpm3, rpm4;

unsigned char var_getted;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished(void);
void *aciThread(void);


lcm_t* lcm;
state_t state_msg;

//filters
double accel_x[NUM_SAMPLES_MED], accel_y[NUM_SAMPLES_MED], accel_z[NUM_SAMPLES_MED];
double accel_x_med[NUM_SAMPLES_AVG], accel_y_med[NUM_SAMPLES_AVG], accel_z_med[NUM_SAMPLES_AVG];
double accel_x_avg[NUM_SAMPLES_AVG], accel_y_avg[NUM_SAMPLES_AVG], accel_z_avg[NUM_SAMPLES_AVG];
double accx, accy, accz;

int main(int argc, char *argv[]) {

	var_getted=0;
	angle_pitch = 0;
	angle_roll = 0;
	angle_yaw = 0;
	pthread_t p_acithread;

	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B57600);    // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	lcm = lcm_create(NULL);
	if(!lcm) return 1;

	aciInit();
	aciSetSendDataCallback(&transmit);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetEngineRate(100, 10);

	pthread_create( &p_acithread, NULL, aciThread, NULL);
	aciGetDeviceVariablesList();
	pthread_join( p_acithread, NULL);

	lcm_destroy(lcm);
}

void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *)byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
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

		if(var_getted) {

			aciSynchronizeVars();
			
			accx = acc_x*1.0 - bias_x;
			accy = acc_y*1.0 - bias_y;
			accz = acc_z*1.0;

			addArr(accel_x, accx, NUM_SAMPLES_MED);
			addArr(accel_y, accy, NUM_SAMPLES_MED);
			addArr(accel_z, accz, NUM_SAMPLES_MED);

			addArr(accel_x_med, median(accel_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
			addArr(accel_y_med, median(accel_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
			addArr(accel_z_med, median(accel_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);

			addArr(accel_x_avg, average(accel_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
			addArr(accel_y_avg, average(accel_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
			addArr(accel_z_avg, average(accel_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);


			state_msg.timestamp = utime_now();
			// state_msg.accel[0] = acc_x;
			// state_msg.accel[1] = acc_y;
			// state_msg.accel[2] = acc_z;
			state_msg.accel[0] = accel_x_avg[NUM_SAMPLES_AVG - 1];
			state_msg.accel[1] = accel_y_avg[NUM_SAMPLES_AVG - 1];
			state_msg.accel[2] = accel_z_avg[NUM_SAMPLES_AVG - 1];

			state_msg.angle[0] = angle_roll;
			state_msg.angle[1] = angle_pitch;
			state_msg.angle[2] = angle_yaw;
			state_msg.angular_vel[0] = roll_vel;
			state_msg.angular_vel[1] = pitch_vel;
			state_msg.angular_vel[2] = yaw_vel;
			
			state_t_publish(lcm, "state", &state_msg);
		}

		aciEngine();
		usleep(10000);
	}
	return NULL;
}

void varListUpdateFinished(void) {

	printf("hello\n");

	aciAddContentToVarPacket(0, 0x0300, &angle_pitch);
	aciAddContentToVarPacket(0, 0x0301, &angle_roll);
	aciAddContentToVarPacket(0, 0x0302, &angle_yaw);
	aciAddContentToVarPacket(0, 0x0200, &pitch_vel);
	aciAddContentToVarPacket(0, 0x0201, &roll_vel);
	aciAddContentToVarPacket(0, 0x0202, &yaw_vel);

	aciAddContentToVarPacket(0, 0x0203, &acc_x);
	aciAddContentToVarPacket(0, 0x0204, &acc_y);
	aciAddContentToVarPacket(0, 0x0205, &acc_z);

	aciSetVarPacketTransmissionRate(0,10);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(0);
	var_getted=1;

}
