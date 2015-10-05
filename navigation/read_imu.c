#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <lcm/lcm.h>
#include "../lcmtypes/state_t.h"
#include "../include/asctec.h"

int main(int argc, char *argv[])
{
	char buf[512];
	int i;
	POLL_HEADER* pHead;
	POLL_FOOTER* pFoot;
	struct termios port_settings; // structure to store the port settings in
	int fd;
	int size;

	lcm_t * lcm = lcm_create(NULL);
	if(!lcm) return 1;

	state_t imu_msg;
	// Initializing the Serial Port
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)
	{
		printf("ERROR: Can't open /dev/ttyUSB0. \n");
		return -1;
	}
	else
	{
		printf("/dev/ttyUSB0 opened correctly.\n");
	};

	cfsetispeed(&port_settings, B57600);    // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	// This requests LL Status and IMU CalcData
	// 0x0001 + 0x0004 = 0x0005
	POLL_REQUEST req = { {'>', '*', '>', 'p'}, 0x0004 };

	while(1) {
		size = write(fd, &req, sizeof(POLL_REQUEST));
		fsync(fd);
		if (size == 0)
		{
			printf("ERROR: 0 bytes written. Return. \n");
			return -1;
		}

		// wait until data arrives
		usleep(100000);
		
		// now read the data
		size = read(fd, buf, sizeof(buf));

		if (size == -1)
		{
			printf("ERROR: Reading error. Return.\n");
			return -1;
		}
		else if (size == 0)
		{
			printf("ERROR: Nothing to read. Return. \n");
			return -1;
		}

		// This loops through all received packets and prints their infos
		for (i = 0; i < size;)
		{
			pHead = (POLL_HEADER*) &buf[i];
			i += sizeof(POLL_HEADER);

			switch (pHead->packet_desc)
			{
				case 0x03:	// PD_IMUCALCDATA
				{
					IMU_CALCDATA* pCalcData = (IMU_CALCDATA*)&buf[i];

					imu_msg.accel[0] = pCalcData->acc_x_calib;
					imu_msg.accel[1] = pCalcData->acc_y_calib;
					imu_msg.accel[2] = pCalcData->acc_z_calib;
					state_t_publish(lcm,"imu_msg",&imu_msg);
					// printf("angle_nick: %d\n", pCalcData->angle_nick);
					// printf("angle_roll: %d\n", pCalcData->angle_roll);
					// printf("angle_yaw: %d\n", pCalcData->angle_yaw);
				}
				break;
			}

			i += pHead->length;

			pFoot = (POLL_FOOTER*) &buf[i];

			i += sizeof(POLL_FOOTER);
		}
	}
	close(fd);
	lcm_destroy(lcm);
	return 0;
}
