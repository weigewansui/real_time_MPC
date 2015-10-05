#include "../include/llp_port.h"

LLP_PORT::LLP_PORT() {

	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;

	device_name = "/dev/ttyUSB0";
	device_settings = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK;

	baud_rate = B57600;

}

LLP_PORT::LLP_PORT(termios prt_settings, string dName, int dSettings, int bRate){

	port_settings.c_cflag = prt_settings.c_cflag;
	port_settings.c_iflag = prt_settings.c_iflag;
	port_settings.c_oflag = prt_settings.c_oflag;
	port_settings.c_lflag = prt_settings.c_lflag;

	device_name = dName;
	device_settings = dSettings;

	baud_rate = bRate;

}

void LLP_PORT::open_port() {

	fd = open(device_name.c_str(), device_settings);

	if (fd == -1)
	{
		printf("ERROR: Can't open /dev/ttyUSB0. \n");
		exit (EXIT_FAILURE);
	}
	else
	{
		// printf("/dev/ttyUSB0 opened correctly.\n");
	};

	cfsetispeed(&port_settings, baud_rate);  // set baud rates
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

}

void LLP_PORT::close_port() {

	close(fd);

}
