#ifndef CMMD_H
#define CMMD_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <iostream>
#include <string>

#include "asctec.h"
#include "address.h"

using namespace std;

class LLP_PORT {
protected:

	termios port_settings;
	string device_name;
	int device_settings;
	int baud_rate; // set baud rate
	int fd;

public:

	LLP_PORT(); //initialize settings
	LLP_PORT(termios, string, int, int); //store port settings and port name
	void open_port();
	void close_port();
}; 

#endif