#include "../include/llp_nav.h"

LLP_NAV::LLP_NAV() : LLP_PORT(){}
LLP_NAV::LLP_NAV(termios prt_settings, string dName, int dSettings, int bRate) : LLP_PORT(prt_settings, dName, dSettings, bRate) {}

IMU_CALCDATA LLP_NAV::read_imu() {

	IMU_CALCDATA imu_ret;

	char buf[512];
	int i;
	POLL_HEADER* pHead;
	POLL_FOOTER* pFoot;
	int size;

	POLL_REQUEST req = { {'>', '*', '>', 'p'}, IMU_CALC_ADD };
	size = write(fd, &req, sizeof(POLL_REQUEST));
	fsync(fd);
	// printf("%d bytes written.\n", size);
	if (size == 0)
	{
		printf("ERROR: 0 bytes written. Return. \n");
		exit(EXIT_FAILURE);
	}

	// wait until data arrives
	usleep(100000);
	
	// now read the data
	size = read(fd, buf, sizeof(buf));

	if (size == -1)
	{
		printf("ERROR: Reading error. Return.\n");
		exit(EXIT_FAILURE);
	}
	else if (size == 0)
	{
		printf("ERROR: Nothing to read. Return. \n");
		exit(EXIT_FAILURE);
	}

		i = 0;
		pHead = (POLL_HEADER*) &buf[i];
		i += sizeof(POLL_HEADER);
		IMU_CALCDATA* pCalcData = (IMU_CALCDATA*)&buf[i];
		imu_ret = *pCalcData;
		// i += pHead->length;
		// pFoot = (POLL_FOOTER*) &buf[i];
		// i += sizeof(POLL_FOOTER);

		return imu_ret;
	}

RC_DATA LLP_NAV::read_rc() {

	RC_DATA rc_ret;

	char buf[512];
	int i;
	POLL_HEADER* pHead;
	POLL_FOOTER* pFoot;
	int size;

	POLL_REQUEST req = { {'>', '*', '>', 'p'}, RC_DATA_ADD };
	size = write(fd, &req, sizeof(POLL_REQUEST));
	fsync(fd);
	// printf("%d bytes written.\n", size);
	if (size == 0)
	{
		printf("ERROR: 0 bytes written. Return. \n");
		exit(EXIT_FAILURE);
	}

	// wait until data arrives
	usleep(100000);
	
	// now read the data
	size = read(fd, buf, sizeof(buf));

	if (size == -1)
	{
		printf("ERROR: Reading error. Return.\n");
		exit(EXIT_FAILURE);
	}
	else if (size == 0)
	{
		printf("ERROR: Nothing to read. Return. \n");
		exit(EXIT_FAILURE);
	}

		i = 0;
		pHead = (POLL_HEADER*) &buf[i];
		i += sizeof(POLL_HEADER);
		RC_DATA* pCalcData = (RC_DATA*)&buf[i];
		rc_ret = *pCalcData;
		// i += pHead->length;
		// pFoot = (POLL_FOOTER*) &buf[i];
		// i += sizeof(POLL_FOOTER);

		return rc_ret;
}

