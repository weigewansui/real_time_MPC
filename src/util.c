#include <stdlib.h>
#include <sys/time.h>

#include "../include/util.h"

#define PI 3.141592653

int64_t utime_now (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


float rad2deg(float radius) {

	float degree = radius / PI * 180;
	return radius;

}

float deg2rad(float degree) {

	float radius = degree / 180 * PI;
	return radius;
}
