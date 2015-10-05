#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
int64_t utime_now (void);
float rad2deg (float);
float deg2rad (float);
#ifdef __cplusplus
}
#endif

#endif
