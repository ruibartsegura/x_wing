#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

#include "sdkconfig.h"


// PIN configuration



void imu_init(void);
bool imu_test(void); // Check leds

#endif // IMU_H
