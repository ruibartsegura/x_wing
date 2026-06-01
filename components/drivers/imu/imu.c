/**
 * Made by Rui B.S.
 * Date: 23/05/226
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "imu.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool is_init = false;

void imu_init(void) {
    if (is_init) {
        return;
    }

    is_init = true;
}

bool imu_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}
