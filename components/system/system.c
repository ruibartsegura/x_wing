#include <stdbool.h>
#include <stdint.h>

#include "system.h"

// Free RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Drivers
#include "led.h"

// Microros
#include "ros_coordinator.h"

static bool is_init = false;

void system_init(void) {
    if (is_init) {
        return;
    }

    led_init(); // Primero en init

    vTaskDelay(pdMS_TO_TICKS(5000));

    ros_init(); // último en init

    is_init = true;
}

bool system_test(void) {
    bool test = is_init;

    test &= led_test();
    test &= ros_test();

    return test;
}

void system_launch(void) {
    system_init();

    // if (!system_test()) {
    //     led_on(LED_RED);
    //     return;
    // }
}

// TODO
//      Encender led cuando sistema iniciado
