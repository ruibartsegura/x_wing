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

// How much error it's admitted in the altitude
#define ALTITUDE_ERROR 5

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


// Change the state-machine state 
void change_state(int new_state) {
    state_ = new_state;
}

// Get the state-machine state 
int get_state() {
    return state_;
}

// Check if the take_off has reached the desired altitude
bool check_takeOff_2_hov(float hov_h) {
    float h = get_h();

    float above_error, below_error;
    above_error = hov_h + hov_h * (ALTITUDE_ERROR/100);
    below_error = hov_h - hov_h * (ALTITUDE_ERROR/100);

    if(h > below_error && h < above_error) {
        return true;
    } else {
        return false;
    }
}

void system_launch(void) {

    switch (state_) {
        case INIT:
            system_init();
            
            change_state(CHECKING);
            
            break;

        case CHECKING:
            // While this state the params can be updated
            if (system_test()) {
                change_state(ARMING);
            } else {
                change_state(ERROR);
            }
            break;

        case ARMING:
            // start_engine();

            if(get_take_off_ready()) {
                change_state(TAKING_OFF);
            } else {
                // Check if take_off service has arrive each 150ms
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            break;

        case TAKING_OFF:
            // go_to(0, 0, get_take_off_alt());

            if (check_takeOff_2_hov(get_take_off_alt())) {
                change_state(HOVERING);
            }
            break;

        case HOVERING:
            // go_to(x, y, get_take_off_alt());

            // if (get_new_pos()) {
            //     change_state(EXTERNAL_CONTROL);
            // }

            // if (get_land()) {
            //     change_state(LANDING);
            // }
            break;

        case EXTERNAL_CONTROL:
            // go_to(x, y, h);

            // if (!get_new_pos()) {
            //     change_state(HOVERING);
            // }

            // if (get_land()) {
            //     change_state(LANDING);
            // }
            break;

        case LANDING:
            // go_to(x, y, 0);

            if (get_land()) {
                change_state(LANDING);
            }
            break;

        case DISARMING:
            // shut_down_engine();
            break;

        case ERROR:
            break;

    }
}

// TODO
//      Encender led cuando sistema iniciado
// 
//      Implementar get_take_off() en ros_coordinator
// 
//      Implementar go_to(x, y, h) en X
//
//      Implementar params en configuration, hacer get/set para modificarlo con ros_coordinator
//
//      
