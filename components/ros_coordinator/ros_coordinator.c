/**
 * Made by Rui B.S.
 * Date: 23/05/226
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "ros_coordinator.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Msg types
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>

#include "led.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define N_2_HANDLE 2 // How many elements the executor need to handle (timer, subs...)

// Test pub
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// IMU pub & msg
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;


// Sub
rcl_subscription_t test_sub;
std_msgs__msg__Int32 recv_msg;


static bool is_init;

void blink(int blinks, unsigned long time) {
    for (int x = 0; x < blinks; x++) {
        // Blink esp led
        led_on(LED_ESP);
        vTaskDelay(pdMS_TO_TICKS(time));
        led_off(LED_ESP);
        vTaskDelay(pdMS_TO_TICKS(time));
    }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time, uintptr_t arg)
{
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(arg);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
        //blink(1, 100);

        int64_t now_ms = rmw_uros_epoch_millis();
        imu_msg.header.stamp.sec = now_ms / 1000;
        imu_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000;
        
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        imu_msg.angular_velocity.x = 0.01;
        imu_msg.angular_velocity.y = -0.02;
        imu_msg.angular_velocity.z = 0.005;

        imu_msg.linear_acceleration.x = 0.1;
        imu_msg.linear_acceleration.y = -0.1;
        imu_msg.linear_acceleration.z = 9.81;
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));


    }
}

void test_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * new_msg = (const std_msgs__msg__Int32 *)msgin;
    msg = *new_msg;
}

void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    //RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif


    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "xwing_drone", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "freertos_int32_publisher"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_pub"));

    // Create sub
    RCCHECK(rclc_subscription_init_default(&test_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "test_sub"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, N_2_HANDLE, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &test_sub, &recv_msg,
		&test_callback, ON_NEW_DATA));

    msg.data = 0;

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_publisher_fini(&imu_pub, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void ros_init(void)
{
    if (is_init) {
        return;
    }

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACKs,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    is_init = true;
}

bool ros_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}