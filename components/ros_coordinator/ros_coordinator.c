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
#include <rosidl_runtime_c/string_functions.h>

// Msg types
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <my_msgs/msg/params.h>

// Services types
#include <my_msgs/srv/takeoff.h>


#include "led.h"
#include "system.h"
#include "parameters.h"

#include <pthread.h>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define N_HANDLERS 5 // How many elements the executor need to handle (timer, subs, services)
static const float MAX_ALT = 3.0f; // Max altitude allowed in meters
static const float MIN_ALT = 0.5f; // Min altitude allowed in meters

// IMU pub & msg
// rcl_publisher_t imu_pub;
// sensor_msgs__msg__Imu imu_msg;


// Subscribers
rcl_subscription_t params_sub;
std_msgs__msg__Int32 recv_msg;

// Params
my_msgs__msg__Params param_msg;
static bool params_2_update = false;

// Serviecs
rcl_service_t takeoff_srv;
my_msgs__srv__Takeoff_Request  takeoff_req;
my_msgs__srv__Takeoff_Response takeoff_res;


// Mutex
pthread_mutex_t lock;


static bool is_init = false;

// Receive and manage parameters
void apply_pending_params() {
    if (params_2_update) {
        pthread_mutex_lock(&lock);

        set_hovering_h(param_msg.h_max);
        set_max_velocity(param_msg.v_max);
        set_land_on_site(param_msg.land_on_site);

        params_2_update = false;

        pthread_mutex_unlock(&lock);
    }
}
void param_callback(const void * msgin)
{
    const my_msgs__msg__Params * new_msg = (const my_msgs__msg__Params *)msgin;
    param_msg = *new_msg;
    params_2_update = true;
}

// Manage take off
bool get_take_off_ready() {
    bool res = take_off_ready;
    take_off_ready = false;
    return res;
}

int get_take_off_alt() {
    return altitude;
}

void takeoff_callback(const void * req_msg, void * res_msg) {
    my_msgs__srv__Takeoff_Request  * takeoff_req =
        (my_msgs__srv__Takeoff_Request *)req_msg;

    my_msgs__srv__Takeoff_Response * takeoff_res =
        (my_msgs__srv__Takeoff_Response *)res_msg;

    // Check if the drone is able to take off
    if (get_state() == 8) { // 8 == Error state 
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Status error");
        return;
    } else if (get_state() != 2) { // 2 == Arming state
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Not armed");
        return;
    }
    
    // Check if the altitude is valid
    if (takeoff_req->altitude < MIN_ALT || takeoff_req->altitude > MAX_ALT) {
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Invalid altitude");
        return;
    }

    pthread_mutex_lock(&lock);
    take_off_ready = true;
    altitude = takeoff_req->altitude;
    pthread_mutex_unlock(&lock);

    takeoff_res->accepted = true;
    rosidl_runtime_c__String__assign(&takeoff_res->reason, "OK");
}


// Timer
void timer_callback(rcl_timer_t * timer, int64_t last_call_time, uintptr_t arg) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(arg);
    if (timer != NULL) {
        int64_t now_ms = rmw_uros_epoch_millis();
        
        if (get_state() == 1 && params_2_update) {
            apply_pending_params()
        } 
    }
}


// Ros declarations
void micro_ros_task(void * arg) {
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


    // Create sub
    // Params Sub - qos
    rmw_qos_profile_t params_qos = rmw_qos_profile_default;

    params_qos.reliability  = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    params_qos.durability   = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    params_qos.history      = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    params_qos.depth        = 2;  // queue depth

    RCCHECK(rclc_subscription_init(
        &params_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(my_msgs, msg, Params),
        "params", &params_qos));


    // Create services
    RCCHECK(rclc_service_init_default(
        &takeoff_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_msgs, srv, Takeoff),
        "takeoff_srv"
    ));

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

    // Add everything to the executor
    RCCHECK(rclc_executor_init(&executor, &support.context, N_HANDLERS, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &params_sub, &recv_msg,
		&param_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_service(&executor, &takeoff_srv, &takeoff_req, &takeoff_res, takeoff_callback));

    rclc_executor_add_timer(&executor, &timer);
    

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void ros_init(void) {
    if (is_init) {
        return;
    }

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
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



// IMU pub
// imu_msg.header.stamp.sec = now_ms / 1000;
// imu_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000;

// imu_msg.orientation.x = 0.0;
// imu_msg.orientation.y = 0.0;
// imu_msg.orientation.z = 0.0;
// imu_msg.orientation.w = 1.0;

// imu_msg.angular_velocity.x = 0.01;
// imu_msg.angular_velocity.y = -0.02;
// imu_msg.angular_velocity.z = 0.005;

// imu_msg.linear_acceleration.x = 0.1;
// imu_msg.linear_acceleration.y = -0.1;
// imu_msg.linear_acceleration.z = 9.81;
// RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));