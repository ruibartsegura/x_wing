#ifndef ROS_COORDINATOR_H
#define ROS_COORDINATOR_H

#include <stdbool.h>

void ros_init(void);
bool ros_test(void);

// take off status
bool take_off_ready = false;
int altitude;
bool get_take_off_ready(); // return the if srv arrive
int get_take_off_alt(); // return the alti

#endif // ROS_COORDINATOR_H
