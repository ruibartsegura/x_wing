#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "sdkconfig.h"
#include <stdbool.h>

// X/100 due to the Configuration it is in cm
float hov_h = CONFIG_HOVERING_H / 100;
float vel_max = CONFIG_VEL_MAX / 100;
bool land_on_site = CONFIG_LAND_ON_SITE;

void set_hovering_h(float h);
float get_hovering_h();

void set_max_velocity(float vel);
float get_max_velocity();

void set_land_on_site(bool mode);
bool get_land_on_site();


#endif // PARAMETERS_H
