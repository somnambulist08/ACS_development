/******************************************************************************
* InternalSensors.hh
*
* includes the classes and functions related to making the internal
* sensors do their job
*
* 02/15/2024 - Created file
******************************************************************************/


#ifndef INTERNAL_SENSORS_HH
#define INTERNAL_SENSORS_HH
#define accelerint(a0, a1, dt) (dt * (a1 + a0) / 2.0)  //define because faster than function calls

#define RED 22
#define BLUE 23
#define GREEN 24

#include "Arduino_BMI270_BMM150.h"  //IMU
#include <Arduino_HS300x.h>         //temp and humidity
#include <Arduino_LPS22HB.h>        //pressure
#include <math.h>
#include "GetData.hh"

#endif  //INTERNAL_SENSORS_HH
