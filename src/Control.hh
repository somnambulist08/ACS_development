/******************************************************************************
* Control.hh
*
* Contains all the control functions and constants
*
* 02/15/2024 - Created file
* 02/29/2024 - Updated with newest equations and values
******************************************************************************/


#ifndef CONTROL_HH
#define CONTROL_HH
#include <math.h>


float getDesired(float time);

float predictAltitude(float height, float velocity);

#define CONTROL_P       (-0.005f)
#define CONTROL_I       (-0.001f)
#define CONTROL_BIAS    (M_PI / 8.0f)
// #define UPPER_CONTROL_SATURATION M_PI_2
#define UPPER_CONTROL_SATURATION (2.0f * M_PI / 5.0f) //roughly 73 degrees
 
extern float integratorState;
float getControl(float desired, float predicted, float dt);



#endif //CONTROL_HH
