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

#define CONTROL_P       (-0.001f)
#define CONTROL_I       (-0.0005f)
#define CONTROL_BIAS    (0.314159265358979f)//(M_PI/10.0f)
 
extern float integratorState;
float getControl(float desired, float predicted, float dt);



#endif //CONTROL_HH
