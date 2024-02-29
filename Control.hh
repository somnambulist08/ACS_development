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

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define CONTROL_P       (-0.001f)
#define CONTROL_I       (-0.0005f)
#define CONTROL_BIAS    (PI/8.0f)
 
extern float integratorState;
float getControl(float desired, float predicted, float dt);



#endif //CONTROL_HH
