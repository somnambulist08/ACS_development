/******************************************************************************
* Control.hh
*
* Contains all the control functions and constants
*
* 02/15/2024 - Created file
******************************************************************************/


#ifndef CONTROL_HH
#define CONTROL_HH

float getDesired(float time);




#define predictAltitude(height, velocity) (height + velocity*velocity/2.0/9.81 )
/*
float predictAltitude(float height, float velocity);
*/
#ifndef PI
#define PI 3.14159265358979323846
#endif

#define Pee -0.0006
#define Iye -0.0006
 
extern float integratorState;
float getControl(float desired, float predicted, float dt);



#endif //CONTROL_HH
