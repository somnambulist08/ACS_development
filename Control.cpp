#include "Control.hh"


float getDesired(float time) {
  return 2200.0f * powf(M_E, -time) + 1219.2f;
}


float predictAltitude(float height, float velocity){
  return height + velocity*velocity/2.0f/9.81f;
}

float integratorState = 0;
float getControl(float desired, float predicted, float dt){
  float err = desired - predicted;
  float control = CONTROL_P * err; 
  integratorState += CONTROL_I*err*dt;
 
  control += CONTROL_BIAS;
  if (control > M_PI_2) control = M_PI_2;
  if (control < 0) control = 0;
  return control;
}