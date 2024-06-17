#include "Control.hh"


float getDesired(float time) {
  return 1600.0f * powf(M_E, -1.0f/3.0f * time) + 3048.0f;
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
  if (control > UPPER_CONTROL_SATURATION) control = UPPER_CONTROL_SATURATION;
  if (control < 0) control = 0;
  return control;
}