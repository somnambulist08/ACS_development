#include "Control.hh"
#include <math.h>


float integratorState = 0;
float getDesired(float time) {
  return 480.0f * powf(M_E, -time * 3.0f / 10.0f) + 1143.0f;
}


/*
float predictAltitude(float height, float velocity){
  return height + velocity*velocity/2/9.81;
}*/

float getControl(float desired, float predicted, float dt){
  float err = desired - predicted;
  float control = Pee * err; 
  integratorState += Iye*err*dt;
 
  control += PI/6;
  if (control > PI/2) control = PI/2;
  if (control < 0) control = 0;
  return control;
}