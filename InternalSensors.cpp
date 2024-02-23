#include "InternalSensors.hh"
#include <math.h>

/*
long accelerint(float a0, float a1, float dt) {
  return dt * (a1 + a0) / 2.0;  //central Reimann sum
}*/

float pressureAlt(float pressure) {
  return (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;
}

