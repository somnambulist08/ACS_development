#ifndef SAAM_HH
#define SAAM_HH

#include <math.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <Arduino.h>

#define RED 22 
#define BLUE 24

struct Quaternion {
  float w, x, y, z;
};
struct EulerAngles {
  float yaw, pitch, roll;
};
extern Quaternion q_origin;
extern EulerAngles e_origin;
Quaternion SAAM(float a[3], float m[3]);
Quaternion hamProduct(Quaternion a, Quaternion b);
EulerAngles toEulerAngles(Quaternion q);
Quaternion conjugate(Quaternion a);
Quaternion rotDiff(Quaternion q1, Quaternion q2);
int zerocal(float &ax, float &ay, float &az, float &mx, float &my, float &mz);

#endif //SAAM_HH