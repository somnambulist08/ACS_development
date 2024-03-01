#include <math.h>
#define _USE_MATH_DEFINES
#include <cmath>
struct Quaternion;
struct EulerAngles;
static Quaternion q;
static EulerAngles e;
static Quaternion q_origin;
static EulerAngles e_origin;
Quaternion SAAM(float a[3], float m[3]);
Quaternion hamProduct(Quaternion a, Quaternion b);
EulerAngles toEulerAngles(Quaternion q);
int zerocal();