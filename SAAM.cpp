#include "SAAM.hh"

struct Quaternion {
  float w, x, y, z;
};
struct EulerAngles {
  float yaw, pitch, roll;
};

// sensor fusion following
// https://inria.hal.science/hal-01922922/document
Quaternion SAAM(float a[3], float m[3]) {
  Quaternion q;
  float anorm = sqrt(pow(a[0], 2) + pow(a[1], 2) + pow(a[2], 2));
  float ax = a[0] / anorm;
  float ay = a[1] / anorm;
  float az = a[2] / anorm;
  float mnorm = sqrt(pow(m[0], 2) + pow(m[1], 2) + pow(m[2], 2));
  float mx = m[0] / mnorm;
  float my = m[1] / mnorm;
  float mz = m[2] / mnorm;
  float magD = ax * mx + ay * my + az * mz;
  float magN = sqrt(1.0f - magD * magD);
  float q0 = -ay * (magN + mx) + ax * my;
  float q1 = (az - 1.0f) * (magN + mx) + ax * (magD - mz);
  float q2 = (az - 1.0f) * my + ay * (magD - mz);
  float q3 = az * magD - ax * magN - mz;

  float qnorm = sqrt(pow(q0, 2) + pow(q1, 2) + pow(q2, 2) + pow(q3, 2));
  q.w = q3 / qnorm;
  q.x = q0 / qnorm;
  q.y = q1 / qnorm;
  q.z = q2 / qnorm;
  return q;
}
//yaw and roll are transposed here since the z axis points to the nose instead of x
EulerAngles toEulerAngles(Quaternion q) {
  EulerAngles angles;
  //yaw - X AXIS rotation
  float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1.0f- 2.0f * (std::pow(q.x,2.0) + std::pow(q.y,2.0));
  angles.yaw = std::atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  float sinp = std::sqrt(1.0f + 2.0f * (q.w * q.y - q.x * q.z));
  float cosp = std::sqrt(1.0f - 2.0f * (q.w * q.y - q.x * q.z));
  angles.pitch = 2.0f * std::atan2(sinp, cosp) - M_PI / 2.0f;
  // roll - Z AXIS rotation
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (std::pow(q.y,2.0) + std::pow(q.z,2.0));
  angles.roll = std::atan2(siny_cosp, cosy_cosp);
  return angles;
}
void zerocal() {
  while (abs(acc_z) != 1) {
    if (sixteenIMU.accelerationAvailable()) {
      sixteenIMU.readAcceleration(acc_x, acc_y, acc_z);    
    }
    if (sixteenIMU.magneticFieldAvailable()) {
      sixteenIMU.readMagneticField(mag_x, mag_y, mag_z);
    }
    if (abs(acc_z) > 1) {
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);
    } else {
      digitalWrite(RED, HIGH);
      digitalWrite(BLUE, LOW);
    }
  }
  float accs[3] = { acc_x, acc_y, acc_z };
  float mags[3] = { mag_x, mag_y, mag_z };
  q_origin = SAAM(accs, mags);
  e_origin = toEulerAngles(q_origin);
  delay(50);
  digitalWrite(RED, LOW);
  delay(50);
  digitalWrite(BLUE, LOW);
  delay(50);
}