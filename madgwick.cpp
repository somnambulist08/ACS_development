// implemented following https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
#include "madgwick.hh"
// helper methods and shit
/*madgwick::madgwick(){};
private struct Quaternion{
    float w = 0.0;
    float x, y, z;
};
private struct Eulers{
    float psi, theta, phi;
}; 
Quaternion madgwick::product(Quaternion a, Quaternion b){
    Quaternion q;
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return q;
}
Eulers madgwick::toEulers(Quaternion q) {
    Eulers e;
    e.psi = atan2(2.0 * q.x * q.y - 2.0 * q.w * q.z, 2.0 * q.w * q.w + 2.0 * q.x * q.x - 1.0);
    e.theta = -asin(2.0 * q.x * q.z + 2.0 * q.w * q.y);
    e.phi = atan2(2.0 * q.y * q.z - 2.0 * q.w * q.x, 2.0 * q.w * q.w + 2.0 * q.z * q.z - 1.0);
    return e;
}*/