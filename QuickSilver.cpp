// implemented following the QuickSilver project and Kevin Plaizier knowledge ;)
// QuickSilver https://github.com/BossHobby/QUICKSILVER/blob/develop/src/flight/imu.c
#include "QuickSilver.hh"

// acc should be in units of g's and gyro in rad/s
// acc should be filtered at this point
void QuickSilver::update_estimate(float acc[], float gyro[], float dt) { // TODO potentially add a vay to disable acc entirely during some parts of flight
    float roll_delta = gyro[ROLL] * dt;
    float pitch_delta = gyro[PITCH] * dt;
    float yaw_delta = gyro[YAW] * dt;

    gravity_vector[VEC_Z] -= roll_delta * gravity_vector[VEC_X];
    gravity_vector[VEC_X] += roll_delta * gravity_vector[VEC_Z];

    gravity_vector[VEC_Y] += pitch_delta * gravity_vector[VEC_Z];
    gravity_vector[VEC_Z] -= pitch_delta * gravity_vector[VEC_Y];

    gravity_vector[VEC_X] -= yaw_delta * gravity_vector[VEC_Y];
    gravity_vector[VEC_Y] += yaw_delta * gravity_vector[VEC_X];

    // not doing sqrt to save a little cpu
    float acc_mag_squared = acc[VEC_X] * acc[VEC_X] + acc[VEC_Y] * acc[VEC_Y] + acc[VEC_Z] * acc[VEC_Z];
    if (acc_mag_squared > 1.1 || acc_mag_squared < 0.9) { // todo test to see if this window is too small
        for (int axis = 0; axis < 3; axis++) {
            // slowly fuse the estimate towards the acc reading
            gravity_vector[axis] += beta * (acc[axis] - gravity_vector[axis]);
        }
    }

    // normalize the gravity_vector
    float gravity_mag = sqrt(gravity_vector[VEC_X] * gravity_vector[VEC_X] + gravity_vector[VEC_Y] * gravity_vector[VEC_Y] + gravity_vector[VEC_Z] * gravity_vector[VEC_Z]);
    for (int axis = 0; axis < 3; axis++) {
        gravity_vector[axis] /= gravity_mag;
    }
};

// this is just the magnitude of the vector projection of acceleration onto the gravity_vector
// acc should be filtered when we run this equation
float QuickSilver::vertical_acceleration_from_acc(float acc[]) {
    // v*w/||w||^2 * w gives us the projection of v onto w
    float dot_product = gravity_vector[VEC_X] * acc[VEC_X] + gravity_vector[VEC_Y] * acc[VEC_Y] + gravity_vector[VEC_Z] * acc[VEC_Z];

    // below line not needed, should always return 1 as its a normalized vector :) nice.
    // float gravity_mag_squared = gravity_vector[VEC_X] * gravity_vector[VEC_X] + gravity_vector[VEC_Y] * gravity_vector[VEC_Y] + gravity_vector[VEC_Z] * gravity_vector[VEC_Z];
    float gravity_mag_squared = 1.0f;

    float projection[3] = {
        (dot_product / gravity_mag_squared) * gravity_vector[VEC_X],
        (dot_product / gravity_mag_squared) * gravity_vector[VEC_Y],
        (dot_product / gravity_mag_squared) * gravity_vector[VEC_Z]
        };
    float projection_magnitude = sqrt(projection[VEC_X] * projection[VEC_X] + projection[VEC_Y] * projection[VEC_Y] + projection[VEC_Z] * projection[VEC_Z]);

    // give the magnitude a negative value if its pointing opposite the direction of gravity, aka further than 90 degrees
    if (dot_product < 0.0) {
        projection_magnitude = -projection_magnitude;
    }

    return projection_magnitude;
};

void QuickSilver::initialize(float b) {
    beta = b;
    gravity_vector[VEC_X] = 0.0;
    gravity_vector[VEC_Y] = 0.0;
    gravity_vector[VEC_Z] = 1.0;
}

float* QuickSilver::getGravityVector(){
    return gravity_vector;
}
