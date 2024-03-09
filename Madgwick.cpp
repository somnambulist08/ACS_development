// implemented following the QuickSilver project and Kevin Plaizier knowledge ;)
#include "Madgwick.hh"

// acc should be in units of g's and gyro in rad/s
// acc should be filtered at this point
Madgwick::update_estimate(float acc[], float gyro[], float dt) { // TODO potentially add a vay to disable acc entirely during some parts of flight
    float roll_delta = gyro[ROLL] * dt;
    float pitch_delta = gyro[PITCH] * dt;
    float yaw_delta = gyro[YAW] * dt;

    gravity_vector[Z] -= roll_delta * gravity_vector[X];
    gravity_vector[X] += roll_delta * gravity_vector[Z];

    gravity_vector[Y] += pitch_delta * gravity_vector[Z];
    gravity_vector[Z] -= pitch_delta * gravity_vector[Y];

    gravity_vector[X] -= yaw_delta * gravity_vector[Y];
    gravity_vector[Y] += yaw_delta * gravity_vector[X];

    // not doing sqrt to save a little cpu
    float acc_mag_squared = acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z];
    if (acc_mag_squared > 1.1 || acc_mag_squared < 0.9) { // todo test to see if this window is too small
        for (int axis = 0; axis < 3; axis++) {
            // slowly fuse the estimate towards the acc reading
            gravity_vector[axis] += beta * (acc[axis] - gravity_vector);
        }
    }

    // normalize the gravity_vector
    float gravity_mag = sqrt(gravity_vector[X] * gravity_vector[X] + gravity_vector[Y] * gravity_vector[Y] + gravity_vector[Z] * gravity_vector[Z]);
    for (int axis = 0; axis < 3; axis++) {
        gravity_vector[axis] /= gravity_mag;
    }
};

// this is just the magnitude of the vector projection of acceleration onto the gravity_vector
// acc should be filtered when we run this equation
Madgwick::vertical_acceleration_from_acc(float acc[]) {
    // v*w/||w||^2 * w gives us the projection of v onto w
    float dot_product = gravity_vector[X] * acc[X] + gravity_vector[Y] * acc[Y] + gravity_vector[Z] * acc[Z];

    // below line not needed, should always return 1 as its a normalized vector :) nice.
    // float gravity_mag_squared = gravity_vector[X] * gravity_vector[X] + gravity_vector[Y] * gravity_vector[Y] + gravity_vector[Z] * gravity_vector[Z];
    float gravity_mag_squared = 1.0f;

    float projection[3] = {
        (dot_product / gravity_mag_squared) * gravity_vector[X],
        (dot_product / gravity_mag_squared) * gravity_vector[Y],
        (dot_product / gravity_mag_squared) * gravity_vector[Z]
        };
    float projection_magnitude = sqrt(projection[X] * projection[X] + projection[Y] * projection[Y] + projection[Z] * projection[Z]);

    // give the magnitude a negative value if its pointing opposite the direction of gravity, aka further than 90 degrees
    if (dot_product < 0.0) {
        projection_magnitude = -projection_magnitude;
    }

    return projection_magnitude;
};

Madgwick::initialize(float b) {
    beta = b;
    gravity_vector[X] = 0.0;
    gravity_vector[Y] = 0.0;
    gravity_vector[Z] = 1.0;
}
