// implemented following the QuickSilver project and Kevin Plaizier knowledge ;)
// QuickSilver https://github.com/BossHobby/QUICKSILVER/blob/develop/src/flight/imu.c
#include "QuickSilver.hh"
#include <math.h>
#include <Arduino.h>
// acc should be in units of g's and gyro in rad/s
// acc should be filtered at this point
void QuickSilver::update_estimate(float acc[], float gyro[], float dt, bool fuseAcc) { // TODO potentially add a vay to disable acc entirely during some parts of flight
    float pitch_delta = gyro[PITCH] * dt * DEG_TO_RAD;
    float roll_delta = gyro[ROLL] * dt * DEG_TO_RAD;
    float yaw_delta = gyro[YAW] * dt * DEG_TO_RAD;

    float gravity_vector_temp[3]; 
    for (int axis = 0; axis < 3; axis++) {
        gravity_vector_temp[axis] = gravity_vector[axis];
    }

    gravity_vector[VEC_Z] += pitch_delta * gravity_vector_temp[VEC_X];
    gravity_vector[VEC_X] -= pitch_delta * gravity_vector_temp[VEC_Z];

    gravity_vector[VEC_Y] += roll_delta * gravity_vector_temp[VEC_Z];
    gravity_vector[VEC_Z] -= roll_delta * gravity_vector_temp[VEC_Y];

    gravity_vector[VEC_X] += yaw_delta * gravity_vector_temp[VEC_Y];
    gravity_vector[VEC_Y] -= yaw_delta * gravity_vector_temp[VEC_X];


//   const float cosx = cosf(roll_delta);
//   const float sinx = sinf(roll_delta);
//   const float cosy = cosf(pitch_delta);
//   const float siny = sinf(-pitch_delta);
//   const float cosz = cosf(-yaw_delta);
//   const float sinz = sinf(yaw_delta);

//   const float coszcosx = cosz * cosx;
//   const float coszcosy = cosz * cosy;
//   const float sinzcosx = sinz * cosx;
//   const float coszsinx = sinx * cosz;
//   const float sinzsinx = sinx * sinz;

//   float mat[3][3];
//   mat[0][0] = coszcosy;
//   mat[0][1] = -cosy * sinz;
//   mat[0][2] = siny;
//   mat[1][0] = sinzcosx + (coszsinx * siny);
//   mat[1][1] = coszcosx - (sinzsinx * siny);
//   mat[1][2] = -sinx * cosy;
//   mat[2][0] = (sinzsinx) - (coszcosx * siny);
//   mat[2][1] = (coszsinx) + (sinzcosx * siny);
//   mat[2][2] = cosy * cosx;

//   gravity_vector[0] = gravity_vector_temp[0] * mat[0][0] + gravity_vector_temp[1] * mat[1][0] + gravity_vector_temp[2] * mat[2][0];
//   gravity_vector[1] = gravity_vector_temp[0] * mat[0][1] + gravity_vector_temp[1] * mat[1][1] + gravity_vector_temp[2] * mat[2][1];
//   gravity_vector[2] = gravity_vector_temp[0] * mat[0][2] + gravity_vector_temp[1] * mat[1][2] + gravity_vector_temp[2] * mat[2][2];

    if(fuseAcc){
        // not doing sqrt to save a little cpu
        float acc_mag_squared = acc[VEC_X] * acc[VEC_X] + acc[VEC_Y] * acc[VEC_Y] + acc[VEC_Z] * acc[VEC_Z];
        // acc_mag_squared = 0; //disable acc fusing
        if (acc_mag_squared < (1 + ACC_FUSION_BOUND) && acc_mag_squared > (1 - ACC_FUSION_BOUND)) { // todo test to see if this window is too small
            for (int axis = 0; axis < 3; axis++) {
                // slowly fuse the estimate towards the acc reading
                gravity_vector[axis] += beta * (acc[axis] - gravity_vector[axis]);
            }
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

    return projection_magnitude - 1.0;
};

void QuickSilver::initialize(float b) {
    beta = b;
    gravity_vector[VEC_X] = 0.0;
    gravity_vector[VEC_Y] = 0.0;
    gravity_vector[VEC_Z] = -1.0;
}

float* QuickSilver::getGravityVector(){
    return gravity_vector;
}
