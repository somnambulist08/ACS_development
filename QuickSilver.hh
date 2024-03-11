#include <math.h>
#define ROLL 0
#define PITCH 1
#define YAW 2

#define X 0
#define Y 1
#define Z 2

class QuickSilver {
public:
    void update_estimate(float acc[], float gyro[], float dt);
    float vertical_acceleration_from_acc(float acc[]);
    void initialize(float b);
private:
    float gravity_vector[3]
    float beta; // filter gain used to fuse acc into the estimate should be a fairly small value
};