#define ROLL 0
#define PITCH 1
#define YAW 2

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

#define ACC_FUSION_BOUND 0.05


class QuickSilver {
public:
    void update_estimate(float acc[], float gyro[], float dt, bool fuseAcc);
    float vertical_acceleration_from_acc(float acc[]);
    void initialize(float b);
    float* getGravityVector();
private:
    float gravity_vector[3];
    float beta; // filter gain used to fuse acc into the estimate should be a fairly small value
};