/******************************************************************************
 * StateMachine.hh
 * Contains a class to track and update class for the determineState() portion
 * of the RocketRTOS code
******************************************************************************/
#ifndef STATE_MACHINE_HH
#define STATE_MACHINE_HH

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20
#define MIN_LOOPS_IN_STATE 3
#define MIN_POINTS_TO_LEAVE 3

#include "RocketRTOS.hh"

class StateMachine {
public:
    StateMachine();
    void updateState(float h, float v, float a);
private:
    //rocketState
    unsigned int stayCounter;
    unsigned int leaveCounter;
};



#endif //STATE_MACHINE_HH