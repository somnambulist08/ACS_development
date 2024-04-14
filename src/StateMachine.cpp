#include "StateMachine.hh"

StateMachine::StateMachine() : stayCounter(0), leaveCounter(0) {
    rocketState = ROCKET_PRE;
}

void StateMachine::updateState(float h, float v, float a) {
    if(stayCounter < MIN_LOOPS_IN_STATE){
        stayCounter++;
    } else {
        bool stay;
        bool proceed = false;
        RocketState_t nextState;
        switch(rocketState){
            case(ROCKET_PRE):
                stay = (a < LAUNCH_THRESHOLD_A_M_S2 || h < LAUNCH_THRESHOLD_H_M );
                nextState = ROCKET_LAUNCH;
                break;
            case(ROCKET_LAUNCH):
                stay = (a > -10); //-10 is roughly gravity, so will always be reached. Technically, this value should be whatever the force of drag is at that point
                nextState = ROCKET_FREEFALL;
                break;
            case(ROCKET_FREEFALL):
                stay = (v > 0);
                nextState = ROCKET_RECOVERY;
                break;
            case(ROCKET_RECOVERY):
                stay = true;
                nextState = ROCKET_RECOVERY;
                break;
            default:
                //How'd you even get here?
                stay = false;
                nextState = ROCKET_PRE;
                break;
        }
        if(stay){
            leaveCounter = 0;
        } else {
            leaveCounter++;
            if(leaveCounter > MIN_POINTS_TO_LEAVE){
                proceed = true;
            }
        }
        if(proceed){
            rocketState = nextState;
            stayCounter = 0;
        }
    }
}