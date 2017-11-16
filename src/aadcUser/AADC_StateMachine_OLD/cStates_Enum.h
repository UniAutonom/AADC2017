#ifndef _STATES_ENUM_H
#define _STATES_ENUM_H


/*! enum that represents all valid states of the state machine. */
enum primaryStates{
    primaryState_emergencyBreak = 0,
    primaryState_run
};

enum runStates{
    runState_stop = 0,
    runState_ready,
    runState_follow,
    runState_turning_left,
    runState_turning_right,
    runState_parking,
    runState_overtaking_check,
    runState_overtaking_run
};


#endif
