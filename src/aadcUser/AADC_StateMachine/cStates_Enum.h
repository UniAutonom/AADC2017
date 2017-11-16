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
    runState_running,
    runState_parking,
    runState_overtaking_check,
    runState_overtaking_run
};


#endif
