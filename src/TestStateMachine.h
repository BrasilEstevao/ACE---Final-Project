#ifndef TEST_STATE_MACHINE_H
#define TEST_STATE_MACHINE_H

#include "L298NMotor.h"
#include "Odometry.h"

enum TestState {
    TEST_IDLE,
    TEST_TURN_LEFT_90,
    TEST_TURN_RIGHT_90,
    TEST_U_TURN,
    TEST_FORWARD,
    TEST_COMPLETE
};

class TestStateMachine {
public:
    TestStateMachine(L298NMotor* left, L298NMotor* right, Odometry* odom);
    
    void update();
    void start();
    void stop();
    void reset();
    
    TestState getState() { return _state; }
    bool isComplete() { return _state == TEST_COMPLETE; }
    
    void printStatus();

private:
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    Odometry* _odometry;
    
    TestState _state;
    unsigned long _stateStartTime;
    
    void changeState(TestState newState);
    void executeTest();
};

#endif