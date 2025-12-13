#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include "L298NMotor.h"
#include "LineSensor.h"
#include "Odometry.h"

enum RobotState {
    STATE_IDLE,
    STATE_LINE_FOLLOW,
    STATE_SMALL_FORWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_TURN_AROUND,
    STATE_FINISHED
};

enum RobotMode {
    MODE_LINE_FOLLOW,
    MODE_MAZE_SOLVE
};

class RobotStateMachine {
public:
    RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor, Odometry* odom);
    
    void update();
    void start();
    void stop();
    
    void setMode(RobotMode mode);
    void setSpeed(int speed);
    void setPID(float kp, float ki, float kd);
    
    RobotState getState() { return _state; }
    RobotMode getMode() { return _currentMode; }
    
    void printStatus();

private:
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    LineSensor* _lineSensor;
    Odometry* _odometry;
    
    RobotState _state;
    RobotMode _currentMode;
    
    unsigned long _stateStartTime;
    JunctionType _storedJunction;
    
    float _kp, _ki, _kd;
    int _lastError;
    int _integral;
    int _baseSpeed;
    int _maxSpeed;
    
    float _smallForwardDist;
    float _turn90Angle;
    float _turn180Angle;
    
    void changeState(RobotState newState);
    void handleLineFollow();
    void handleSmallForward();
    void handleTurns();
    void calculatePIDSpeeds(int& leftSpeed, int& rightSpeed);
};

#endif