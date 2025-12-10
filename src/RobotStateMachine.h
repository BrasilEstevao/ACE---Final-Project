#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include "L298NMotor.h"
#include "LineSensor.h"

enum RobotState {
    STATE_IDLE,
    STATE_LINE_FOLLOW,
    STATE_SMALL_FORWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_TURN_AROUND,
    STATE_OBSTACLE_AVOID,
    STATE_MAZE_SOLVE,
    STATE_LOST,
    STATE_FINISHED
};

enum RobotMode {
    MODE_LINE_FOLLOW,
    MODE_OBSTACLE_AVOID,
    MODE_MAZE_SOLVE,
    MODE_NAVIGATE
};

class RobotStateMachine {
public:
    RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor);

    JunctionType _storedJunction;

    void update();
    void start();
    void stop();

    void setMode(RobotMode mode);
    void setSpeed(int speed);
    void setPID(float kp, float ki, float kd);

    RobotState getState();
    RobotMode getMode();
    unsigned long getTIS();

    void printStatus();

private:
    // Hardware
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    LineSensor* _lineSensor;

    // State info
    RobotState _state;
    RobotState _newState;
    RobotMode _currentMode;

    unsigned long _tes;
    unsigned long _tis;

    // PID
    float _kp, _ki, _kd;
    int _lastError;
    int _integral;

    // Speed
    int _baseSpeed;
    int _maxSpeed;

    // Outputs
    int _leftSpeedNew;
    int _rightSpeedNew;

    void updateTIS();
    void changeState(RobotState newState);
    void updateStateTransitions();
    void updateOutputs();
    void applyOutputs();
    void calculatePIDSpeeds();
};

#endif // ROBOT_STATE_MACHINE_H
