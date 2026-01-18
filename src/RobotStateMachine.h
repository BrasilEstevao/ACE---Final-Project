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
    STATE_OBSTACLE_AVOID,
    STATE_MAZE_SOLVE,
    STATE_LOST,
    STATE_FINISHED,
    STATE_VELOCITY_CONTROL,
    STATE_GOTO_DISTANCE,
    STATE_GOTO_ANGLE
};

enum RobotMode {
    MODE_LINE_FOLLOW,
    MODE_OBSTACLE_AVOID,
    MODE_MAZE_SOLVE,
    MODE_NAVIGATE,
    MODE_VELOCITY_CONTROL
};

class RobotStateMachine {
public:
    RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor, Odometry* odom);

    JunctionType _storedJunction;
    
    float _targetV;
    float _targetW;
    
    float _targetDistance;
    float _targetAngle;

    void update();
    void start();
    void stop();

    void setMode(RobotMode mode);
    void setSpeed(int speed);
    void setPID(float kp, float ki, float kd);
    
    /**
     * @brief Configura ganhos PID para controle de velocidade
     */
    void setVelocityPID(float kp_v, float ki_v, float kd_v, 
                        float kp_w, float ki_w, float kd_w);
    
    /**
     * @brief Escolhe entre controle PID (fechado) ou direto (aberto)
     * @param usePID true = usa PID com feedback, false = controle direto
     */
    void setPIDControlMode(bool usePID);
    
    void setTurnAngles(float turn90, float turn180);
    void setVelocity(float v, float w);
    void gotoDistance(float distance);
    void gotoAngle(float angle);

    RobotState getState();
    RobotMode getMode();
    bool isPIDControlEnabled();
    unsigned long getTIS();

    void printStatus();

private:
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    LineSensor* _lineSensor;
    Odometry* _odometry;

    RobotState _state;
    RobotState _newState;
    RobotMode _currentMode;

    unsigned long _tes;
    unsigned long _tis;

    // PID para line following
    float _kp, _ki, _kd;
    int _lastError;
    int _integral;

    // PID para controle de velocidade
    float _pidV_kp, _pidV_ki, _pidV_kd;
    float _pidV_lastError;
    float _pidV_integral;
    
    float _pidW_kp, _pidW_ki, _pidW_kd;
    float _pidW_lastError;
    float _pidW_integral;
    
    bool _usePIDControl;  // true = PID, false = direto

    int _baseSpeed;
    int _maxSpeed;

    int _leftSpeedNew;
    int _rightSpeedNew;
    
    bool _useOdometry;
    float _turn90Angle;
    float _turn180Angle;
    float _smallForwardDist;

    void updateTIS();
    void changeState(RobotState newState);
    void updateStateTransitions();
    void updateOutputs();
    void applyOutputs();
    void calculatePIDSpeeds();
    void calculateVelocityControl();     // Controle direto (sem PID)
    void calculateVelocityControlPID();  // Controle com PID
};

#endif // ROBOT_STATE_MACHINE_H