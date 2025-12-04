#include "RobotStateMachine.h"
#include "config.h"
#include <Arduino.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

RobotStateMachine::RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor)
{
    _leftMotor = left;
    _rightMotor = right;
    _lineSensor = sensor;
    
    _currentState = STATE_IDLE;
    _currentMode = MODE_LINE_FOLLOW;
    
    _kp = KP;
    _ki = KI;
    _kd = KD;
    _lastError = 0;
    _integral = 0;
    
    _baseSpeed = BASE_SPEED;
    _maxSpeed = MAX_SPEED;
    
    _turnStartTime = 0;
    _junctionForwardTime = 0;
}

// ============================================================================
// STATE MACHINE UPDATE
// ============================================================================

void RobotStateMachine::update()
{
    _lineSensor->read();
    
    switch (_currentState) {
        case STATE_IDLE:
            handleIdleState();
            break;
        case STATE_LINE_FOLLOW:
            handleLineFollowState();
            break;
        case STATE_JUNCTION_FORWARD:
            handleJunctionForwardState();
            break;
        case STATE_TURN_LEFT:
            handleTurnLeftState();
            break;
        case STATE_TURN_RIGHT:
            handleTurnRightState();
            break;
        case STATE_TURN_AROUND:
            handleTurnAroundState();
            break;
        case STATE_OBSTACLE_AVOID:
            handleObstacleAvoidState();
            break;
        case STATE_MAZE_SOLVE:
            handleMazeSolveState();
            break;
        case STATE_LOST:
            handleLostState();
            break;
    }
}

// ============================================================================
// STATE HANDLERS
// ============================================================================

void RobotStateMachine::handleIdleState()
{
    stop();
}

void RobotStateMachine::handleLineFollowState()
{
    JunctionType junction = _lineSensor->detectJunction();
    
    if (junction != JUNCTION_NONE && junction != JUNCTION_LOST) {
        if (_currentMode == MODE_MAZE_SOLVE) {
            _currentState = STATE_JUNCTION_FORWARD;
            _junctionForwardTime = millis();
            return;
        }
    }
    
    if (junction == JUNCTION_LOST) {
        _currentState = STATE_LOST;
        return;
    }
    
    followLine();
}

void RobotStateMachine::handleJunctionForwardState()
{
    _leftMotor->setSpeed(_baseSpeed);
    _rightMotor->setSpeed(_baseSpeed);
    
    if (millis() - _junctionForwardTime > JUNCTION_FWD_MS) {
        JunctionType junction = _lineSensor->detectJunction();
        
        if (junction == JUNCTION_LEFT || junction == JUNCTION_T) {
            _currentState = STATE_TURN_LEFT;
            _turnStartTime = millis();
        }
        else if (junction == JUNCTION_RIGHT) {
            _currentState = STATE_TURN_RIGHT;
            _turnStartTime = millis();
        }
        else if (junction == JUNCTION_CROSS) {
            _currentState = STATE_LINE_FOLLOW;
        }
        else {
            _currentState = STATE_LINE_FOLLOW;
        }
    }
}

void RobotStateMachine::handleTurnLeftState()
{
    _leftMotor->setSpeed(0);
    _rightMotor->setSpeed(TURN_SPEED);
    
    if (millis() - _turnStartTime > TURN_90_TIME_MS) {
        _currentState = STATE_LINE_FOLLOW;
    }
}

void RobotStateMachine::handleTurnRightState()
{
    _leftMotor->setSpeed(TURN_SPEED);
    _rightMotor->setSpeed(0);
    
    if (millis() - _turnStartTime > TURN_90_TIME_MS) {
        _currentState = STATE_LINE_FOLLOW;
    }
}

void RobotStateMachine::handleTurnAroundState()
{
    _leftMotor->setSpeed(TURN_SPEED);
    _rightMotor->setSpeed(-TURN_SPEED);
    
    if (millis() - _turnStartTime > TURN_180_TIME_MS) {
        _currentState = STATE_LINE_FOLLOW;
    }
}

void RobotStateMachine::handleObstacleAvoidState()
{
    stop();
    delay(1000);
    _currentState = STATE_LINE_FOLLOW;
}

void RobotStateMachine::handleMazeSolveState()
{
    _currentState = STATE_LINE_FOLLOW;
}

void RobotStateMachine::handleLostState()
{
    stop();
    delay(500);
    _currentState = STATE_TURN_RIGHT;
    _turnStartTime = millis();
}

// ============================================================================
// PID LINE FOLLOWING
// ============================================================================

void RobotStateMachine::followLine()
{
    int position = _lineSensor->getPosition();
    int error = position;
    
    _integral += error;
    _integral = constrain(_integral, -10000, 10000);
    
    int derivative = error - _lastError;
    _lastError = error;
    
    float correction = _kp * error + _ki * _integral + _kd * derivative;
    
    int leftSpeed = _baseSpeed - correction;
    int rightSpeed = _baseSpeed + correction;
    
    leftSpeed = constrain(leftSpeed, -_maxSpeed, _maxSpeed);
    rightSpeed = constrain(rightSpeed, -_maxSpeed, _maxSpeed);
    
    _leftMotor->setSpeed(leftSpeed);
    _rightMotor->setSpeed(rightSpeed);
}

// ============================================================================
// CONTROL METHODS
// ============================================================================

void RobotStateMachine::start()
{
    _currentState = STATE_LINE_FOLLOW;
}

void RobotStateMachine::stop()
{
    _leftMotor->stop();
    _rightMotor->stop();
    _currentState = STATE_IDLE;
}

void RobotStateMachine::setMode(RobotMode mode)
{
    _currentMode = mode;
    _currentState = STATE_IDLE;
}

void RobotStateMachine::setSpeed(int speed)
{
    _baseSpeed = constrain(speed, 0, 255);
}

void RobotStateMachine::setPID(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

RobotState RobotStateMachine::getState()
{
    return _currentState;
}

RobotMode RobotStateMachine::getMode()
{
    return _currentMode;
}

// ============================================================================
// STATUS METHODS
// ============================================================================

void RobotStateMachine::printStatus()
{
    Serial.print("State: ");
    switch(_currentState) {
        case STATE_IDLE: Serial.print("IDLE"); break;
        case STATE_LINE_FOLLOW: Serial.print("LINE_FOLLOW"); break;
        case STATE_JUNCTION_FORWARD: Serial.print("JUNCTION_FWD"); break;
        case STATE_TURN_LEFT: Serial.print("TURN_LEFT"); break;
        case STATE_TURN_RIGHT: Serial.print("TURN_RIGHT"); break;
        case STATE_TURN_AROUND: Serial.print("TURN_AROUND"); break;
        case STATE_OBSTACLE_AVOID: Serial.print("OBSTACLE_AVOID"); break;
        case STATE_MAZE_SOLVE: Serial.print("MAZE_SOLVE"); break;
        case STATE_LOST: Serial.print("LOST"); break;
    }
    
    Serial.print(" | Mode: ");
    switch(_currentMode) {
        case MODE_LINE_FOLLOW: Serial.print("LINE_FOLLOW"); break;
        case MODE_OBSTACLE_AVOID: Serial.print("OBSTACLE_AVOID"); break;
        case MODE_MAZE_SOLVE: Serial.print("MAZE_SOLVE"); break;
        case MODE_NAVIGATE: Serial.print("NAVIGATE"); break;
    }
    
    Serial.print(" | Speed: L=");
    Serial.print(_leftMotor->getSpeed());
    Serial.print(" R=");
    Serial.println(_rightMotor->getSpeed());
}