#include "RobotStateMachine.h"
#include "config.h"
#include <Arduino.h>

RobotStateMachine::RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor, Odometry* odom)
{
    _leftMotor = left;
    _rightMotor = right;
    _lineSensor = sensor;
    _odometry = odom;
    
    _state = STATE_IDLE;
    _currentMode = MODE_LINE_FOLLOW;
    _stateStartTime = 0;
    _storedJunction = JUNCTION_NONE;
    
    _kp = KP;
    _ki = KI;
    _kd = KD;
    _lastError = 0;
    _integral = 0;
    
    _baseSpeed = BASE_SPEED;
    _maxSpeed = MAX_SPEED;
    
    _smallForwardDist = 0.10;
    _turn90Angle = PI / 2.0;
    _turn180Angle = PI;
}

void RobotStateMachine::update()
{
    _lineSensor->read();
    
    switch (_state) {
        case STATE_IDLE:
            _leftMotor->setSpeed(0);
            _rightMotor->setSpeed(0);
            break;
            
        case STATE_LINE_FOLLOW:
            handleLineFollow();
            break;
            
        case STATE_SMALL_FORWARD:
            handleSmallForward();
            break;
            
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
        case STATE_TURN_AROUND:
            handleTurns();
            break;
            
        case STATE_FINISHED:
            _leftMotor->setSpeed(0);
            _rightMotor->setSpeed(0);
            break;
    }
}

void RobotStateMachine::changeState(RobotState newState)
{
    if (_state != newState) {
        _state = newState;
        _stateStartTime = millis();
        if (_odometry) _odometry->resetRelative();
    }
}

void RobotStateMachine::handleLineFollow()
{
    int leftSpeed, rightSpeed;
    calculatePIDSpeeds(leftSpeed, rightSpeed);
    
    _leftMotor->setSpeed(leftSpeed);
    _rightMotor->setSpeed(rightSpeed);
    
    if (_currentMode == MODE_MAZE_SOLVE) {
        JunctionType junction = _lineSensor->detectJunction();
        
        if (junction == JUNCTION_LEFT || junction == JUNCTION_RIGHT || junction == JUNCTION_T) {
            _storedJunction = junction;
            changeState(STATE_SMALL_FORWARD);
        } else if (junction == JUNCTION_LOST) {
            changeState(STATE_TURN_AROUND);
        }
    }
}

void RobotStateMachine::handleSmallForward()
{
    _leftMotor->setSpeed(_baseSpeed);
    _rightMotor->setSpeed(_baseSpeed);
    
    // Timeout
    if (millis() - _stateStartTime > SMALL_FWD_MS * 3) {
        changeState(STATE_LINE_FOLLOW);
        return;
    }
    
    if (_odometry && _odometry->reachedDistance(_smallForwardDist)) {
        JunctionType junction = _lineSensor->detectJunction();
        
        if (junction == JUNCTION_T) {
            changeState(STATE_FINISHED);
        } else if (junction == JUNCTION_NONE) {
            if (_storedJunction == JUNCTION_LEFT || _storedJunction == JUNCTION_T) {
                changeState(STATE_TURN_LEFT);
            } else {
                changeState(STATE_LINE_FOLLOW);
            }
        } else if (junction == JUNCTION_LOST) {
            if (_storedJunction == JUNCTION_LEFT || _storedJunction == JUNCTION_T) {
                changeState(STATE_TURN_LEFT);
            } else if (_storedJunction == JUNCTION_RIGHT) {
                changeState(STATE_TURN_RIGHT);
            } else {
                changeState(STATE_TURN_AROUND);
            }
        }
    }
}

void RobotStateMachine::handleTurns()
{
    if (_state == STATE_TURN_LEFT) {
        _leftMotor->setSpeed(-TURN_SPEED);
        _rightMotor->setSpeed(TURN_SPEED);
    } else if (_state == STATE_TURN_RIGHT) {
        _leftMotor->setSpeed(TURN_SPEED);
        _rightMotor->setSpeed(-TURN_SPEED);
    } else if (_state == STATE_TURN_AROUND) {
        _leftMotor->setSpeed(TURN_SPEED);
        _rightMotor->setSpeed(-TURN_SPEED);
    }
    
    float targetAngle = (_state == STATE_TURN_AROUND) ? _turn180Angle : _turn90Angle;
    unsigned long timeout = (_state == STATE_TURN_AROUND) ? TURN_180_TIME_MS : LEFT_TURN_90_TIME_MS;
    
    if (_odometry && _odometry->reachedRotation(targetAngle)) {
        JunctionType junction = _lineSensor->detectJunction();
        if (junction == JUNCTION_NONE) {
            changeState(STATE_LINE_FOLLOW);
        }
    }
    
    if (millis() - _stateStartTime > timeout * 2) {
        changeState(STATE_LINE_FOLLOW);
    }
}

void RobotStateMachine::calculatePIDSpeeds(int& leftSpeed, int& rightSpeed)
{
    int position = _lineSensor->getPosition();
    int error = position;
    
    _integral += error;
    _integral = constrain(_integral, -10000, 10000);
    
    int derivative = error - _lastError;
    _lastError = error;
    
    float correction = _kp * error + _ki * _integral + _kd * derivative;
    
    leftSpeed = constrain(_baseSpeed - correction, -_maxSpeed, _maxSpeed);
    rightSpeed = constrain(_baseSpeed + correction, -_maxSpeed, _maxSpeed);
}

void RobotStateMachine::start()
{
    changeState(STATE_LINE_FOLLOW);
    _lastError = 0;
    _integral = 0;
}

void RobotStateMachine::stop()
{
    changeState(STATE_IDLE);
}

void RobotStateMachine::setMode(RobotMode mode)
{
    _currentMode = mode;
    changeState(STATE_IDLE);
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

void RobotStateMachine::printStatus()
{
    Serial.print("State: ");
    switch(_state) {
        case STATE_IDLE: Serial.print("IDLE"); break;
        case STATE_LINE_FOLLOW: Serial.print("FOLLOW"); break;
        case STATE_SMALL_FORWARD: Serial.print("SMALL_FWD"); break;
        case STATE_TURN_LEFT: Serial.print("LEFT"); break;
        case STATE_TURN_RIGHT: Serial.print("RIGHT"); break;
        case STATE_TURN_AROUND: Serial.print("AROUND"); break;
        case STATE_FINISHED: Serial.print("FINISHED"); break;
    }
    
    if (_odometry) {
        Serial.print(" | Dist: ");
        Serial.print(_odometry->getRelativeDistance() * 100, 1);
        Serial.print("cm Rot: ");
        Serial.print(_odometry->getRelativeRotation() * 180 / PI, 1);
        Serial.print("°");
    }
    
    Serial.print(" | L=");
    Serial.print(_leftMotor->getSpeed());
    Serial.print(" R=");
    Serial.println(_rightMotor->getSpeed());
}