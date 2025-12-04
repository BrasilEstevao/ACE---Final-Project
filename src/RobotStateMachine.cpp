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
    
    // Initialize output variables
    _leftSpeedNew = 0;
    _rightSpeedNew = 0;
}

// ============================================================================
// STATE MACHINE UPDATE - MAIN LOOP
// ============================================================================

void RobotStateMachine::update()
{
    // Read sensors
    _lineSensor->read();
    
    // ========================================================================
    // STATE TRANSITIONS
    // ========================================================================
    updateStateTransitions();
    
    // ========================================================================
    // UPDATE OUTPUTS 
    // ========================================================================
    updateOutputs();
    
    // ========================================================================
    // APPLY OUTPUTS
    // ========================================================================
    applyOutputs();
}

// ============================================================================
// PART 1: STATE TRANSITIONS
// ============================================================================

void RobotStateMachine::updateStateTransitions()
{
    JunctionType junction = _lineSensor->detectJunction();
    
    switch (_currentState) 
    {
        case STATE_IDLE:
            // No automatic transitions - controlled by start() command
            break;
            
        case STATE_LINE_FOLLOW:
            // Check for junction (only in MAZE mode)
            if (junction != JUNCTION_NONE && junction != JUNCTION_LOST) {
                if (_currentMode == MODE_MAZE_SOLVE) {
                    _currentState = STATE_JUNCTION_FORWARD;
                    _junctionForwardTime = millis();
                }
            }
            
            // Check for lost line
            if (junction == JUNCTION_LOST) {
                _currentState = STATE_LOST;
            }
            break;
            
        case STATE_JUNCTION_FORWARD:
            // Wait for forward movement to complete
            if (millis() - _junctionForwardTime > JUNCTION_FWD_MS) {
                junction = _lineSensor->detectJunction();
                
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
            break;
            
        case STATE_TURN_LEFT:
            // Check if turn time completed
            if (millis() - _turnStartTime > LEFT_TURN_90_TIME_MS) {
                _currentState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_TURN_RIGHT:
            // Check if turn time completed
            if (millis() - _turnStartTime > RIGHT_TURN_90_TIME_MS) {
                _currentState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_TURN_AROUND:
            // Check if U-turn time completed
            if (millis() - _turnStartTime > TURN_180_TIME_MS) {
                _currentState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_OBSTACLE_AVOID:
            // Simple timeout transition
            if (millis() - _turnStartTime > 1000) {
                _currentState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_MAZE_SOLVE:
            // Redirect to line follow
            _currentState = STATE_LINE_FOLLOW;
            break;
            
        case STATE_LOST:
            // After stopping briefly, start searching
            if (millis() - _turnStartTime > 500) {
                _currentState = STATE_TURN_RIGHT;
                _turnStartTime = millis();
            }
            break;
    }
}

// ============================================================================
// UPDATE OUTPUTS
// ============================================================================

void RobotStateMachine::updateOutputs()
{
    switch (_currentState)
    {
        case STATE_IDLE:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LINE_FOLLOW:
            calculatePIDSpeeds();
            break;
            
        case STATE_JUNCTION_FORWARD:
            _leftSpeedNew = _baseSpeed;
            _rightSpeedNew = _baseSpeed;
            break;
            
        case STATE_TURN_LEFT:
            _leftSpeedNew = 0;
            _rightSpeedNew = TURN_SPEED;
            break;
            
        case STATE_TURN_RIGHT:
            _leftSpeedNew = TURN_SPEED;
            _rightSpeedNew = 0;
            break;
            
        case STATE_TURN_AROUND:
            _leftSpeedNew = TURN_SPEED;
            _rightSpeedNew = -TURN_SPEED;
            break;
            
        case STATE_OBSTACLE_AVOID:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_MAZE_SOLVE:
            // Should not reach here due to transition
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LOST:
            // Stop during lost state
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            // Set timer for search
            if (_turnStartTime == 0) {
                _turnStartTime = millis();
            }
            break;
    }
}

// ============================================================================
// APPLY OUTPUTS
// ============================================================================

void RobotStateMachine::applyOutputs()
{
    _leftMotor->setSpeed(_leftSpeedNew);
    _rightMotor->setSpeed(_rightSpeedNew);
}

// ============================================================================
// PID CALCULATION (Helper for LINE_FOLLOW state)
// ============================================================================

void RobotStateMachine::calculatePIDSpeeds()
{
    int position = _lineSensor->getPosition();
    int error = position;
    
    // Integral with anti-windup
    _integral += error;
    _integral = constrain(_integral, -10000, 10000);
    
    // Derivative
    int derivative = error - _lastError;
    _lastError = error;
    
    // PID formula
    float correction = _kp * error + _ki * _integral + _kd * derivative;
    
    // Apply correction to base speed
    int leftSpeed = _baseSpeed - correction;
    int rightSpeed = _baseSpeed + correction;
    
    // Constrain to valid range
    _leftSpeedNew = constrain(leftSpeed, -_maxSpeed, _maxSpeed);
    _rightSpeedNew = constrain(rightSpeed, -_maxSpeed, _maxSpeed);
}

// ============================================================================
// CONTROL METHODS
// ============================================================================

void RobotStateMachine::start()
{
    _currentState = STATE_LINE_FOLLOW;
    _lastError = 0;
    _integral = 0;
}

void RobotStateMachine::stop()
{
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
    
    Serial.print(" | PID: Kp=");
    Serial.print(_kp, 3);
    Serial.print(" Ki=");
    Serial.print(_ki, 3);
    Serial.print(" Kd=");
    Serial.println(_kd, 3);
}