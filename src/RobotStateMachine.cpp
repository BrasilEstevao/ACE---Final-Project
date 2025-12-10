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
    
    _state = STATE_IDLE;
    _newState = STATE_IDLE;
    _currentMode = MODE_LINE_FOLLOW;
    
    
    _tes = millis();
    _tis = 0;
    
    _kp = KP;
    _ki = KI;
    _kd = KD;
    _lastError = 0;
    _integral = 0;
    
    _baseSpeed = BASE_SPEED;
    _maxSpeed = MAX_SPEED;
    
    _storedJunction = JUNCTION_NONE;
    
    
    _leftSpeedNew = 0;
    _rightSpeedNew = 0;
}

// ============================================================================
// STATE MACHINE UPDATE - MAIN LOOP
// ============================================================================

void RobotStateMachine::update()
{

    updateTIS();
    

    _lineSensor->read();
    
    updateStateTransitions();
       
    updateOutputs();
    
    applyOutputs();
}

// ============================================================================
// TIMING MANAGEMENT (TIS/TES)
// ============================================================================

void RobotStateMachine::updateTIS()
{
    _tis = millis() - _tes;
}

void RobotStateMachine::changeState(RobotState newState)
{
    if (_state != newState) {
        _state = newState;
        _tes = millis();  
        _tis = 0;       
    }
}

unsigned long RobotStateMachine::getTIS()
{
    return _tis;
}

// ============================================================================
// STATE TRANSITIONS
// ============================================================================

void RobotStateMachine::updateStateTransitions()
{
    JunctionType junction = _lineSensor->detectJunction();
    
    // Store current state for comparison
    _newState = _state;
    
    switch (_state) 
    {
        case STATE_IDLE:
            // No automatic transitions - controlled by start() command
            break;
            
        case STATE_LINE_FOLLOW:
            // Check for junction (only in MAZE mode)
            if (junction != JUNCTION_NONE && junction != JUNCTION_LOST) {
                if (_currentMode == MODE_MAZE_SOLVE) 
                {
                    switch (junction)
                    {
                        case JUNCTION_LEFT:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_LEFT;
                            break;
                        case JUNCTION_RIGHT:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_RIGHT;
                            break;
                        case JUNCTION_T:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_T;
                            break;
                        case JUNCTION_LOST:
                            _newState = STATE_TURN_AROUND;
                            break;
                        default:
                            break;
                    }
                }
            }
            // Check for lost line
            if (junction == JUNCTION_LOST) {
                _newState = STATE_TURN_AROUND;
            }
            break;
            
        case STATE_SMALL_FORWARD:
            // Check if enough time has passed
            if (_tis > SMALL_FWD_MS) {
                junction = _lineSensor->detectJunction();

                Serial.print("Stored Junction = ");
                switch (_storedJunction) {
                    case JUNCTION_LEFT:  Serial.println("LEFT"); break;
                    case JUNCTION_RIGHT: Serial.println("RIGHT"); break;
                    case JUNCTION_T:     Serial.println("T"); break;
                    case JUNCTION_NONE:  Serial.println("NONE"); break;
                    case JUNCTION_LOST:  Serial.println("LOST"); break;
                    default:             Serial.println("UNKNOWN"); break;
                }

                switch (junction)
                {
                    case JUNCTION_T:
                        _newState = STATE_FINISHED;
                        break;

                    case JUNCTION_NONE:
                        switch (_storedJunction)
                        {
                            case JUNCTION_LEFT:
                            case JUNCTION_T:
                                _newState = STATE_TURN_LEFT;
                                break;

                            case JUNCTION_RIGHT:
                                _newState = STATE_LINE_FOLLOW;
                                break;

                            default:
                                break;
                        }
                        break;

                    case JUNCTION_LOST:
                        switch (_storedJunction)
                        {
                            case JUNCTION_LEFT:
                            case JUNCTION_T:
                                _newState = STATE_TURN_LEFT;
                                break;

                            case JUNCTION_RIGHT:
                                _newState = STATE_TURN_RIGHT;
                                break;

                            default:
                                break;
                        }
                        break;

                    default:
                        break;
                }
            }
            break;
            
        case STATE_TURN_LEFT:
            // Check if turn time completed AND back on line
            if (_tis > LEFT_TURN_90_TIME_MS && junction == JUNCTION_NONE) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_TURN_RIGHT:
            // Check if turn time completed AND back on line
            if (_tis > RIGHT_TURN_90_TIME_MS && junction == JUNCTION_NONE) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_TURN_AROUND:
            // Check if U-turn time completed AND back on line
            if (_tis > TURN_180_TIME_MS && junction == JUNCTION_NONE) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_OBSTACLE_AVOID:
            // Simple timeout transition
            if (_tis > 1000) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_MAZE_SOLVE:
            // Redirect to line follow
            _newState = STATE_LINE_FOLLOW;
            break;
            
        case STATE_LOST:
            // After stopping briefly, start searching
            if (_tis > 500) {
                _newState = STATE_TURN_RIGHT;
            }
            break;
            
        case STATE_FINISHED:
            // Stay in finished state
            break;
    }
    
    // Apply state change if needed
    changeState(_newState);
}

// ============================================================================
// UPDATE OUTPUTS
// ============================================================================

void RobotStateMachine::updateOutputs()
{
    switch (_state)
    {
        case STATE_IDLE:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LINE_FOLLOW:
            calculatePIDSpeeds();
            break;
            
        case STATE_SMALL_FORWARD:
            _leftSpeedNew = _baseSpeed;
            _rightSpeedNew = _baseSpeed;
            break;
            
        case STATE_TURN_LEFT:
            _leftSpeedNew = -TURN_SPEED;
            _rightSpeedNew = TURN_SPEED;
            break;
            
        case STATE_TURN_RIGHT:
            _leftSpeedNew = TURN_SPEED;
            _rightSpeedNew = -TURN_SPEED;
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
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LOST:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_FINISHED:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
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
// PID CALCULATION
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

RobotState RobotStateMachine::getState()
{
    return _state;
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
    switch(_state) {
        case STATE_IDLE: Serial.print("IDLE"); break;
        case STATE_LINE_FOLLOW: Serial.print("LINE_FOLLOW"); break;
        case STATE_SMALL_FORWARD: Serial.print("SMALL_FWD"); break;
        case STATE_TURN_LEFT: Serial.print("TURN_LEFT"); break;
        case STATE_TURN_RIGHT: Serial.print("TURN_RIGHT"); break;
        case STATE_TURN_AROUND: Serial.print("TURN_AROUND"); break;
        case STATE_OBSTACLE_AVOID: Serial.print("OBSTACLE_AVOID"); break;
        case STATE_MAZE_SOLVE: Serial.print("MAZE_SOLVE"); break;
        case STATE_LOST: Serial.print("LOST"); break;
        case STATE_FINISHED: Serial.print("FINISHED"); break;
    }
    
    Serial.print(" | TIS: ");
    Serial.print(_tis);
    Serial.print("ms");
    
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