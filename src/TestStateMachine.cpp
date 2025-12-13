#include "TestStateMachine.h"
#include "config.h"
#include <Arduino.h>

TestStateMachine::TestStateMachine(L298NMotor* left, L298NMotor* right, Odometry* odom)
{
    _leftMotor = left;
    _rightMotor = right;
    _odometry = odom;
    
    _state = TEST_IDLE;
    _stateStartTime = 0;
}

void TestStateMachine::update()
{
    if (_state == TEST_IDLE || _state == TEST_COMPLETE) {
        _leftMotor->setSpeed(0);
        _rightMotor->setSpeed(0);
        return;
    }
    
    executeTest();
}

void TestStateMachine::changeState(TestState newState)
{
    if (_state != newState) {
        _state = newState;
        _stateStartTime = millis();
        if (_odometry) _odometry->resetRelative();
        
        Serial.print("Test: ");
        switch(newState) {
            case TEST_TURN_LEFT_90: Serial.println("Turn Left 90°"); break;
            case TEST_TURN_RIGHT_90: Serial.println("Turn Right 90°"); break;
            case TEST_U_TURN: Serial.println("U-Turn 180°"); break;
            case TEST_FORWARD: Serial.println("Forward 10cm"); break;
            case TEST_COMPLETE: Serial.println("COMPLETE!"); break;
            default: break;
        }
    }
}

void TestStateMachine::executeTest()
{
    const int turnSpeed = 120;
    const int fwdSpeed = 100;
    const float turn90 = PI / 2.0;
    const float turn180 = PI;
    const float fwdDist = 0.10; 
    
    bool reachedTarget = false;
    unsigned long timeout = 5000; 
    
    switch (_state) {
        case TEST_TURN_LEFT_90:
            _leftMotor->setSpeed(-turnSpeed);
            _rightMotor->setSpeed(turnSpeed);
            if (_odometry && _odometry->reachedRotation(turn90)) {
                reachedTarget = true;
            }
            break;
            
        case TEST_TURN_RIGHT_90:
            _leftMotor->setSpeed(turnSpeed);
            _rightMotor->setSpeed(-turnSpeed);
            if (_odometry && _odometry->reachedRotation(turn90)) {
                reachedTarget = true;
            }
            break;
            
        case TEST_U_TURN:
            _leftMotor->setSpeed(turnSpeed);
            _rightMotor->setSpeed(-turnSpeed);
            if (_odometry && _odometry->reachedRotation(turn180)) {
                reachedTarget = true;
            }
            break;
            
        case TEST_FORWARD:
            _leftMotor->setSpeed(fwdSpeed);
            _rightMotor->setSpeed(fwdSpeed);
            if (_odometry && _odometry->reachedDistance(fwdDist)) {
                reachedTarget = true;
            }
            break;
            
        default:
            break;
    }
    
    // Check if reached target or timeout
    if (reachedTarget || (millis() - _stateStartTime > timeout)) {
        if (millis() - _stateStartTime > timeout) {
            Serial.println("WARNING: Timeout reached");
        }
        
        _leftMotor->setSpeed(0);
        _rightMotor->setSpeed(0);
        delay(500); // Pause between tests
        
        // Advance to next test
        switch (_state) {
            case TEST_TURN_LEFT_90:
                changeState(TEST_TURN_RIGHT_90);
                break;
            case TEST_TURN_RIGHT_90:
                changeState(TEST_TURN_LEFT_90);
                break;
            case TEST_U_TURN:
                changeState(TEST_FORWARD);
                break;
            case TEST_FORWARD:
                changeState(TEST_COMPLETE);
                break;
            default:
                break;
        }
    }
}

void TestStateMachine::start()
{
    if (_odometry) _odometry->resetGlobal();
    changeState(TEST_TURN_LEFT_90);
}

void TestStateMachine::stop()
{
    changeState(TEST_IDLE);
    _leftMotor->setSpeed(0);
    _rightMotor->setSpeed(0);
}

void TestStateMachine::reset()
{
    stop();
    if (_odometry) _odometry->resetGlobal();
}

void TestStateMachine::printStatus()
{
    Serial.print("Test State: ");
    switch(_state) {
        case TEST_IDLE: Serial.print("IDLE"); break;
        case TEST_TURN_LEFT_90: Serial.print("LEFT_90"); break;
        case TEST_TURN_RIGHT_90: Serial.print("RIGHT_90"); break;
        case TEST_U_TURN: Serial.print("U_TURN"); break;
        case TEST_FORWARD: Serial.print("FORWARD"); break;
        case TEST_COMPLETE: Serial.print("COMPLETE"); break;
    }
    
    if (_odometry) {
        Serial.print(" | Dist: ");
        Serial.print(_odometry->getRelativeDistance() * 100, 1);
        Serial.print("cm Rot: ");
        Serial.print(_odometry->getRelativeRotation() * 180 / PI, 1);
        Serial.print("°");
    }
    
    Serial.println();
}