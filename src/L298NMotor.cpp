#include "L298NMotor.h"
#include <Arduino.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================


L298NMotor::L298NMotor(int in1, int in2, int en)
{
    _in1 = in1;
    _in2 = in2;
    _en = en;
    _currentSpeed = 0;
    
    // Configure pins
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_en, OUTPUT);
    
    // Initialize stopped
    stop();
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void L298NMotor::setSpeed(int speed)
{
    // Clamp speed to valid range
    speed = constrain(speed, -255, 255);
    _currentSpeed = speed;
    
    if (speed == 0) {
        // Stop motor (brake)
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
        analogWrite(_en, 0);
    }
    else if (speed > 0) {
        // Forward direction
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
        analogWrite(_en, speed);
    }
    else {
        // Reverse direction
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
        analogWrite(_en, -speed);  // Convert negative to positive PWM
    }
}

void L298NMotor::forward(int speed)
{
    speed = constrain(speed, 0, 255);
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    analogWrite(_en, speed);
    _currentSpeed = speed;
}

void L298NMotor::reverse(int speed)
{
    speed = constrain(speed, 0, 255);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    analogWrite(_en, speed);
    _currentSpeed = -speed;
}

void L298NMotor::stop()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    analogWrite(_en, 0);
    _currentSpeed = 0;
}

void L298NMotor::brake()
{
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    analogWrite(_en, 255);
    _currentSpeed = 0;
}

int L298NMotor::getSpeed()
{
    return _currentSpeed;
}