#include "Odometry.h"
#include <math.h>

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

Odometry* Odometry::_instance = nullptr;
volatile long Odometry::_leftEncoderCount = 0;
volatile long Odometry::_rightEncoderCount = 0;
volatile uint8_t Odometry::_leftEncoderState = 0;
volatile uint8_t Odometry::_rightEncoderState = 0;

// Encoder state machine: [old_AB][new_AB] -> delta
const int8_t Odometry::_encoderTable[16] = {
    0, 1, -1, 0,   // 00 -> 00, 01, 10, 11
    -1, 0, 0, 1,   // 01 -> 00, 01, 10, 11
    1, 0, 0, -1,   // 10 -> 00, 01, 10, 11
    0, -1, 1, 0    // 11 -> 00, 01, 10, 11
};

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

void Odometry::leftEncoderISR() {
    if (!_instance) return;
    
    uint8_t newState = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
    uint8_t tableIndex = (_leftEncoderState << 2) | newState;
    _leftEncoderCount += _encoderTable[tableIndex];
    _leftEncoderState = newState;
}

void Odometry::rightEncoderISR() {
    if (!_instance) return;
    
    uint8_t newState = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
    uint8_t tableIndex = (_rightEncoderState << 2) | newState;
    _rightEncoderCount += _encoderTable[tableIndex];
    _rightEncoderState = newState;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Odometry::Odometry(float wheelDist, float wheelRadius, int encoderCPR)
{
    _wheelDist = wheelDist;
    _wheelRadius = wheelRadius;
    _encoderCPR = encoderCPR;
    
    _enc1 = 0;
    _enc2 = 0;
    _v1 = 0;
    _v2 = 0;
    _v = 0;
    _w = 0;
    
    _x = 0;
    _y = 0;
    _theta = 0;
    
    _relDistance = 0;
    _relTheta = 0;
    
    _instance = this;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void Odometry::begin()
{
    // Configure encoder pins
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);
    
    // Read initial state
    _leftEncoderState = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
    _rightEncoderState = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
    
    // Attach interrupts on both A and B channels for better resolution
    attachInterrupt(digitalPinToInterrupt(ENC1_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), rightEncoderISR, CHANGE);
    
    Serial.println("[Odometry] Encoder interrupts initialized");
}

// ============================================================================
// UPDATE CALCULATIONS (Based on Paulo Costa's implementation)
// ============================================================================

void Odometry::update(float dt)
{
    if (dt <= 0) return;
    
    // Read encoder deltas atomically
    noInterrupts();
    long enc1 = _leftEncoderCount;
    long enc2 = _rightEncoderCount;
    _leftEncoderCount = 0;
    _rightEncoderCount = 0;
    interrupts();
    
    _enc1 = enc1;
    _enc2 = enc2;
    
    // ========================================================================
    // ODOMETRY CALCULATION - Paulo Costa's Method
    // ========================================================================
    
    // Estimate wheel angular velocities (rad/s)
    // Formula: w = enc * 2Ï€ / (2 * CPR * dt)
    // The "2.0" factor is because we're counting both edges (quadrature)
    float w1e = (float)enc1 * TWO_PI / (2.0f * (float)_encoderCPR * dt);
    float w2e = (float)enc2 * TWO_PI / (2.0f * (float)_encoderCPR * dt);
    
    // Calculate wheel linear velocities (m/s)
    _v1 = w1e * _wheelRadius;
    _v2 = w2e * _wheelRadius;
    
    // Estimate robot velocities
    _v = (_v1 + _v2) / 2.0f;              // Linear velocity (m/s)
    _w = (_v1 - _v2) / _wheelDist;        // Angular velocity (rad/s)
    
    // Calculate displacement in this time step
    float ds = _v * dt;                   // Distance traveled (m)
    float dtheta = _w * dt;               // Angle turned (rad)
    
    // Update pose using midpoint integration method
    // This is more accurate than simple Euler integration
    _x += ds * cos(_theta + dtheta / 2.0f);
    _y += ds * sin(_theta + dtheta / 2.0f);
    _theta += dtheta;
    
    // Normalize theta to [-PI, PI]
    while (_theta > PI) _theta -= TWO_PI;
    while (_theta < -PI) _theta += TWO_PI;
    
    // Update relative displacement (for maneuvers)
    _relDistance += fabs(ds);  // Always positive distance
    _relTheta += dtheta;       // Can be positive or negative
}

// ============================================================================
// RESET FUNCTIONS
// ============================================================================

void Odometry::reset()
{
    noInterrupts();
    _leftEncoderCount = 0;
    _rightEncoderCount = 0;
    interrupts();
    
    _x = 0;
    _y = 0;
    _theta = 0;
    _v = 0;
    _w = 0;
    
    _relDistance = 0;
    _relTheta = 0;
}

void Odometry::resetRelative()
{
    _relDistance = 0;
    _relTheta = 0;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void Odometry::printValues()
{
    Serial.print("Odom: x=");
    Serial.print(_x, 3);
    Serial.print("m y=");
    Serial.print(_y, 3);
    Serial.print("m Î¸=");
    Serial.print(_theta * 180.0f / PI, 1);
    Serial.print("Â° | v=");
    Serial.print(_v, 3);
    Serial.print("m/s w=");
    Serial.print(_w, 2);
    Serial.print("rad/s | rel_d=");
    Serial.print(_relDistance, 3);
    Serial.print("m rel_Î¸=");
    Serial.print(_relTheta * 180.0f / PI, 1);
    Serial.println("Â°");
}