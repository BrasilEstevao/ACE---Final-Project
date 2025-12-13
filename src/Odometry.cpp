#include "Odometry.h"
#include <Arduino.h>

Odometry::Odometry(float wheelDist, float wheelRadius, int pulsesPerRev)
{
    _wheelDist = wheelDist;
    _wheelRadius = wheelRadius;
    _pulsesPerRev = pulsesPerRev;
    
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;
    _relDistance = 0.0;
    _relRotation = 0.0;
    _v = 0.0;
    _w = 0.0;
    _v1 = 0.0;
    _v2 = 0.0;
}

void Odometry::update(int enc1, int enc2, float dt)
{
    if (dt <= 0.001) return;
    
    // Angular velocities (rad/s)
    float w1 = (enc1 * TWO_PI) / ((float)_pulsesPerRev * dt);
    float w2 = (enc2 * TWO_PI) / ((float)_pulsesPerRev * dt);
    
    // Linear velocities (m/s)
    _v1 = w1 * _wheelRadius;
    _v2 = w2 * _wheelRadius;
    
    // Differential drive kinematics
    _v = (_v1 + _v2) / 2.0;
    _w = (_v2 - _v1) / _wheelDist; // Corrigido o sinal aqui para ser (V2 - V1) para giros padrões
    
    // Incremental displacements
    float ds = _v * dt;
    float dtheta = _w * dt;
    
    // Update global pose
    _x += ds * cos(_theta + dtheta / 2.0);
    _y += ds * sin(_theta + dtheta / 2.0);
    _theta += dtheta;
    
    // Normalize theta
    while (_theta > PI) _theta -= TWO_PI;
    while (_theta < -PI) _theta += TWO_PI;
    
    // Update relative
    _relDistance += abs(ds);
    _relRotation += dtheta;
}

void Odometry::resetRelative()
{
    _relDistance = 0.0;
    _relRotation = 0.0;
}

void Odometry::resetGlobal()
{
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;
    resetRelative();
}

bool Odometry::reachedDistance(float targetDistance)
{
    return _relDistance >= abs(targetDistance);
}

bool Odometry::reachedRotation(float targetAngle)
{
    return abs(_relRotation) >= abs(targetAngle);
}

