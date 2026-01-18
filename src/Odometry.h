#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "config.h"

/**
 * @class Odometry
 * @brief Tracks robot position and orientation using wheel encoders
 * 
 * Calculates:
 * - Linear velocity (v)
 * - Angular velocity (w)
 * - Position (x, y)
 * - Orientation (theta)
 */
class Odometry {
public:
    /**
     * @brief Constructor
     * @param wheelDist Distance between wheels (meters)
     * @param wheelRadius Wheel radius (meters)
     * @param encoderCPR Encoder counts per revolution
     */
    Odometry(float wheelDist = 0.105, float wheelRadius = 0.0345, int encoderCPR = 1920);
    
    /**
     * @brief Initialize encoder pins and interrupts
     */
    void begin();
    
    /**
     * @brief Update odometry calculations
     * @param dt Time delta in seconds
     */
    void update(float dt);
    
    /**
     * @brief Reset position and orientation to zero
     */
    void reset();
    
    /**
     * @brief Reset relative displacement counters
     */
    void resetRelative();
    
    /**
     * @brief Get current X position (meters)
     */
    float getX() { return _x; }
    
    /**
     * @brief Get current Y position (meters)
     */
    float getY() { return _y; }
    
    /**
     * @brief Get current orientation (radians)
     */
    float getTheta() { return _theta; }
    
    /**
     * @brief Get current linear velocity (m/s)
     */
    float getVelocity() { return _v; }
    
    /**
     * @brief Get current angular velocity (rad/s)
     */
    float getAngularVelocity() { return _w; }
    
    /**
     * @brief Get relative distance traveled since last reset (meters)
     */
    float getRelativeDistance() { return _relDistance; }
    
    /**
     * @brief Get relative angle turned since last reset (radians)
     */
    float getRelativeAngle() { return _relTheta; }
    
    /**
     * @brief Get left wheel velocity (m/s)
     */
    float getLeftVelocity() { return _v1; }
    
    /**
     * @brief Get right wheel velocity (m/s)
     */
    float getRightVelocity() { return _v2; }
    
    /**
     * @brief Get left encoder count (since last update)
     */
    int getLeftEncoder() { return _enc1; }
    
    /**
     * @brief Get right encoder count (since last update)
     */
    int getRightEncoder() { return _enc2; }
    
    /**
     * @brief Print odometry values to Serial
     */
    void printValues();
    
private:
    // Robot geometry
    float _wheelDist;      // Distance between wheels (m)
    float _wheelRadius;    // Wheel radius (m)
    int _encoderCPR;       // Counts per revolution
    
    // Encoder readings
    volatile int _enc1;    // Left encoder delta
    volatile int _enc2;    // Right encoder delta
    
    // Wheel velocities
    float _v1;             // Left wheel velocity (m/s)
    float _v2;             // Right wheel velocity (m/s)
    
    // Robot state
    float _v;              // Linear velocity (m/s)
    float _w;              // Angular velocity (rad/s)
    float _x;              // X position (m)
    float _y;              // Y position (m)
    float _theta;          // Orientation (rad)
    
    // Relative displacement (for maneuvers)
    float _relDistance;    // Distance traveled (m)
    float _relTheta;       // Angle turned (rad)
    
    // Encoder interrupt handlers (must be static for ISR)
    static void leftEncoderISR();
    static void rightEncoderISR();
    
    // Static instance pointer for ISR access
    static Odometry* _instance;
    
    // Volatile encoder counters (accessed by ISR)
    static volatile long _leftEncoderCount;
    static volatile long _rightEncoderCount;
    static volatile uint8_t _leftEncoderState;
    static volatile uint8_t _rightEncoderState;
    
    // Encoder state machine lookup table
    static const int8_t _encoderTable[16];
};

#endif // ODOMETRY_H