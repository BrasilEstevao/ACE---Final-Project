#ifndef L298N_MOTOR_H
#define L298N_MOTOR_H

#include <Arduino.h>

/**
 * @class L298NMotor
 * @brief Controls a single DC motor via L298N H-bridge
 * 
 * L298N pinout per motor:
 * - IN1/IN2: Direction control (digital)
 * - EN: Speed control (PWM)
 * 
 * Truth table:
 * IN1 | IN2 | EN  | Result
 * -----|-----|-----|--------
 * LOW | LOW | x   | Stop (coast)
 * HIGH| LOW | PWM | Forward
 * LOW |HIGH | PWM | Reverse
 * HIGH|HIGH | PWM | Brake
 */
class L298NMotor {
public:
    /**
     * @brief Constructor
     * @param in1 Direction pin 1 (GPIO)
     * @param in2 Direction pin 2 (GPIO)
     * @param en Enable/speed pin (PWM capable)
     */
    L298NMotor(int in1, int in2, int en);
    
    /**
     * @brief Set motor speed and direction
     * @param speed -255 (full reverse) to 255 (full forward), 0 = stop
     */
    void setSpeed(int speed);
    
    /**
     * @brief Drive motor forward
     * @param speed 0-255 PWM value
     */
    void forward(int speed);
    
    /**
     * @brief Drive motor in reverse
     * @param speed 0-255 PWM value
     */
    void reverse(int speed);
    
    /**
     * @brief Stop motor (coast)
     */
    void stop();
    
    /**
     * @brief Brake motor (active braking)
     */
    void brake();
    
    /**
     * @brief Get current speed setting
     * @return Current speed (-255 to 255)
     */
    int getSpeed();
    
private:
    int _in1;           // Direction pin 1
    int _in2;           // Direction pin 2
    int _en;            // Enable/PWM pin
    int _currentSpeed;  // Current speed value
};

#endif // L298N_MOTOR_H