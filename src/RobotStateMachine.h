#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include "L298NMotor.h"
#include "LineSensor.h"

/**
 * @brief Robot operational states
 */
enum RobotState {
    STATE_IDLE,              // Stopped, waiting for command
    STATE_LINE_FOLLOW,       // Following line with PID
    STATE_SMALL_FORWARD,  // Moving forward at junction
    STATE_TURN_LEFT,         // Executing left turn
    STATE_TURN_RIGHT,        // Executing right turn
    STATE_TURN_AROUND,       // Executing U-turn
    STATE_OBSTACLE_AVOID,    // Avoiding obstacle
    STATE_MAZE_SOLVE,        // Solving maze (redirects to LINE_FOLLOW)
    STATE_LOST,               // Lost line, searching
    STATE_FINISHED
};

/**
 * @brief Robot operational modes (challenges)
 */
enum RobotMode {
    MODE_LINE_FOLLOW,      // Challenge 1: Simple line following
    MODE_OBSTACLE_AVOID,   // Challenge 2: Obstacle avoidance
    MODE_MAZE_SOLVE,       // Challenge 3: Grid maze solver
    MODE_NAVIGATE          // Challenge 4: Self-localization navigation
};


/**
 * @class RobotStateMachine
 * @brief Main robot behavior controller using 3-stage state machine pattern
 * 
 * State machine organization (inspired by control.cpp):
 * 1. updateStateTransitions() - Determine next state based on conditions
 * 2. updateOutputs()          - Calculate new motor speeds for current state
 * 3. applyOutputs()           - Apply calculated speeds to actual motors
 * 
 * This separation allows for:
 * - Clear logic flow
 * - Easy debugging
 * - State changes without immediate hardware impact
 * - Testability
 */
class RobotStateMachine {
public:
    /**
     * @brief Constructor
     * @param left Pointer to left motor object
     * @param right Pointer to right motor object
     * @param sensor Pointer to line sensor object
     */
    RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor);
    JunctionType _storedJunction = JUNCTION_LEFT;
    
    /**
     * @brief Update state machine (call in main loop at 50Hz)
     * 
     * This is the main entry point that orchestrates:
     * 1. Read sensors
     * 2. Update state transitions
     * 3. Calculate outputs
     * 4. Apply outputs
     */
    void update();
    
    /**
     * @brief Start robot operation
     */
    void start();
    
    /**
     * @brief Stop robot
     */
    void stop();
    
    /**
     * @brief Set operational mode
     * @param mode RobotMode enum value
     */
    void setMode(RobotMode mode);
    
    /**
     * @brief Set base speed
     * @param speed Speed value (0-255)
     */
    void setSpeed(int speed);
    
    /**
     * @brief Set PID parameters
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPID(float kp, float ki, float kd);
    
    /**
     * @brief Get current state
     * @return Current RobotState
     */
    RobotState getState();
    
    /**
     * @brief Get current mode
     * @return Current RobotMode
     */
    RobotMode getMode();
    
    /**
     * @brief Print status to Serial
     */
    void printStatus();
    
private:
    // ========================================================================
    // HARDWARE REFERENCES
    // ========================================================================
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    LineSensor* _lineSensor;
    
    // ========================================================================
    // STATE VARIABLES
    // ========================================================================
    RobotState _currentState;
    RobotMode _currentMode;
    
    // ========================================================================
    // PID CONTROL VARIABLES
    // ========================================================================
    float _kp, _ki, _kd;        // PID gains
    int _lastError;             // Previous error for derivative
    int _integral;              // Accumulated error for integral
    
    // ========================================================================
    // SPEED PARAMETERS
    // ========================================================================
    int _baseSpeed;             // Base forward speed
    int _maxSpeed;              // Maximum allowed speed
    
    // ========================================================================
    // OUTPUT VARIABLES (NEW values calculated in updateOutputs)
    // ========================================================================
    int _leftSpeedNew;          // New left motor speed
    int _rightSpeedNew;         // New right motor speed
    
    // ========================================================================
    // TIMING VARIABLES
    // ========================================================================
    unsigned long _turnStartTime;        // When turn started
    unsigned long _smallForwardTime;  // When junction forward started
    
    // ========================================================================
    // STATE MACHINE METHODS (3-stage pattern)
    // ========================================================================
    
    /**
     * @brief Stage 1: Update state transitions
     * 
     * Determines next state based on:
     * - Current state
     * - Sensor readings
     * - Timers
     * - Mode
     * 
     * NO motor commands here!
     */
    void updateStateTransitions();
    
    /**
     * @brief Stage 2: Calculate outputs
     * 
     * Sets _leftSpeedNew and _rightSpeedNew based on current state
     * Does NOT apply to motors yet
     */
    void updateOutputs();
    
    /**
     * @brief Stage 3: Apply outputs to hardware
     * 
     * Takes calculated speeds and commands actual motors
     */
    void applyOutputs();
    
    // ========================================================================
    // HELPER METHODS
    // ========================================================================
    
    /**
     * @brief Calculate PID motor speeds
     * 
     * Used by STATE_LINE_FOLLOW
     * Sets _leftSpeedNew and _rightSpeedNew
     */
    void calculatePIDSpeeds();
};

#endif // ROBOT_STATE_MACHINE_H