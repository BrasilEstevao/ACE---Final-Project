#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include "L298NMotor.h"
#include "LineSensor.h"

/**
 * @brief Robot operational states
 */
enum RobotState {
    STATE_IDLE,              // Stopped, waiting for command
    STATE_LINE_FOLLOW,       // Following line
    STATE_JUNCTION_FORWARD,  // Moving forward at junction
    STATE_TURN_LEFT,         // Executing left turn
    STATE_TURN_RIGHT,        // Executing right turn
    STATE_TURN_AROUND,       // Executing U-turn
    STATE_OBSTACLE_AVOID,    // Avoiding obstacle
    STATE_MAZE_SOLVE,        // Solving maze
    STATE_LOST               // Lost line, searching
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
 * @brief Main robot behavior controller
 * 
 * Manages robot states and implements PID line following with
 * junction detection for maze navigation.
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
    
    /**
     * @brief Update state machine (call in main loop)
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
    // Hardware references
    L298NMotor* _leftMotor;
    L298NMotor* _rightMotor;
    LineSensor* _lineSensor;
    
    // State variables
    RobotState _currentState;
    RobotMode _currentMode;
    
    // PID variables
    float _kp, _ki, _kd;
    int _lastError;
    int _integral;
    
    // Speed parameters
    int _baseSpeed;
    int _maxSpeed;
    
    // Timing
    unsigned long _turnStartTime;
    unsigned long _junctionForwardTime;
    
    // State handlers
    void handleIdleState();
    void handleLineFollowState();
    void handleJunctionForwardState();
    void handleTurnLeftState();
    void handleTurnRightState();
    void handleTurnAroundState();
    void handleObstacleAvoidState();
    void handleMazeSolveState();
    void handleLostState();
    
    // Control methods
    void followLine();
};

#endif // ROBOT_STATE_MACHINE_H