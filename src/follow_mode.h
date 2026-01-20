/* Maze solver - Finite State Machine (RobotStateMachine style)
   NO motor control, NO PID - just state transitions */

#ifndef FOLLOW_MODE_H
#define FOLLOW_MODE_H

#include <Arduino.h>
#include "LineSensor.h"

// Follow line states
typedef enum {
  FOLLOW_IDLE,
  FOLLOW_LINE,
  FOLLOW_TURN_AROUND,
} FollowState;

class FollowMode {
public:
  // ========================================================================
  // STATE MACHINE VARIABLES
  // ========================================================================
  FollowState state;           // Current state
  FollowState last_state;      // Previous state
  
  unsigned long tes;         // Time Entering State (ms)
  unsigned long tis;         // Time In State (ms)
  
  JunctionType stored_junction;  // Stored junction for decision making
  
  // ========================================================================
  // CONSTRUCTOR
  // ========================================================================
  FollowMode();
  
  // ========================================================================
  // MAIN UPDATE 
  // ========================================================================
  void update(JunctionType current_junction, float rel_angle, float rel_dist);
  
  // ========================================================================
  // STATE OUTPUTS 
  // ========================================================================
  bool shouldFollowLine();
  bool shouldTurnAround();
  
  // ========================================================================
  // CONTROL
  // ========================================================================
  void start();
  void stop();
  void reset();
  
  // ========================================================================
  // GETTERS
  // ========================================================================
  FollowState getState();
  unsigned long getTimeInState();
  
private:
  // ========================================================================
  // INTERNAL STATE MANAGEMENT
  // ========================================================================
  unsigned long state_entry_time;
  
  void changeState(FollowState new_state);
  
  // ========================================================================
  // STATE TRANSITIONS 
  // ========================================================================
  FollowState transitionIdle();
  FollowState transitionFollowing(JunctionType junction);
  FollowState transitionTurnAround(JunctionType junction, float rel_angle);
};

#endif // FOLLOW_MODE_H