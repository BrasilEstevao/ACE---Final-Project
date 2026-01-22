/* Maze solver - Finite State Machine (RobotStateMachine style)
   NO motor control, NO PID - just state transitions */

#ifndef FOLLOW_MODE_H
#define FOLLOW_MODE_H

#include <Arduino.h>
#include "LineSensor.h"
#include "Sonar.h"

// Follow line states
typedef enum {
  FOLLOW_IDLE,
  FOLLOW_LINE,
  FOLLOW_LOST,
  FOLLOW_APPROACH,
  FOLLOW_BLOCKED,
  FOLLOW_AROUND,
} FollowState;

class FollowMode {
private:

  WiFiTerminal* _terminal;  // Pointer to terminal
  Sonar* _Sonar1;  // Pointer to sonar

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

  void setSonar(Sonar* sonar) { _Sonar1 = sonar; }

  void setTerminal(WiFiTerminal* terminal) { _terminal = terminal; }
  
  // ========================================================================
  // MAIN UPDATE 
  // ========================================================================
  void update(JunctionType current_junction, float rel_angle, float rel_dist);
  
  // ========================================================================
  // STATE OUTPUTS 
  // ========================================================================
  bool shouldFollowLine();
  bool shouldSpiral();
  bool shouldApproach();
  bool shouldStop();
  bool shouldGoAround();
  
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
  FollowState transitionLost(JunctionType junction, float rel_angle);
  FollowState transitionApproach();
  FollowState transitionBlocked();
  FollowState transitionAround(JunctionType junction);
};

#endif // FOLLOW_MODE_H