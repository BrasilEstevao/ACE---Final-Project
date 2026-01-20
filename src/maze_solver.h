/* Maze solver - Finite State Machine (RobotStateMachine style)
   NO motor control, NO PID - just state transitions */

#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include <Arduino.h>
#include "LineSensor.h"

// Maze solving states
typedef enum {
  MAZE_IDLE,
  MAZE_FOLLOWING,
  MAZE_JUNCTION_DETECTED,
  MAZE_SMALL_FORWARD,
  MAZE_TURNING_LEFT,
  MAZE_TURNING_RIGHT,
  MAZE_TURNING_AROUND,
  MAZE_FINISHED,
  MAZE_LOST
} MazeState;

class MazeSolver {
public:
  // ========================================================================
  // STATE MACHINE VARIABLES
  // ========================================================================
  MazeState state;           // Current state
  MazeState last_state;      // Previous state
  
  unsigned long tes;         // Time Entering State (ms)
  unsigned long tis;         // Time In State (ms)
  
  JunctionType stored_junction;  // Stored junction for decision making
  
  // ========================================================================
  // CONSTRUCTOR
  // ========================================================================
  MazeSolver();
  
  // ========================================================================
  // MAIN UPDATE (call every control loop)
  // ========================================================================
  void update(JunctionType current_junction, float rel_angle, float rel_dist);
  
  // ========================================================================
  // STATE OUTPUTS (what main loop should do based on state)
  // ========================================================================
  bool shouldFollowLine();
  bool shouldGoForward();     // Small forward before turn
  bool shouldTurnLeft();
  bool shouldTurnRight();
  bool shouldTurnAround();
  bool isFinished();
  bool isLost();
  
  // ========================================================================
  // CONTROL
  // ========================================================================
  void start();
  void stop();
  void reset();
  
  // ========================================================================
  // GETTERS
  // ========================================================================
  MazeState getState();
  unsigned long getTimeInState();
  
private:
  // ========================================================================
  // INTERNAL STATE MANAGEMENT
  // ========================================================================
  unsigned long state_entry_time;
  
  void changeState(MazeState new_state);
  
  // ========================================================================
  // STATE TRANSITIONS (one function per state)
  // ========================================================================
  MazeState transitionIdle(JunctionType junction);
  MazeState transitionFollowing(JunctionType junction);
  MazeState transitionSmallForward(JunctionType junction, float rel_dist);
  MazeState transitionTurnLeft(JunctionType junction, float rel_angle);
  MazeState transitionTurnRight(JunctionType junction, float rel_angle);
  MazeState transitionTurnAround(JunctionType junction, float rel_angle);
  MazeState transitionLost(JunctionType junction);
  MazeState transitionFinished();
};

#endif // MAZE_SOLVER_H