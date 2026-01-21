/* Maze solver - LEFT-HAND RULE Algorithm
   Finite State Machine + Position Tracking */

#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include <Arduino.h>
#include "LineSensor.h"
#include "WifiTerminal.h"

// ============================================================================
// DIRECTIONS (Absolute)
// ============================================================================
typedef enum {
  DIR_NORTH = 0,  // UP (initial direction)
  DIR_EAST = 1,   // RIGHT
  DIR_SOUTH = 2,  // DOWN
  DIR_WEST = 3    // LEFT
} Direction;

// ============================================================================
// MAZE SOLVING STATES
// ============================================================================
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

// ============================================================================
// MAZE SOLVER CLASS
// ============================================================================
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
  // POSITION TRACKING
  // ========================================================================
  int x;                     // Current X position in grid
  int y;                     // Current Y position in grid
  Direction currentDir;      // Current absolute direction
  
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
  void printPosition();
  void setTerminal(WiFiTerminal* terminal) { _terminal = terminal; }
  
  // ========================================================================
  // DEBUG OUTPUT (call from main to print via WiFi)
  // ========================================================================
  const char* getStateName();
  const char* getDirectionName();
  void getPositionString(char* buffer, size_t bufferSize);
  
private:
  // ========================================================================
  // INTERNAL STATE MANAGEMENT
  // ========================================================================
  unsigned long state_entry_time;

  WiFiTerminal* _terminal;
  
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
  
  // ========================================================================
  // LEFT-HAND RULE DECISION LOGIC
  // ========================================================================
  MazeState decideDirection(JunctionType junction);
  
  // ========================================================================
  // POSITION TRACKING
  // ========================================================================
  void updatePosition();
  void updateDirection(MazeState nextState);
};

#endif // MAZE_SOLVER_H