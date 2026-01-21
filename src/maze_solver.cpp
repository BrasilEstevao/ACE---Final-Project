#include "maze_solver.h"
#include "config.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

MazeSolver::MazeSolver()
{
  state = MAZE_IDLE;
  last_state = MAZE_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = 0;
  tes = 0;
  tis = 0;
  
  // Start at origin facing NORTH
  x = 0;
  y = 0;
  currentDir = DIR_NORTH;
}

// ============================================================================
// UPDATE - Main state machine loop
// ============================================================================

void MazeSolver::update(JunctionType current_junction, float rel_angle, float rel_dist)
{
  // Calculate timing
  tis = millis() - state_entry_time;
  tes = millis();
  
  // STATE TRANSITIONS
  MazeState next_state = state;
  
  switch (state) {
    case MAZE_IDLE:
      next_state = transitionIdle(current_junction);
      break;
      
    case MAZE_FOLLOWING:
      next_state = transitionFollowing(current_junction);
      break;
      
    case MAZE_SMALL_FORWARD:
      next_state = transitionSmallForward(current_junction, rel_dist);
      break;
      
    case MAZE_TURNING_LEFT:
      next_state = transitionTurnLeft(current_junction, rel_angle);
      break;
      
    case MAZE_TURNING_RIGHT:
      next_state = transitionTurnRight(current_junction, rel_angle);
      break;
      
    case MAZE_TURNING_AROUND:
      next_state = transitionTurnAround(current_junction,rel_angle);
      break;
      
    case MAZE_LOST:
      next_state = transitionLost(current_junction);
      break;
      
    case MAZE_FINISHED:
      next_state = transitionFinished();
      break;
  }
  
  // Update direction based on state change
  if (next_state != state) {
    updateDirection(next_state);
    changeState(next_state);
  }
}

// ============================================================================
// STATE TRANSITIONS
// ============================================================================

MazeState MazeSolver::transitionIdle(JunctionType junction)
{
  return MAZE_IDLE;
}

MazeState MazeSolver::transitionFollowing(JunctionType junction)
{
  switch (junction) {
    case JUNCTION_LEFT:
    case JUNCTION_RIGHT:
    case JUNCTION_T:
      stored_junction = junction;
      _terminal->print("[MAZE] Junction detected: ");
      if (junction == JUNCTION_LEFT) _terminal->println("LEFT");
      else if (junction == JUNCTION_RIGHT) _terminal->println("RIGHT");
      else _terminal->println("T");
      return MAZE_SMALL_FORWARD;

    case JUNCTION_NONE:
      return MAZE_FOLLOWING;

    case JUNCTION_LOST:
      _terminal->println("[MAZE] Lost line - turning around");
      return MAZE_TURNING_AROUND;
      
    default:
      return MAZE_FOLLOWING;
  }
}

MazeState MazeSolver::transitionSmallForward(JunctionType junction, float rel_dist)
{
  if((junction == JUNCTION_LEFT) || (junction == JUNCTION_RIGHT) || (junction == JUNCTION_T) || (rel_dist < SMALL_FWD_DISTANCE)) {
    return MAZE_SMALL_FORWARD;
  }
  
  _terminal->print("[MAZE] Small forward complete (");
  _terminal->print(rel_dist * 100, 1);
  _terminal->println("cm)");
  
  // Update position (moved forward one cell)
  updatePosition();
  
  // Apply LEFT-HAND RULE decision
  return decideDirection(junction);

}

MazeState MazeSolver::transitionTurnLeft(JunctionType junction, float rel_angle)
{
  // TIMEOUT
  // if (tis > 3000) {
  //   _terminal->println("[MAZE] Left turn TIMEOUT");
  //   updatePosition();
  //   return MAZE_FOLLOWING;
  // }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_90_ANGLE) {
    if (junction == JUNCTION_NONE) {
      _terminal->print("[MAZE] Left turn complete (");
      _terminal->print(fabs(rel_angle) * 180.0f / PI, 1);
      _terminal->println("°)");
      updatePosition();
      return MAZE_FOLLOWING;
    }
  }
  
  return MAZE_TURNING_LEFT;
}

MazeState MazeSolver::transitionTurnRight(JunctionType junction, float rel_angle)
{
  // TIMEOUT
  // if (tis > 3000) {
  //   _terminal->println("[MAZE] Right turn TIMEOUT");
  //   updatePosition();
  //   return MAZE_FOLLOWING;
  // }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_90_ANGLE) {
    if (junction == JUNCTION_NONE) {
      _terminal->print("[MAZE] Right turn complete (");
      _terminal->print(fabs(rel_angle) * 180.0f / PI, 1);
      _terminal->println("°)");
      updatePosition();
      return MAZE_FOLLOWING;
    }
  }
  
  return MAZE_TURNING_RIGHT;
}

MazeState MazeSolver::transitionTurnAround(JunctionType junction, float rel_angle)
{
  // TIMEOUT
  if (tis > 5000) {
    _terminal->println("[MAZE] U-turn TIMEOUT");
    updatePosition();
    return MAZE_FINISHED;
  }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_180_ANGLE) {
    if (junction == JUNCTION_NONE) {
      _terminal->print("[MAZE] U-turn complete (");
      _terminal->print(fabs(rel_angle) * 180.0f / PI, 1);
      _terminal->println("°)");
      updatePosition();
      return MAZE_FOLLOWING;
    }
  }
  
  return MAZE_TURNING_AROUND;
}

MazeState MazeSolver::transitionLost(JunctionType junction)
{
  if (tis > 1000) {
    _terminal->println("[MAZE] Lost state timeout");
    return MAZE_IDLE;
  }
  return MAZE_LOST;
}

MazeState MazeSolver::transitionFinished()
{
  return MAZE_FINISHED;
}

// ============================================================================
// LEFT-HAND RULE DECISION LOGIC
// ============================================================================

MazeState MazeSolver::decideDirection(JunctionType junction)
{
  _terminal->println("\n[MAZE] === LEFT-HAND RULE DECISION ===");
  
  // Check what the current junction looks like AFTER small forward
  switch (junction) {

    case JUNCTION_NONE:
      // Line continues - decide based on what junction we stored
      _terminal->print("[MAZE] Line found. Original junction was: ");
      
      switch (stored_junction) {
        case JUNCTION_LEFT:
          // Had left turn option - LEFT-HAND RULE says TAKE IT
          _terminal->println("LEFT → Decision: TURN LEFT");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_T:
          // Was a T, but now line continues = prioritize LEFT
          _terminal->println("T → Decision: TURN LEFT");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_RIGHT:
          // Had only right turn option - continue straight
          _terminal->println("RIGHT → Decision: CONTINUE STRAIGHT");
          return MAZE_FOLLOWING;
          
        default:
          _terminal->println("UNKNOWN → Decision: CONTINUE STRAIGHT");
          return MAZE_FOLLOWING;
      }
      break;
      
    case JUNCTION_LOST:
      // No line after small forward = forced turn
      _terminal->print("[MAZE] No line! Original junction was: ");
      
      switch (stored_junction) {
        case JUNCTION_LEFT:
          // Had left option - MUST turn left now
          _terminal->println("LEFT → Decision: TURN LEFT (forced)");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_T:
          // Was a T - LEFT-HAND RULE = go left
          _terminal->println("T → Decision: TURN LEFT (forced)");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_RIGHT:
          // Had only right option - turn right
          _terminal->println("RIGHT → Decision: TURN RIGHT (forced)");
          return MAZE_TURNING_RIGHT;
          
        default:
          // Shouldn't happen, but turn around as safety
          _terminal->println("UNKNOWN → Decision: TURN AROUND");
          return MAZE_TURNING_AROUND;
      }
      break;
      
    default:
      _terminal->println("[MAZE] Unexpected junction state - continuing straight");
      return MAZE_FOLLOWING;
  }
  
  return MAZE_FOLLOWING;
}

// ============================================================================
// POSITION TRACKING
// ============================================================================

void MazeSolver::updatePosition()
{
  // Update grid position based on current direction
  switch (currentDir) {
    case DIR_NORTH:
      y++;
      _terminal->print("[MAZE] Moved NORTH → ");
      break;
    case DIR_EAST:
      x++;
      _terminal->print("[MAZE] Moved EAST → ");
      break;
    case DIR_SOUTH:
      y--;
      _terminal->print("[MAZE] Moved SOUTH → ");
      break;
    case DIR_WEST:
      x--;
      _terminal->print("[MAZE] Moved WEST → ");
      break;
  }
  
  printPosition();
}

void MazeSolver::updateDirection(MazeState nextState)
{
  // Update absolute direction based on turn
  switch (nextState) {
    case MAZE_TURNING_LEFT:
      // Counter-clockwise turn
      currentDir = (Direction)((currentDir + 3) % 4);
      _terminal->print("[MAZE] Direction changed to: ");
      break;
      
    case MAZE_TURNING_RIGHT:
      // Clockwise turn
      currentDir = (Direction)((currentDir + 1) % 4);
      _terminal->print("[MAZE] Direction changed to: ");
      break;
      
    case MAZE_TURNING_AROUND:
      // 180° turn
      currentDir = (Direction)((currentDir + 2) % 4);
      _terminal->print("[MAZE] Direction changed to: ");
      break;
      
    default:
      return; // No direction change
  }
  
  // Print new direction
  switch (currentDir) {
    case DIR_NORTH: _terminal->println("NORTH"); break;
    case DIR_EAST:  _terminal->println("EAST"); break;
    case DIR_SOUTH: _terminal->println("SOUTH"); break;
    case DIR_WEST:  _terminal->println("WEST"); break;
  }
}

void MazeSolver::printPosition()
{
  _terminal->print("Position (");
  _terminal->print(x);
  _terminal->print(", ");
  _terminal->print(y);
  _terminal->print(") facing ");
  
  switch (currentDir) {
    case DIR_NORTH: _terminal->println("NORTH ↑"); break;
    case DIR_EAST:  _terminal->println("EAST →"); break;
    case DIR_SOUTH: _terminal->println("SOUTH ↓"); break;
    case DIR_WEST:  _terminal->println("WEST ←"); break;
  }
}

// ============================================================================
// STATE OUTPUTS
// ============================================================================

bool MazeSolver::shouldFollowLine()
{
  return state == MAZE_FOLLOWING;
}

bool MazeSolver::shouldGoForward()
{
  return state == MAZE_SMALL_FORWARD;
}

bool MazeSolver::shouldTurnLeft()
{
  return state == MAZE_TURNING_LEFT;
}

bool MazeSolver::shouldTurnRight()
{
  return state == MAZE_TURNING_RIGHT;
}

bool MazeSolver::shouldTurnAround()
{
  return state == MAZE_TURNING_AROUND;
}

bool MazeSolver::isFinished()
{
  return state == MAZE_FINISHED;
}

bool MazeSolver::isLost()
{
  return state == MAZE_LOST;
}

// ============================================================================
// STATE CHANGE
// ============================================================================

void MazeSolver::changeState(MazeState new_state)
{
  if (state != new_state) {
    last_state = state;
    state = new_state;
    state_entry_time = millis();
  }
}

// ============================================================================
// CONTROL
// ============================================================================

void MazeSolver::start()
{
  // Reset position and direction
  x = 0;
  y = 0;
  currentDir = DIR_NORTH;
  
  changeState(MAZE_FOLLOWING);
  
  _terminal->println("\n========================================");
  _terminal->println("[MAZE] STARTED - LEFT-HAND RULE");
  _terminal->println("[MAZE] Starting position: (0, 0) facing NORTH");
  _terminal->println("========================================\n");
}

void MazeSolver::stop()
{
  changeState(MAZE_IDLE);
  _terminal->println("\n[MAZE] Stopped");
}

void MazeSolver::reset()
{
  state = MAZE_IDLE;
  last_state = MAZE_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = millis();
  tes = 0;
  tis = 0;
  x = 0;
  y = 0;
  currentDir = DIR_NORTH;
  
  _terminal->println("[MAZE] Reset");
}

// ============================================================================
// GETTERS
// ============================================================================

MazeState MazeSolver::getState()
{
  return state;
}

unsigned long MazeSolver::getTimeInState()
{
  return tis;
}

// ============================================================================
// DEBUG OUTPUT HELPERS
// ============================================================================

const char* MazeSolver::getStateName()
{
  switch (state) {
    case MAZE_IDLE: return "IDLE";
    case MAZE_FOLLOWING: return "FOLLOWING";
    case MAZE_JUNCTION_DETECTED: return "JUNCTION";
    case MAZE_SMALL_FORWARD: return "SMALL_FWD";
    case MAZE_TURNING_LEFT: return "TURN_LEFT";
    case MAZE_TURNING_RIGHT: return "TURN_RIGHT";
    case MAZE_TURNING_AROUND: return "U_TURN";
    case MAZE_FINISHED: return "FINISHED";
    case MAZE_LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

const char* MazeSolver::getDirectionName()
{
  switch (currentDir) {
    case DIR_NORTH: return "NORTH ↑";
    case DIR_EAST:  return "EAST →";
    case DIR_SOUTH: return "SOUTH ↓";
    case DIR_WEST:  return "WEST ←";
    default: return "UNKNOWN";
  }
}

void MazeSolver::getPositionString(char* buffer, size_t bufferSize)
{
  snprintf(buffer, bufferSize, "(%d, %d) %s", x, y, getDirectionName());
}