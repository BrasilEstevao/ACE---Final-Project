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
  tes = 0;  // Time Entering State
  tis = 0;  // Time In State
}

// ============================================================================
// UPDATE - Main state machine loop
// ============================================================================

void MazeSolver::update(JunctionType current_junction, float rel_angle, float rel_dist)
{
  // Calculate timing
  tis = millis() - state_entry_time;  // Time In State (how long in current state)
  tes = millis();                      // Time Entering State (absolute time)
  
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
      next_state = transitionTurnAround(current_junction, rel_angle);
      break;
      
    case MAZE_LOST:
      next_state = transitionLost(current_junction);
      break;
      
    case MAZE_FINISHED:
      next_state = transitionFinished();
      break;
  }
  
  // Change state if needed
  if (next_state != state) {
    changeState(next_state);
  }
}

// ============================================================================
// STATE TRANSITIONS
// ============================================================================

MazeState MazeSolver::transitionIdle(JunctionType junction)
{
  // Wait for start command - no automatic transitions
  return MAZE_IDLE;
}

MazeState MazeSolver::transitionFollowing(JunctionType junction)
{
  switch (junction) {
    case JUNCTION_LEFT:
      stored_junction = junction;
      Serial.println("[MAZE] Junction detected: LEFT");
      return MAZE_SMALL_FORWARD;
      
    case JUNCTION_RIGHT:
      stored_junction = junction;
      Serial.println("[MAZE] Junction detected: RIGHT");
      return MAZE_SMALL_FORWARD;
      
    case JUNCTION_T:
      stored_junction = junction;
      Serial.println("[MAZE] Junction detected: T");
      return MAZE_SMALL_FORWARD;
      
    case JUNCTION_LOST:
      Serial.println("[MAZE] Lost line - turning around");
      return MAZE_TURNING_AROUND;
      
    case JUNCTION_NONE:
    case JUNCTION_CROSS:
    default:
      return MAZE_FOLLOWING;
  }
}

MazeState MazeSolver::transitionSmallForward(JunctionType junction, float rel_dist)
{
  // TIMEOUT - Safety mechanism (using tis = time in state)
  if (tis > 1000) {
    Serial.println("[MAZE] Small forward TIMEOUT");
    return MAZE_FOLLOWING;
  }
  
  // Wait until traveled enough distance
  if (rel_dist < SMALL_FWD_DISTANCE) {
    return MAZE_SMALL_FORWARD;
  }
  
  Serial.print("[MAZE] Small forward complete (");
  Serial.print(rel_dist * 100, 1);
  Serial.println("cm)");
  
  // Decision tree based on current junction and stored junction
  switch (junction) {
    case JUNCTION_T:
      Serial.println("[MAZE] T-junction found - FINISHED!");
      return MAZE_FINISHED;
      
    case JUNCTION_NONE:
      // Line found - decide based on stored junction
      switch (stored_junction) {
        case JUNCTION_LEFT:
        case JUNCTION_T:
          Serial.println("[MAZE] Decision: TURN LEFT");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_RIGHT:
        default:
          Serial.println("[MAZE] Decision: CONTINUE STRAIGHT");
          return MAZE_FOLLOWING;
      }
      break;
      
    case JUNCTION_LOST:
      // No line - must turn
      switch (stored_junction) {
        case JUNCTION_LEFT:
        case JUNCTION_T:
          Serial.println("[MAZE] No line - TURN LEFT");
          return MAZE_TURNING_LEFT;
          
        case JUNCTION_RIGHT:
          Serial.println("[MAZE] No line - TURN RIGHT");
          return MAZE_TURNING_RIGHT;
          
        default:
          Serial.println("[MAZE] No line - TURN AROUND");
          return MAZE_TURNING_AROUND;
      }
      break;
      
    default:
      return MAZE_SMALL_FORWARD;
  }
  
  return MAZE_SMALL_FORWARD;
}

MazeState MazeSolver::transitionTurnLeft(JunctionType junction, float rel_angle)
{
  // TIMEOUT - Safety mechanism (using tis = time in state)
  if (tis > 3000) {
    Serial.println("[MAZE] Left turn TIMEOUT");
    return MAZE_FOLLOWING;
  }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_90_ANGLE * 0.9f) {
    switch (junction) {
      case JUNCTION_NONE:
        Serial.print("[MAZE] Left turn complete (");
        Serial.print(fabs(rel_angle) * 180.0f / PI, 1);
        Serial.println("°)");
        return MAZE_FOLLOWING;
        
      default:
        return MAZE_TURNING_LEFT;
    }
  }
  
  return MAZE_TURNING_LEFT;
}

MazeState MazeSolver::transitionTurnRight(JunctionType junction, float rel_angle)
{
  // TIMEOUT - Safety mechanism (using tis = time in state)
  if (tis > 3000) {
    Serial.println("[MAZE] Right turn TIMEOUT");
    return MAZE_FOLLOWING;
  }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_90_ANGLE * 0.9f) {
    switch (junction) {
      case JUNCTION_NONE:
        Serial.print("[MAZE] Right turn complete (");
        Serial.print(fabs(rel_angle) * 180.0f / PI, 1);
        Serial.println("°)");
        return MAZE_FOLLOWING;
        
      default:
        return MAZE_TURNING_RIGHT;
    }
  }
  
  return MAZE_TURNING_RIGHT;
}

MazeState MazeSolver::transitionTurnAround(JunctionType junction, float rel_angle)
{
  // TIMEOUT - Safety mechanism (using tis = time in state)
  if (tis > 5000) {
    Serial.println("[MAZE] U-turn TIMEOUT");
    return MAZE_FOLLOWING;
  }
  
  // Check if turned enough AND found line
  if (fabs(rel_angle) >= TURN_180_ANGLE * 0.9f) {
    switch (junction) {
      case JUNCTION_NONE:
        Serial.print("[MAZE] U-turn complete (");
        Serial.print(fabs(rel_angle) * 180.0f / PI, 1);
        Serial.println("°)");
        return MAZE_FOLLOWING;
        
      default:
        return MAZE_TURNING_AROUND;
    }
  }
  
  return MAZE_TURNING_AROUND;
}

MazeState MazeSolver::transitionLost(JunctionType junction)
{
  // Recovery logic could go here
  // For now, timeout back to idle (using tis = time in state)
  if (tis > 2000) {
    Serial.println("[MAZE] Lost state timeout");
    return MAZE_IDLE;
  }
  
  return MAZE_LOST;
}

MazeState MazeSolver::transitionFinished()
{
  // Stay finished
  return MAZE_FINISHED;
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
    
    // Debug output
    Serial.print("[MAZE] State: ");
    switch (new_state) {
      case MAZE_IDLE: Serial.println("IDLE"); break;
      case MAZE_FOLLOWING: Serial.println("FOLLOWING"); break;
      case MAZE_JUNCTION_DETECTED: Serial.println("JUNCTION"); break;
      case MAZE_SMALL_FORWARD: Serial.println("SMALL_FWD"); break;
      case MAZE_TURNING_LEFT: Serial.println("TURN_LEFT"); break;
      case MAZE_TURNING_RIGHT: Serial.println("TURN_RIGHT"); break;
      case MAZE_TURNING_AROUND: Serial.println("U_TURN"); break;
      case MAZE_FINISHED: Serial.println("FINISHED"); break;
      case MAZE_LOST: Serial.println("LOST"); break;
    }
  }
}

// ============================================================================
// CONTROL
// ============================================================================

void MazeSolver::start()
{
  changeState(MAZE_FOLLOWING);
  Serial.println("[MAZE] Started maze solving");
}

void MazeSolver::stop()
{
  changeState(MAZE_IDLE);
  Serial.println("[MAZE] Stopped");
}

void MazeSolver::reset()
{
  state = MAZE_IDLE;
  last_state = MAZE_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = millis();
  tes = 0;
  tis = 0;
  Serial.println("[MAZE] Reset");
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
  return tis;  // Time In State
}