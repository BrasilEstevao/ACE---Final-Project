#include "follow_mode.h"
#include "config.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

FollowMode:: FollowMode()
{
  state = FOLLOW_IDLE;
  last_state = FOLLOW_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = 0;
  tes = 0;  // Time Entering State
  tis = 0;  // Time In State
}

// ============================================================================
// UPDATE - Main state machine loop
// ============================================================================

void FollowMode::update(JunctionType current_junction, float rel_angle, float rel_dist)
{
  // Calculate timing
  tis = millis() - state_entry_time;  // Time In State (how long in current state)
  tes = millis();                      // Time Entering State (absolute time)
  
  // STATE TRANSITIONS
  FollowState next_state = state;
  
  switch (state) {
    case FOLLOW_IDLE:
      next_state = transitionIdle();
      break;
      
    case FOLLOW_LINE:
      next_state = transitionFollowing(current_junction);
      break;

    case FOLLOW_LOST:
      next_state = transitionLost(current_junction, rel_angle);
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

FollowState FollowMode::transitionIdle()
{
  // Wait for start command - no automatic transitions
  return FOLLOW_IDLE;
}

FollowState FollowMode::transitionFollowing(JunctionType junction)
{
  switch (junction) {

    case JUNCTION_LOST:
      Serial.println("[FOLLOW] Lost line - turning around");
      return FOLLOW_LOST;
      
    case JUNCTION_NONE:
    case JUNCTION_CROSS:
    default:
      return FOLLOW_LINE;
  }
}


FollowState FollowMode::transitionLost(JunctionType junction, float rel_angle)
{
  // Check if we found a line again
  if (junction != JUNCTION_LOST) {
        return FOLLOW_LINE;
  }

  return FOLLOW_LOST;
}


// ============================================================================
// STATE OUTPUTS 
// ============================================================================

bool FollowMode::shouldFollowLine()
{
  return state == FOLLOW_LINE;
}

bool FollowMode::shouldSpiral()
{
  return state == FOLLOW_LOST;
}

// ============================================================================
// STATE CHANGE
// ============================================================================

void FollowMode::changeState(FollowState new_state)
{
  if (state != new_state) {
    last_state = state;
    state = new_state;
    state_entry_time = millis();
    
    // Debug output
    Serial.print("[FOLLOW] State: ");
    switch (new_state) {
      case FOLLOW_IDLE: Serial.println("FOLLOW IDLE"); break;
      case FOLLOW_LINE: Serial.println("FOLLOWING LINE"); break;
      case FOLLOW_LOST: Serial.println("LOOKING FOR LINE"); break;
    }
  }
}

// ============================================================================
// CONTROL
// ============================================================================

void FollowMode::start()
{
  changeState(FOLLOW_LINE);
  Serial.println("[FOLLOW] Started maze solving");
}

void FollowMode::stop()
{
  changeState(FOLLOW_IDLE);
  Serial.println("[FOLLOW] Stopped");
}

void FollowMode::reset()
{
  state = FOLLOW_IDLE;
  last_state = FOLLOW_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = millis();
  tes = 0;
  tis = 0;
  Serial.println("[FOLLOW] Reset");
}

// ============================================================================
// GETTERS
// ============================================================================

FollowState FollowMode::getState()
{
  return state;
}

unsigned long FollowMode::getTimeInState()
{
  return tis;  // Time In State
}