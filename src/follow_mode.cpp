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
    case FOLLOW_APPROACH:
      next_state = transitionApproach();
      break;
    case FOLLOW_BLOCKED:
      next_state = transitionBlocked();
      break;
    case FOLLOW_AROUND:
      next_state = transitionAround(current_junction);
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

  if(JUNCTION_LOST == junction)
  {
    _terminal->println("[FOLLOW] Lost line");
    return FOLLOW_LOST;
  }

  return FOLLOW_LINE;

  // switch(_Sonar1->getState()) {
  //   case SONAR_OBSTRUCTED:
  //     _terminal->println("[FOLLOW] Path blocked - stopping");
  //     return FOLLOW_APPROACH;
  //   case SONAR_CLEAR:
  //   default:
  //     return FOLLOW_LINE;
  // }
}


FollowState FollowMode::transitionLost(JunctionType junction, float rel_angle)
{
  // Check if we found a line again
  if (junction != JUNCTION_LOST) {
        return FOLLOW_LINE;
  }

  return FOLLOW_LOST;
}

FollowState FollowMode::transitionApproach()
{
  if(_Sonar1->getLastDistanceCm() < SONAR_APPROACH_DISTANCE_CM) {
    return FOLLOW_BLOCKED;
  }
  return FOLLOW_APPROACH;
}

FollowState FollowMode::transitionBlocked()
{
  if(tis > 5000) { // If blocked for more than 5 seconds, go to wait
    return FOLLOW_AROUND;
  }
  switch(_Sonar1->getState()) {
    case SONAR_CLEAR:
      _terminal->println("[FOLLOW] Path clear - resuming line following");
      return FOLLOW_LINE;
    case SONAR_OBSTRUCTED:
    default:
  return FOLLOW_BLOCKED;
  }
}

FollowState FollowMode::transitionAround(JunctionType junction)
{
  if(tis > 1500) { // After 1.5 seconds, try to follow line again
    if(JUNCTION_NONE == junction){
      return FOLLOW_LINE;
    }
  }
  return FOLLOW_AROUND;
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

bool FollowMode::shouldStop()
{
  return state == FOLLOW_BLOCKED;
}

bool FollowMode::shouldApproach()
{
  return state == FOLLOW_APPROACH;
}

bool FollowMode::shouldGoAround()
{
  return state == FOLLOW_AROUND;
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
    _terminal->print("[FOLLOW] State: ");
    switch (new_state) {
      case FOLLOW_IDLE: _terminal->println("FOLLOW IDLE"); break;
      case FOLLOW_LINE: _terminal->println("FOLLOWING LINE"); break;
      case FOLLOW_LOST: _terminal->println("LOOKING FOR LINE"); break;
    }
  }
}

// ============================================================================
// CONTROL
// ============================================================================

void FollowMode::start()
{
  changeState(FOLLOW_LINE);
  _terminal->println("[FOLLOW] Started maze solving");
}

void FollowMode::stop()
{
  changeState(FOLLOW_IDLE);
  _terminal->println("[FOLLOW] Stopped");
}

void FollowMode::reset()
{
  state = FOLLOW_IDLE;
  last_state = FOLLOW_IDLE;
  stored_junction = JUNCTION_NONE;
  state_entry_time = millis();
  tes = 0;
  tis = 0;
  _terminal->println("[FOLLOW] Reset");
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