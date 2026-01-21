#include <HCSR04.h>
#include "config.h"
#include "Sonar.h"
#include <Arduino.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Sonar::Sonar()
{
  state = SONAR_CLEAR;
  last_state = SONAR_CLEAR;
  state_entry_time = 0;
  tes = 0;  // Time Entering State
  tis = 0;  // Time In State
}

// ============================================================================
// UPDATE - Main state machine loop
// ============================================================================

void Sonar::update()
{
  //Serial.println("Updating Sonar...");
  // Calculate timing
  tis = millis() - state_entry_time;  // Time In State (how long in current state)
  tes = millis();                      // Time Entering State (absolute time)
  
  // Read distance from sonar
  double* distance_cm = HCSR04.measureDistanceCm();

//    _terminal->print("Sonar update called. Distance: ");
//    _terminal->println(distance_cm[0], 1);

  // STATE TRANSITIONS
  SonarState next_state = state;

  switch (state) {
    case SONAR_CLEAR:
      if (distance_cm[0] < SONAR_OBSTRUCTION_THRESHOLD_CM) {
        enter_count++;
        if (enter_count >= ENTER_THRESHOLD) {
          next_state = SONAR_OBSTRUCTED;
          enter_count = 0;  // Reset for next transition
          _terminal->println("Obstructed!");
        }
      } else if (enter_count > 0) {
        enter_count--;  // Debounce
      }
      break;
      
    case SONAR_OBSTRUCTED:
      if (distance_cm[0] >= SONAR_OBSTRUCTION_THRESHOLD_CM) {
        exit_count++;
        if (exit_count >= EXIT_THRESHOLD) {
          next_state = SONAR_CLEAR;
          exit_count = 0;  // Reset for next transition
          _terminal->println("Clear!");
        }
      } else if (exit_count > 0) {
        exit_count--;  // Debounce
      }
      break;
  }

  // Change state if needed
  if (next_state != state) {
    changeState(next_state);
  }

}

// =========================================================================
// GETTERS
// ========================================================================

SonarState Sonar::getState()
{
  return state;
}

// ========================================================================
// STATE CHANGE
// ========================================================================

void Sonar::changeState(SonarState new_state)
{
    last_state = state;
    state = new_state;
    state_entry_time = millis();
    //Serial.printf("[SONAR] State changed to: %s\n", (new_state == SONAR_CLEAR) ? "CLEAR" : "OBSTRUCTED");
}