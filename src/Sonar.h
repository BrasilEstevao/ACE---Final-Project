#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
#include "WiFiTerminal.h"

// Sonar states

typedef enum {
  SONAR_CLEAR,
  SONAR_OBSTRUCTED,
} SonarState;

class Sonar {
private:
  WiFiTerminal* _terminal;  // Pointer to terminal
  
public:
  // ========================================================================
  // STATE MACHINE VARIABLES
  // ========================================================================
  SonarState state;           // Current state
  SonarState last_state;      // Previous state
  
  unsigned long tes;         // Time Entering State (ms)
  unsigned long tis;         // Time In State (ms)
  float last_distance_cm; // Last measured distance in cm
  
  // ========================================================================
  // CONSTRUCTOR
  // ========================================================================
  Sonar();

  void setTerminal(WiFiTerminal* terminal) { _terminal = terminal; }

  // ========================================================================
  // MAIN UPDATE
  // ========================================================================
  void update();

  // ========================================================================
  // STATE OUTPUTS
  // ========================================================================
  bool isObstructed(); 
  
  // ========================================================================
  // GETTERS
  // ========================================================================
  SonarState getState();
  unsigned long getTimeInState();
  float getLastDistanceCm();

private:
  // ========================================================================       
  // INTERNAL STATE MANAGEMENT
  // ========================================================================
  unsigned long state_entry_time;
  int enter_count = 0;
  int exit_count = 0;
  static const int ENTER_THRESHOLD = 5;
  static const int EXIT_THRESHOLD = 5;

  void changeState(SonarState new_state);
};

// ============================================================================
// CONSTANTS
// ============================================================================

#define SONAR_OBSTRUCTION_THRESHOLD_CM   22.0f   // Distance threshold in cm
#define SONAR_APPROACH_DISTANCE_CM       6.0f    // Distance to consider "approached"

#endif // SONAR_H