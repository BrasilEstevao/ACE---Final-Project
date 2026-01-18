#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// MOTOR DRIVER L298N PINS
// ============================================================================
// Motor 1 (Left)
#define MOTOR1_IN1      4
#define MOTOR1_IN2      3
#define MOTOR1_EN       5

// Motor 2 (Right)
#define MOTOR2_IN3      2
#define MOTOR2_IN4      1
#define MOTOR2_EN       0

// ============================================================================
// IR SENSOR ARRAY (Direct GPIO Connection)
// ============================================================================
// Sensores das pontas (DIGITAIS) - para detecÃ§Ã£o de junÃ§Ãµes
// Os sensores IR digitais retornam LOW quando detectam preto
#define IR_LEFT_PIN     21    // IR1 - Sensor extremo esquerdo (DIGITAL)
#define IR_RIGHT_PIN    22    // IR5 - Sensor extremo direito (DIGITAL)

// Sensores centrais (ANALÃ“GICOS) - para PID line following
#define IR_CENTER_LEFT  26    // IR4 - A0
#define IR_CENTER       27    // IR3 - A1
#define IR_CENTER_RIGHT 28    // IR2 - A2

// ============================================================================
// ENCODER PINS
// ============================================================================
// Left encoder
#define ENC1_A          15 
#define ENC1_B          16

// Right encoder
#define ENC2_A          17
#define ENC2_B          18

// ============================================================================
// ODOMETRY CONFIGURATION
// ============================================================================
// Based on Paulo Costa's robot implementation
#define WHEEL_DISTANCE  0.105f        // Distance between wheels (meters)
#define WHEEL_RADIUS    0.03445f      // Wheel radius (meters) - 0.0689/2
        

// Turn angles (radians) - Will be calibrated automatically or manually
#define TURN_90_ANGLE   (PI / 2.0f)  // 90 degrees
#define TURN_180_ANGLE  PI            // 180 degrees

// Junction forward distance (meters)
#define SMALL_FWD_DISTANCE  0.01f     // 5cm forward at junctions

// ============================================================================
// MOTOR SPEED CONSTANTS
// ============================================================================
#define BASE_SPEED      90
#define MAX_SPEED       200
#define TURN_SPEED      100

// ============================================================================
// PID TUNING PARAMETERS
// ============================================================================
#define KP              0.3f
#define KI              0.0f
#define KD              0.5f

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define NUM_ANALOG_SENSORS  3    // Sensores centrais para PID
#define NUM_DIGITAL_SENSORS 2    // Sensores das pontas para junÃ§Ãµes
#define IR_THRESHOLD         520 // Threshold para sensores analÃ³gicos

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define CONTROL_LOOP_MS     20

// Fallback time-based turn durations (used if odometry fails)
#define RIGHT_TURN_90_TIME_MS     600
#define LEFT_TURN_90_TIME_MS      600
#define TURN_180_TIME_MS          900
#define SMALL_FWD_MS              400

// ============================================================================
// WiFi CONFIGURATION (compile-time or runtime)
// ============================================================================
#ifndef WIFI_SSID
#define WIFI_SSID       "PicoRobot"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "robot123"
#endif

#ifndef TELNET_PORT
#define TELNET_PORT     8080
#endif

#endif // CONFIG_H