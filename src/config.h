#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// MOTOR DRIVER L298N PINS
// ============================================================================
#define MOTOR1_IN1      4
#define MOTOR1_IN2      3
#define MOTOR1_EN       5

#define MOTOR2_IN3      2
#define MOTOR2_IN4      1
#define MOTOR2_EN       0

// ============================================================================
// SONAR SENSOR PINS
// ============================================================================
#define SONAR_TRIG     7
#define SONAR_ECHO     6

// ============================================================================
// IR SENSOR ARRAY
// ============================================================================
#define IR_LEFT_PIN     21    // Digital edge sensor
#define IR_RIGHT_PIN    22    // Digital edge sensor

#define IR_CENTER_LEFT  26    // A0 - Analog
#define IR_CENTER       27    // A1 - Analog
#define IR_CENTER_RIGHT 28    // A2 - Analog

// ============================================================================
// ENCODER PINS
// ============================================================================
#define ENC1_A          15 
#define ENC1_B          16
#define ENC2_A          17
#define ENC2_B          18

// ============================================================================
// ROBOT PHYSICAL PARAMETERS 
// ============================================================================
//#define WHEEL_DISTANCE  0.105f        // m (distance between wheels)
#define WHEEL_DISTANCE  0.113f
#define WHEEL_RADIUS    0.03445f      // m (wheel radius = 68.9mm / 2)

// ============================================================================
// CONTROL TIMING 
// ============================================================================
#define CONTROL_LOOP_MS     40        // 40ms = 25Hz
#define CONTROL_LOOP_S      0.040f    // 40ms in seconds

// ============================================================================
// MOTOR SPEEDS
// ============================================================================
#define BASE_SPEED      120
#define MAX_SPEED       200
#define TURN_SPEED      100

// ============================================================================
// LINE FOLLOWING PID
// ============================================================================
#define LINE_KP         0.3f
#define LINE_KI         0.0f
#define LINE_KD         0.5f

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define NUM_ANALOG_SENSORS  3
#define NUM_DIGITAL_SENSORS 2
#define IR_THRESHOLD        520

// ============================================================================
// MAZE SOLVING PARAMETERS
// ============================================================================
#define TURN_90_ANGLE       (PI / 2.0f)
#define TURN_180_ANGLE      PI
#define SMALL_FWD_DISTANCE  0.04f

// ============================================================================
// WIFI CONFIGURATION
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

#endif