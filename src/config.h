#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// MOTOR DRIVER
// ============================================================================

#define MOTOR1A_PIN 16
#define MOTOR1B_PIN 17

#define MOTOR2A_PIN 14
#define MOTOR2B_PIN 15


// ============================================================================
// SONAR SENSOR PINS
// ============================================================================
#define SONAR_TRIG     7
#define SONAR_ECHO     6

// ============================================================================
// IR SENSOR ARRAY - MULTIPLEXER CONFIGURATION
// ============================================================================
// Analog multiplexer control pins
#define MUXA_PIN       18    // MUX control bit 0
#define MUXB_PIN       19    // MUX control bit 1
#define MUXC_PIN       20    // MUX control bit 2

// ADC input pin (where multiplexer output connects)
#define ADC_IN_PIN     28    // A2 on Pico (ADC2)

// MUX channels for IR sensors (channels 3-7 on the multiplexer)
#define IR_SENSOR_COUNT    5
#define IR_MUX_START_CH    3  // First MUX channel for IR sensors

// IR detection threshold
#define IR_THRESHOLD       700  // Threshold for black/white detection (0-1023)

// ============================================================================
// ENCODER PINS
// ============================================================================

#define ENC1_A 2
#define ENC1_B 3

#define ENC2_A 0
#define ENC2_B 1


// ============================================================================
// ROBOT PHYSICAL PARAMETERS 
// ============================================================================
#define WHEEL_DISTANCE  0.113f
#define WHEEL_RADIUS    0.03445f

// ============================================================================
// CONTROL TIMING 
// ============================================================================
#define CONTROL_LOOP_MS     10
#define CONTROL_LOOP_S      0.010f

// ============================================================================
// MOTOR SPEEDS
// ============================================================================

#define BASE_SPEED 50 
#define MAX_SPEED       200
#define MAX_CORRECTION  150
#define TURN_SPEED      100
#define ALIGN_FACTOR 1.05

// ============================================================================
// LINE FOLLOWING PID
// ============================================================================
#define LINE_KP         0.044f
#define LINE_KI         0.0f
#define LINE_KD         0.007246494f

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define NUM_ANALOG_SENSORS  5  // All 5 sensors via multiplexer
#define NUM_DIGITAL_SENSORS 0  // No digital sensors in this config

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