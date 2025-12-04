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
// Sensores das pontas (DIGITAIS) - para detecção de junções
// Os sensores IR digitais retornam LOW quando detectam preto
#define IR_LEFT_PIN     21    // IR1 - Sensor extremo esquerdo (DIGITAL)
#define IR_RIGHT_PIN    22    // IR5 - Sensor extremo direito (DIGITAL)

// Sensores centrais (ANALÓGICOS) - para PID line following
#define IR_CENTER_LEFT  26    // IR4 - A0
#define IR_CENTER       27    // IR3 - A1
#define IR_CENTER_RIGHT 28    // IR2 - A2

// ============================================================================
// ENCODER PINS (Optional)
// ============================================================================
#define ENC1_A          2
#define ENC1_B          3
#define ENC2_A          0
#define ENC2_B          1

// ============================================================================
// MOTOR SPEED CONSTANTS
// ============================================================================
#define BASE_SPEED      120
#define MAX_SPEED       200
#define TURN_SPEED      100

// ============================================================================
// PID TUNING PARAMETERS
// ============================================================================
#define KP              0.15f
#define KI              0.0f
#define KD              0.35f

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define NUM_ANALOG_SENSORS  3    // Sensores centrais para PID
#define NUM_DIGITAL_SENSORS 2    // Sensores das pontas para junções
#define IR_THRESHOLD        850  // Threshold para sensores analógicos

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define CONTROL_LOOP_MS     20
#define RIGHT_TURN_90_TIME_MS     600
#define LEFT_TURN_90_TIME_MS      1000
#define TURN_180_TIME_MS    1200
#define JUNCTION_FWD_MS     150

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