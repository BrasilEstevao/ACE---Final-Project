#ifndef CONFIG_H
#define CONFIG_H

// Motor pins
#define MOTOR1_IN1      4
#define MOTOR1_IN2      3
#define MOTOR1_EN       5
#define MOTOR2_IN3      2
#define MOTOR2_IN4      1
#define MOTOR2_EN       0

// IR sensors
#define IR_LEFT_PIN     21
#define IR_RIGHT_PIN    22
#define IR_CENTER_LEFT  26
#define IR_CENTER       27
#define IR_CENTER_RIGHT 28

// Encoders
#define ENC1_A          15
#define ENC1_B          16
#define ENC2_A          17
#define ENC2_B          18

// Motor speeds
#define BASE_SPEED      90
#define MAX_SPEED       200
#define TURN_SPEED      100

// PID tuning
#define KP              0.3f
#define KI              0.0f
#define KD              0.5f

// Sensor config
#define NUM_ANALOG_SENSORS  3
#define NUM_DIGITAL_SENSORS 2
#define IR_THRESHOLD        520

// Timing
#define CONTROL_LOOP_MS         20
#define RIGHT_TURN_90_TIME_MS   700
#define LEFT_TURN_90_TIME_MS    700
#define TURN_180_TIME_MS        900
#define SMALL_FWD_MS            350

// WiFi
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