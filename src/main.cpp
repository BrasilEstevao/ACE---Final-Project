#include <Arduino.h>
#include "config.h"
#include "L298NMotor.h"
#include "LineSensor.h"
#include "RobotStateMachine.h"
#include "WiFiTerminal.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

L298NMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
L298NMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);
LineSensor lineSensor;
RobotStateMachine robot(&leftMotor, &rightMotor, &lineSensor);
WiFiTerminal terminal;

// ============================================================================
// TIMING VARIABLES
// ============================================================================

unsigned long lastLoopTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorPrintTime = 0;
bool sensorStreamEnabled = false;

// ============================================================================
// ULTRASONIC SENSOR VARIABLES
// ============================================================================

unsigned long duration;
float distance;

// ============================================================================
// TERMINAL OUTPUT WRAPPER
// ============================================================================

class DualOutput {
public:
    void print(const char* str) {
        Serial.print(str);
        terminal.print(str);
    }
    
    void println(const char* str) {
        Serial.println(str);
        terminal.println(str);
    }
    
    void print(const String& str) {
        Serial.print(str);
        terminal.print(str);
    }
    
    void println(const String& str) {
        Serial.println(str);
        terminal.println(str);
    }
    
    void print(int val) {
        Serial.print(val);
        terminal.print(val);
    }
    
    void println(int val) {
        Serial.println(val);
        terminal.println(val);
    }
    
    void print(float val, int decimals = 2) {
        Serial.print(val, decimals);
        terminal.print(val, decimals);
    }
    
    void println(float val, int decimals = 2) {
        Serial.println(val, decimals);
        terminal.println(val, decimals);
    }
    
    void println() {
        Serial.println();
        terminal.println();
    }
} output;

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "help" || cmd == "?") {
        output.println("\n========== AVAILABLE COMMANDS ==========");
        output.println("help           - Show this help");
        output.println("start          - Start robot");
        output.println("stop           - Stop robot");
        output.println("status         - Show status");
        output.println("sensors        - Show sensor values once");
        output.println("stream on/off  - Stream sensors continuously");
        output.println("calibrate      - Calibrate analog sensors");
        output.println("speed <n>      - Set speed (0-255)");
        output.println("mode <n>       - Set mode (1=follow, 3=maze)");
        output.println("pid <p> <i> <d> - Set PID gains");
        output.println("threshold <n>  - Set sensor threshold");
        output.println("motors <l> <r> - Test motors directly");
        output.println("========================================\n");
    }
    else if (cmd == "start") {
        robot.start();
        output.println(">>> Robot STARTED");
    }
    else if (cmd == "stop") {
        robot.stop();
        output.println(">>> Robot STOPPED");
    }
    else if (cmd == "status") {
        output.println("\n--- Robot Status ---");
        
        output.print("State: ");
        RobotState currentState = robot.getState();
        switch(currentState) {
            case STATE_IDLE: output.print("IDLE"); break;
            case STATE_LINE_FOLLOW: output.print("LINE_FOLLOW"); break;
            case STATE_SMALL_FORWARD: output.print("SMALL_FWD"); break;
            case STATE_TURN_LEFT: output.print("TURN_LEFT"); break;
            case STATE_TURN_RIGHT: output.print("TURN_RIGHT"); break;
            case STATE_TURN_AROUND: output.print("TURN_AROUND"); break;
            case STATE_LOST: output.print("LOST"); break;
            default: output.print("UNKNOWN"); break;
        }
        output.println();
        
        output.print("Speed: L=");
        output.print(leftMotor.getSpeed());
        output.print(" R=");
        output.println(rightMotor.getSpeed());
        
        output.print("Threshold: ");
        output.println(lineSensor.getThreshold());
        output.println("--------------------\n");
    }
    else if (cmd == "sensors") {
        lineSensor.read();
        
        output.print("D: [");
        output.print(lineSensor.getDigitalSensorValue(0) ? "B" : "W"); 
        output.print("] A: [");
        output.print(lineSensor.getAnalogSensorValue(0)); 
        output.print(",");
        output.print(lineSensor.getAnalogSensorValue(1)); 
        output.print(",");
        output.print(lineSensor.getAnalogSensorValue(2)); 
        output.print("] D: [");
        output.print(lineSensor.getDigitalSensorValue(1) ? "B" : "W");
        output.print("] Pos: ");
        output.print(lineSensor.getPosition());
        output.print(" | ");
        
        JunctionType j = lineSensor.detectJunction();
        switch(j) {
            case JUNCTION_NONE: output.println("LINE"); break;
            case JUNCTION_LEFT: output.println("LEFT"); break;
            case JUNCTION_RIGHT: output.println("RIGHT"); break;
            case JUNCTION_T: output.println("T"); break;
            case JUNCTION_CROSS: output.println("CROSS"); break;
            case JUNCTION_LOST: output.println("LOST"); break;
        }
    }
    else if (cmd.startsWith("stream ")) {
        String option = cmd.substring(7);
        if (option == "on") {
            sensorStreamEnabled = true;
            output.println(">>> Sensor stream ENABLED");
        } else if (option == "off") {
            sensorStreamEnabled = false;
            output.println(">>> Sensor stream DISABLED");
        }
    }
    else if (cmd == "calibrate") {
        output.println("\n>>> CALIBRATION MODE <<<");
        output.println("Move robot over BLACK and WHITE surfaces");
        output.println("Calibrating in 3 seconds...");
        delay(3000);
        
        robot.stop();
        
        output.println("Calibrating...");
        lineSensor.calibrate();
        
        output.println(">>> Calibration complete!");
        output.print("Threshold: ");
        output.println(lineSensor.getThreshold());
    }
    else if (cmd.startsWith("speed ")) {
        int speed = cmd.substring(6).toInt();
        robot.setSpeed(speed);
        output.print(">>> Speed set to ");
        output.println(speed);
    }
    else if (cmd.startsWith("mode ")) {
        int mode = cmd.substring(5).toInt();
        switch(mode) {
            case 1:
                robot.setMode(MODE_LINE_FOLLOW);
                output.println(">>> Mode: LINE_FOLLOW");
                break;
            case 3:
                robot.setMode(MODE_MAZE_SOLVE);
                output.println(">>> Mode: MAZE_SOLVE");
                break;
            default:
                output.println(">>> ERROR: Invalid mode (use 1 or 3)");
        }
    }
    else if (cmd.startsWith("pid ")) {
        int space1 = cmd.indexOf(' ', 4);
        int space2 = cmd.indexOf(' ', space1 + 1);
        
        if (space1 > 0 && space2 > 0) {
            float kp = cmd.substring(4, space1).toFloat();
            float ki = cmd.substring(space1 + 1, space2).toFloat();
            float kd = cmd.substring(space2 + 1).toFloat();
            
            robot.setPID(kp, ki, kd);
            output.print(">>> PID set to: Kp=");
            output.print(kp, 3);
            output.print(" Ki=");
            output.print(ki, 3);
            output.print(" Kd=");
            output.println(kd, 3);
        }
        else {
            output.println(">>> Usage: pid <kp> <ki> <kd>");
        }
    }
    else if (cmd.startsWith("threshold ")) {
        int threshold = cmd.substring(10).toInt();
        lineSensor.setThreshold(threshold);
        output.print(">>> Threshold set to ");
        output.println(threshold);
    }
    else if (cmd.startsWith("motors ")) {
        int space = cmd.indexOf(' ', 7);
        if (space > 0) {
            int left = cmd.substring(7, space).toInt();
            int right = cmd.substring(space + 1).toInt();
            
            robot.stop();
            leftMotor.setSpeed(left);
            rightMotor.setSpeed(right);
            
            output.print(">>> Motors: L=");
            output.print(left);
            output.print(" R=");
            output.println(right);
            output.println("    (Type 'stop' to resume robot control)");
        }
        else {
            output.println(">>> Usage: motors <left> <right>");
        }
    }
    else if (cmd.length() > 0) {
        output.println(">>> Unknown command. Type 'help' for list.");
    }
    
    // Prompt for next command
    output.print("> ");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    delay(2000);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Ensure ultrasonic trig pin is low
    digitalWrite(TRIG_PIN, LOW);
    
    Serial.println("\n========================================");
    Serial.println(" LINE FOLLOWING ROBOT - DUAL CONTROL");
    Serial.println("   Serial + WiFi Terminal");
    Serial.println("   Raspberry Pi Pico W + L298N");
    Serial.println("========================================\n");
    
    // Initialize WiFi terminal
    if (terminal.begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("[OK] WiFi terminal ready");
    } else {
        Serial.println("[ERROR] WiFi terminal failed!");
        Serial.println("[INFO] Serial terminal still available");
    }
    
    // Blink LED to show ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    
    Serial.println("\n========================================");
    Serial.println("   SETUP COMPLETE - READY!");
    Serial.println("========================================");
    Serial.println("Type 'help' for available commands");
    Serial.println();
    Serial.print("> ");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long currentTime = millis();

    // ========================================================================
    // READ ULTRASONIC SENSOR
    // ========================================================================

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration*.0343)/2;

    Serial.print("Distance: ");
    Serial.println(distance);
    
    // ========================================================================
    // UPDATE WIFI CONNECTION
    // ========================================================================
    terminal.update();
    
    // ========================================================================
    // CONTROL LOOP (15ms ≈ 67 Hz)
    // ========================================================================
    if (currentTime - lastLoopTime >= CONTROL_LOOP_MS) {
        lastLoopTime = currentTime;
        robot.update();
    }
    
    // ========================================================================
    // COMMAND HANDLING - SERIAL
    // ========================================================================
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.length() > 0) {
            Serial.println(cmd);  // Echo command
            processCommand(cmd);
        }
    }
    
    // ========================================================================
    // COMMAND HANDLING - WIFI
    // ========================================================================
    String wifiCmd = terminal.readLine();
    if (wifiCmd.length() > 0) {
        Serial.print("WiFi CMD: ");
        Serial.println(wifiCmd);
        processCommand(wifiCmd);
    }
    
    // ========================================================================
    // STATUS OUTPUT (1 second)
    // ========================================================================
    if (currentTime - lastStatusTime >= 20) {
        lastStatusTime = currentTime;
        
        RobotState currentState = robot.getState();
        
        if (currentState != STATE_IDLE) {
            output.print("[");
            switch(currentState) {
                case STATE_IDLE: output.print("IDLE"); break;
                case STATE_LINE_FOLLOW: output.print("FOLLOW"); break;
                case STATE_SMALL_FORWARD: output.print("SMALL_FWD"); break;
                case STATE_TURN_LEFT: output.print("LEFT"); break;
                case STATE_TURN_RIGHT: output.print("RIGHT"); break;
                case STATE_TURN_AROUND: output.print("U-TURN"); break;
                case STATE_LOST: output.print("LOST"); break;
                case STATE_FINISHED: output.print("FINISHED"); break;
                default: output.print("?"); break;
            }
            output.print("] L=");
            output.print(leftMotor.getSpeed());
            output.print(" R=");
            output.println(rightMotor.getSpeed());
        }
    }
    
    // ========================================================================
    // SENSOR STREAM (200ms) 
    // ========================================================================
    if (sensorStreamEnabled && currentTime - lastSensorPrintTime >= 200) {
        lastSensorPrintTime = currentTime;
        
        lineSensor.read();
        
        output.print("S:");
        output.print(lineSensor.getDigitalSensorValue(0) ? "B" : "W");
        output.print("|");
        output.print(lineSensor.getAnalogSensorValue(0));
        output.print(",");
        output.print(lineSensor.getAnalogSensorValue(1));
        output.print(",");
        output.print(lineSensor.getAnalogSensorValue(2));
        output.print("|");
        output.print(lineSensor.getDigitalSensorValue(1) ? "B" : "W");
        output.print("|P:");
        output.println(lineSensor.getPosition());
    }
}