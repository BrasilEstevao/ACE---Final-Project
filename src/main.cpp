#include <Arduino.h>
#include "config.h"
#include "L298NMotor.h"
#include "LineSensor.h"
#include "Odometry.h"
#include "RobotStateMachine.h"
#include "WiFiTerminal.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

L298NMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
L298NMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);
LineSensor lineSensor;
Odometry odometry(WHEEL_DISTANCE, WHEEL_RADIUS);
RobotStateMachine robot(&leftMotor, &rightMotor, &lineSensor, &odometry);
WiFiTerminal terminal;

// ============================================================================
// TIMING VARIABLES
// ============================================================================

unsigned long lastLoopTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorPrintTime = 0;
bool sensorStreamEnabled = false;
bool odomStreamEnabled = false;

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
        output.println("odom           - Show odometry values once");
        output.println("ostream on/off - Stream odometry continuously");
        output.println("resetodom      - Reset odometry to zero");
        output.println("calibrate      - Calibrate analog sensors");
        output.println("");
        output.println("=== VELOCITY CONTROL MODE ===");
        output.println("vel <v> <w>    - Set velocity (v in m/s, w in rad/s)");
        output.println("               Example: vel 0.2 0.5");
        output.println("               v > 0 = forward, v < 0 = backward");
        output.println("               w > 0 = turn right, w < 0 = turn left");
        output.println("");
        output.println("=== GOTO COMMANDS ===");
        output.println("forward <m>    - Move forward/back specific distance");
        output.println("               Example: forward 0.5  (50cm forward)");
        output.println("               Example: forward -0.3 (30cm backward)");
        output.println("turn <deg>     - Turn specific angle");
        output.println("               Example: turn 90   (90Â° right)");
        output.println("               Example: turn -180 (180Â° left)");
        output.println("");
        output.println("=== CONFIGURATION ===");
        output.println("speed <n>      - Set speed (0-255)");
        output.println("mode <n>       - Set mode:");
        output.println("                 1=follow, 3=maze, 4=velocity");
        output.println("pid <p> <i> <d> - Set PID gains for line sensors");
        output.println("threshold <n>  - Set sensor threshold");
        output.println("motors <l> <r> - Test motors directly");
        output.println("velpid <gains> - Tune velocity PID controller");
        output.println("               6 params: kp_v ki_v kd_v kp_w ki_w kd_w");
        output.println("               Example: velpid 50 10 5 20 2 1");
        output.println("               Units: m/s for v, rad/s for w");
        output.println("pidmode <on|off> - Toggle PID velocity control");
        output.println("                 on  = closed-loop (uses odometry)");
        output.println("                 off = open-loop (direct PWM)");
        output.println("========================================\n");
    }
    else if (cmd == "start") {
        robot.start();
        output.println(">>> Robot STARTED (odometry reset)");
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
        
        output.print("Position: x=");
        output.print(odometry.getX(), 3);
        output.print("m y=");
        output.print(odometry.getY(), 3);
        output.print("m Angle=");
        output.print(odometry.getTheta() * 180.0f / PI, 1);
        output.println("dgs");
        
        output.print("Relative: dist=");
        output.print(odometry.getRelativeDistance(), 3);
        output.print("m angle=");
        output.print(odometry.getRelativeAngle() * 180.0f / PI, 1);
        output.println("Â°");
        
        output.print("Threshold: ");
        output.println(lineSensor.getThreshold());
        output.println("--------------------\n");

        output.print("Control Mode: ");
        output.println(robot.isPIDControlEnabled() ? "PID (closed-loop)" : "DIRECT (open-loop)");
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
    else if (cmd == "odom") {
        output.print("Position: x=");
        output.print(odometry.getX(), 3);
        output.print("m y=");
        output.print(odometry.getY(), 3);
        output.print("m Î¸=");
        output.print(odometry.getTheta() * 180.0f / PI, 1);
        output.println("Â°");
        
        output.print("Velocity: v=");
        output.print(odometry.getVelocity(), 3);
        output.print("m/s w=");
        output.print(odometry.getAngularVelocity(), 2);
        output.println("rad/s");
        
        output.print("Relative: dist=");
        output.print(odometry.getRelativeDistance(), 3);
        output.print("m angle=");
        output.print(odometry.getRelativeAngle() * 180.0f / PI, 1);
        output.println("Â°");
        
        output.print("Encoders: L=");
        output.print(odometry.getLeftEncoder());
        output.print(" R=");
        output.println(odometry.getRightEncoder());
    }
    else if (cmd == "resetodom") {
        odometry.reset();
        output.println(">>> Odometry RESET to zero");
    }
    else if (cmd.startsWith("vel ")) {
        int space = cmd.indexOf(' ', 4);
        if (space > 0) {
            float v = cmd.substring(4, space).toFloat();
            float w = cmd.substring(space + 1).toFloat();
            
            // Switch to velocity control mode if not already
            if (robot.getMode() != MODE_VELOCITY_CONTROL) {
                robot.setMode(MODE_VELOCITY_CONTROL);
                output.println(">>> Switched to VELOCITY_CONTROL mode");
            }
            
            robot.setVelocity(v, w);
            output.print(">>> Velocity set: v=");
            output.print(v, 3);
            output.print(" m/s, w=");
            output.print(w, 3);
            output.println(" rad/s");
            
            // Show what this means
            if (v > 0) output.print("    (Moving FORWARD");
            else if (v < 0) output.print("    (Moving BACKWARD");
            else output.print("    (STOPPED");
            
            if (w > 0) output.println(" while turning RIGHT)");
            else if (w < 0) output.println(" while turning LEFT)");
            else output.println(")");
        }
        else {
            output.println(">>> Usage: vel <v> <w>");
            output.println("    v = linear velocity (m/s)");
            output.println("    w = angular velocity (rad/s)");
            output.println("    Example: vel 0.2 0    (forward at 0.2 m/s)");
            output.println("    Example: vel 0 0.5    (spin right at 0.5 rad/s)");
            output.println("    Example: vel 0.1 -0.3 (forward + turn left)");
        }
    }
    else if (cmd.startsWith("forward ")) {
        float distance = cmd.substring(8).toFloat();
        robot.gotoDistance(distance);
        output.print(">>> Moving ");
        if (distance >= 0) {
            output.print("FORWARD ");
        } else {
            output.print("BACKWARD ");
        }
        output.print(fabs(distance) * 100.0f, 1);
        output.println(" cm");
    }
    else if (cmd.startsWith("turn ")) {
        float degrees = cmd.substring(5).toFloat();
        float radians = degrees * PI / 180.0f;
        robot.gotoAngle(radians);
        output.print(">>> Turning ");
        if (degrees >= 0) {
            output.print("RIGHT ");
        } else {
            output.print("LEFT ");
        }
        output.print(fabs(degrees), 1);
        output.println("Â°");
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
    else if (cmd.startsWith("ostream ")) {
        String option = cmd.substring(8);
        if (option == "on") {
            odomStreamEnabled = true;
            output.println(">>> Odometry stream ENABLED");
        } else if (option == "off") {
            odomStreamEnabled = false;
            output.println(">>> Odometry stream DISABLED");
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
            case 4:
                robot.setMode(MODE_VELOCITY_CONTROL);
                output.println(">>> Mode: VELOCITY_CONTROL");
                output.println("    Use 'vel <v> <w>' to control");
                output.println("    v = linear velocity (m/s)");
                output.println("    w = angular velocity (rad/s)");
                break;
            default:
                output.println(">>> ERROR: Invalid mode");
                output.println("    1 = LINE_FOLLOW");
                output.println("    3 = MAZE_SOLVE");
                output.println("    4 = VELOCITY_CONTROL");
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
    // ============================================================================
// ADICIONE ESTES COMANDOS NA FUNÃ‡ÃƒO processCommand() do main.cpp
// ============================================================================

    else if (cmd.startsWith("velpid ")) {
        // Formato: velpid <kp_v> <ki_v> <kd_v> <kp_w> <ki_w> <kd_w>
        int spaces[5];
        int spaceCount = 0;
        int pos = 7;
        
        while (spaceCount < 5 && pos < cmd.length()) {
            pos = cmd.indexOf(' ', pos);
            if (pos > 0) {
                spaces[spaceCount++] = pos;
                pos++;
            } else {
                break;
            }
        }
        
        if (spaceCount == 5) {
            float kp_v = cmd.substring(7, spaces[0]).toFloat();
            float ki_v = cmd.substring(spaces[0] + 1, spaces[1]).toFloat();
            float kd_v = cmd.substring(spaces[1] + 1, spaces[2]).toFloat();
            float kp_w = cmd.substring(spaces[2] + 1, spaces[3]).toFloat();
            float ki_w = cmd.substring(spaces[3] + 1, spaces[4]).toFloat();
            float kd_w = cmd.substring(spaces[4] + 1).toFloat();
            
            robot.setVelocityPID(kp_v, ki_v, kd_v, kp_w, ki_w, kd_w);
            
            output.println(">>> Velocity PID updated:");
            output.print("    Linear  (v): Kp=");
            output.print(kp_v, 1);
            output.print(" Ki=");
            output.print(ki_v, 1);
            output.print(" Kd=");
            output.println(kd_v, 1);
            output.print("    Angular (w): Kp=");
            output.print(kp_w, 1);
            output.print(" Ki=");
            output.print(ki_w, 1);
            output.print(" Kd=");
            output.println(kd_w, 1);
        }
        else {
            output.println(">>> Usage: velpid <kp_v> <ki_v> <kd_v> <kp_w> <ki_w> <kd_w>");
            output.println("    Example: velpid 50 10 5 20 2 1");
            output.println("    Current defaults:");
            output.println("      Linear  (v): Kp=50  Ki=10  Kd=5");
            output.println("      Angular (w): Kp=20  Ki=2   Kd=1");
            output.println("    NOTE: Gains are in m/s and rad/s units");
        }
    }
    else if (cmd.startsWith("pidmode ")) {
        String mode = cmd.substring(8);
        if (mode == "on" || mode == "1" || mode == "true") {
            robot.setPIDControlMode(true);
            output.println(">>> PID velocity control ENABLED (closed-loop)");
            output.println("    Uses odometry feedback for accurate control");
        } else if (mode == "off" || mode == "0" || mode == "false") {
            robot.setPIDControlMode(false);
            output.println(">>> Direct velocity control ENABLED (open-loop)");
            output.println("    Converts v/w directly to PWM (no feedback)");
        } else {
            output.println(">>> Usage: pidmode <on|off>");
            output.print("    Current: ");
            output.println(robot.isPIDControlEnabled() ? "PID (on)" : "DIRECT (off)");
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
    
    Serial.println("\n========================================");
    Serial.println(" LINE FOLLOWING ROBOT - WITH ODOMETRY");
    Serial.println("   Serial + WiFi Terminal");
    Serial.println("   Raspberry Pi Pico W + L298N");
    Serial.println("========================================\n");
    
    // Initialize odometry (encoders)
    Serial.println("[INIT] Setting up odometry...");
    odometry.begin();
    
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
    Serial.println("\nNOTE: Robot now uses ODOMETRY for turns");
    Serial.println("Calibrate wheel parameters if needed:");
    Serial.println("  WHEEL_DISTANCE = " + String(WHEEL_DISTANCE, 4) + " m");
    Serial.println("  WHEEL_RADIUS = " + String(WHEEL_RADIUS, 4) + " m");
    Serial.println();
    Serial.print("> ");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long currentTime = millis();
    
    // ========================================================================
    // UPDATE WIFI CONNECTION
    // ========================================================================
    terminal.update();
    
    // ========================================================================
    // CONTROL LOOP (20ms â‰ˆ 50 Hz)
    // ========================================================================
    if (currentTime - lastLoopTime >= CONTROL_LOOP_MS) {
        lastLoopTime = currentTime;
        robot.update();  // This now includes odometry update
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
    // STATUS OUTPUT (20ms)
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
            output.print(rightMotor.getSpeed());
            
            // Show relative odometry during turns and tests
            if (currentState == STATE_TURN_LEFT || 
                currentState == STATE_TURN_RIGHT || 
                currentState == STATE_TURN_AROUND ||
                currentState == STATE_SMALL_FORWARD ||
                currentState == STATE_VELOCITY_CONTROL ||
                currentState == STATE_GOTO_DISTANCE ||
                currentState == STATE_GOTO_ANGLE) {
                output.print(" [Î¸=");
                output.print(odometry.getRelativeAngle() * 180.0f / PI, 0);
                output.print("Â° d=");
                output.print(odometry.getRelativeDistance(), 2);
                output.print("m]");
                
                // For velocity control, show actual velocities
                if (currentState == STATE_VELOCITY_CONTROL ||
                    currentState == STATE_GOTO_DISTANCE ||
                    currentState == STATE_GOTO_ANGLE) {
                    output.print(" v=");
                    output.print(odometry.getVelocity(), 2);
                    output.print(" w=");
                    output.print(odometry.getAngularVelocity(), 2);
                }
            }
            
            output.println();
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
    
    // ========================================================================
    // ODOMETRY STREAM (200ms)
    // ========================================================================
    if (odomStreamEnabled && currentTime - lastSensorPrintTime >= 200) {
        output.print("O: x=");
        output.print(odometry.getX(), 2);
        output.print(" y=");
        output.print(odometry.getY(), 2);
        output.print(" Î¸=");
        output.print(odometry.getTheta() * 180.0f / PI, 0);
        output.print("Â° | v=");
        output.print(odometry.getVelocity(), 2);
        output.print(" w=");
        output.println(odometry.getAngularVelocity(), 1);
    }
}