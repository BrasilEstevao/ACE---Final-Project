#include <Arduino.h>
#include "config.h"
#include "L298NMotor.h"
#include "LineSensor.h"
#include "Odometry.h"
#include "RobotStateMachine.h"
#include "TestStateMachine.h"
#include "WiFiTerminal.h"

// Encoder globals
volatile int encoder1Pulses = 0;
volatile int encoder2Pulses = 0;

void encoder1ISR() {
    if (digitalRead(ENC1_B) == digitalRead(ENC1_A)) {
        encoder1Pulses++;
    } else {
        encoder1Pulses--;
    }
}

void encoder2ISR() {
    if (digitalRead(ENC2_B) == digitalRead(ENC2_A)) {
        encoder2Pulses++;
    } else {
        encoder2Pulses--;
    }
}

// Hardware objects
L298NMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
L298NMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);
LineSensor lineSensor;
Odometry odometry(0.13, 0.034, 40);

// State machines
RobotStateMachine robot(&leftMotor, &rightMotor, &lineSensor, &odometry);
TestStateMachine testSM(&leftMotor, &rightMotor, &odometry);

WiFiTerminal terminal;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastStatusTime = 0;

// Mode control
enum OperationMode { OP_ROBOT, OP_TEST };
OperationMode currentMode = OP_ROBOT;

// Output wrapper
class DualOutput {
public:
    void print(const char* str) { Serial.print(str); terminal.print(str); }
    void println(const char* str) { Serial.println(str); terminal.println(str); }
    void print(const String& str) { Serial.print(str); terminal.print(str); }
    void println(const String& str) { Serial.println(str); terminal.println(str); }
    void print(int val) { Serial.print(val); terminal.print(val); }
    void println(int val) { Serial.println(val); terminal.println(val); }
    void print(float val, int dec = 2) { Serial.print(val, dec); terminal.print(val, dec); }
    void println(float val, int dec = 2) { Serial.println(val, dec); terminal.println(val, dec); }
    void println() { Serial.println(); terminal.println(); }
} output;


// NOVA FUNÇÃO: Imprime o status da Odometria usando o DualOutput
void printOdometryStatus(Odometry& odom, DualOutput& out) {
    out.print("X: ");
    out.print(odom.getX(), 3);
    out.print("m Y: ");
    out.print(odom.getY(), 3);
    out.print("m Theta: ");
    out.print(odom.getTheta() * 180.0 / PI, 1);
    out.print("° | Rel: ");
    out.print(odom.getRelativeDistance() * 100, 1);
    out.print("cm / ");
    out.print(odom.getRelativeRotation() * 180.0 / PI, 1);
    out.print("° | V: ");
    out.print(odom.getLinearVelocity(), 3);
    out.print("m/s W: ");
    out.print(odom.getAngularVelocity(), 2);
    out.println("rad/s");
}

void processCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "help" || cmd == "?") {
        output.println("\n========== COMMANDS ==========");
        output.println("start          - Start robot");
        output.println("stop           - Stop robot");
        output.println("test           - Run odometry test");
        output.println("mode <1|3>     - Set mode (1=follow, 3=maze)");
        output.println("speed <n>      - Set speed");
        output.println("pid <p> <i> <d> - Set PID");
        output.println("odom           - Show odometry");
        output.println("reset_odom     - Reset odometry");
        output.println("calibrate      - Calibrate sensors");
        output.println("motors <l> <r> - Test motors");
        output.println("==============================\n");
    }
    else if (cmd == "start") {
        currentMode = OP_ROBOT;
        robot.start();
        output.println(">>> Robot STARTED");
    }
    else if (cmd == "stop") {
        robot.stop();
        testSM.stop();
        output.println(">>> STOPPED");
    }
    else if (cmd == "test") {
        currentMode = OP_TEST;
        testSM.start();
        output.println(">>> Test sequence STARTED");
    }
    else if (cmd == "odom") {
        output.println("\n--- Odometry ---");
        printOdometryStatus(odometry, output); // Chamada da nova função
        output.println("----------------\n");
    }
    else if (cmd == "reset_odom") {
        odometry.resetGlobal();
        output.println(">>> Odometry RESET");
    }
    else if (cmd.startsWith("mode ")) {
        int mode = cmd.substring(5).toInt();
        if (mode == 1) {
            robot.setMode(MODE_LINE_FOLLOW);
            output.println(">>> Mode: LINE_FOLLOW");
        } else if (mode == 3) {
            robot.setMode(MODE_MAZE_SOLVE);
            output.println(">>> Mode: MAZE_SOLVE");
        } else {
            output.println(">>> ERROR: Invalid mode");
        }
    }
    else if (cmd.startsWith("speed ")) {
        int speed = cmd.substring(6).toInt();
        robot.setSpeed(speed);
        output.print(">>> Speed: ");
        output.println(speed);
    }
    else if (cmd.startsWith("pid ")) {
        int s1 = cmd.indexOf(' ', 4);
        int s2 = cmd.indexOf(' ', s1 + 1);
        if (s1 > 0 && s2 > 0) {
            float kp = cmd.substring(4, s1).toFloat();
            float ki = cmd.substring(s1 + 1, s2).toFloat();
            float kd = cmd.substring(s2 + 1).toFloat();
            robot.setPID(kp, ki, kd);
            output.print(">>> PID: Kp=");
            output.print(kp, 3);
            output.print(" Ki=");
            output.print(ki, 3);
            output.print(" Kd=");
            output.println(kd, 3);
        }
    }
    else if (cmd == "calibrate") {
        output.println("\n>>> CALIBRATION");
        output.println("Starting in 3s...");
        delay(3000);
        robot.stop();
        lineSensor.calibrate();
        output.println(">>> Complete!");
    }
    else if (cmd.startsWith("motors ")) {
        int sp = cmd.indexOf(' ', 7);
        if (sp > 0) {
            int left = cmd.substring(7, sp).toInt();
            int right = cmd.substring(sp + 1).toInt();
            robot.stop();
            leftMotor.setSpeed(left);
            rightMotor.setSpeed(right);
            output.print(">>> Motors: L=");
            output.print(left);
            output.print(" R=");
            output.println(right);
        }
    }
    else if (cmd.length() > 0) {
        output.println(">>> Unknown command");
    }
    
    output.print("> ");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.println("\n========================================");
    Serial.println(" LINE FOLLOWING ROBOT");
    Serial.println("========================================\n");
    
    // Setup encoders
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);
    
    Serial.println("[OK] Encoders configured");
    
    // Setup WiFi
    if (terminal.begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("[OK] WiFi ready");
    } else {
        Serial.println("[!] WiFi failed");
    }
    
    // Blink ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    
    Serial.println("\n========================================");
    Serial.println("   READY!");
    Serial.println("========================================");
    Serial.println("Type 'help' for commands\n");
    Serial.print("> ");
}

void loop() {
    unsigned long currentTime = millis();
    
    terminal.update();
    
    // Control loop (50Hz)
    if (currentTime - lastLoopTime >= CONTROL_LOOP_MS) {
        float dt = (currentTime - lastLoopTime) / 1000.0;
        lastLoopTime = currentTime;
        
        // Read encoders
        noInterrupts();
        int enc1 = encoder1Pulses;
        int enc2 = encoder2Pulses;
        encoder1Pulses = 0;
        encoder2Pulses = 0;
        interrupts();
        
        // Update odometry
        odometry.update(enc1, enc2, dt);
        
        // Update active state machine
        if (currentMode == OP_ROBOT) {
            robot.update();
        } else if (currentMode == OP_TEST) {
            testSM.update();
            
            // Auto-stop when test complete
            if (testSM.isComplete()) {
                currentMode = OP_ROBOT;
                output.println("\n>>> Test sequence COMPLETE");
                output.print("> ");
            }
        }
    }
    
    // Command handling
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            Serial.println(cmd);
            processCommand(cmd);
        }
    }
    
    String wifiCmd = terminal.readLine();
    if (wifiCmd.length() > 0) {
        Serial.print("WiFi: ");
        Serial.println(wifiCmd);
        processCommand(wifiCmd);
    }
    
    // Status output (1Hz)
    if (currentTime - lastStatusTime >= 1000) {
        lastStatusTime = currentTime;
        
        if (currentMode == OP_ROBOT && robot.getState() != STATE_IDLE) {
            robot.printStatus();
        } else if (currentMode == OP_TEST && !testSM.isComplete()) {
            testSM.printStatus();
        }
    }
}