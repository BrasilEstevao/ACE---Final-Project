// #include <Arduino.h>
// #include "config.h"
// #include "L298NMotor.h"
// #include "LineSensor.h"
// #include "RobotStateMachine.h"

// // ============================================================================
// // GLOBAL OBJECTS
// // ============================================================================

// L298NMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
// L298NMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);
// LineSensor lineSensor;
// RobotStateMachine robot(&leftMotor, &rightMotor, &lineSensor);

// // ============================================================================
// // TIMING VARIABLES
// // ============================================================================

// unsigned long lastLoopTime = 0;
// unsigned long lastStatusTime = 0;
// unsigned long lastSensorPrintTime = 0;
// bool sensorStreamEnabled = false;

// // ============================================================================
// // COMMAND PROCESSING
// // ============================================================================

// void processCommand(String cmd) {
//     cmd.trim();
//     cmd.toLowerCase();
    
//     if (cmd == "help" || cmd == "?") {
//         Serial.println("\n========== AVAILABLE COMMANDS ==========");
//         Serial.println("help           - Show this help");
//         Serial.println("start          - Start robot");
//         Serial.println("stop           - Stop robot");
//         Serial.println("status         - Show status");
//         Serial.println("sensors        - Show sensor values once");
//         Serial.println("stream on/off  - Stream sensors continuously");
//         Serial.println("calibrate      - Calibrate analog sensors");
//         Serial.println("speed <n>      - Set speed (0-255)");
//         Serial.println("mode <n>       - Set mode (1=follow, 3=maze)");
//         Serial.println("pid <p> <i> <d> - Set PID gains");
//         Serial.println("threshold <n>  - Set sensor threshold");
//         Serial.println("motors <l> <r> - Test motors directly");
//         Serial.println("left <n>       - Test left motor only");
//         Serial.println("right <n>      - Test right motor only");
//         Serial.println("test           - Test individual sensors");
//         Serial.println("========================================\n");
//     }
//     else if (cmd == "start") {
//         robot.start();
//         Serial.println(">>> Robot STARTED");
//     }
//     else if (cmd == "stop") {
//         robot.stop();
//         Serial.println(">>> Robot STOPPED");
//     }
//     else if (cmd == "status") {
//         Serial.println("\n--- Robot Status ---");
//         robot.printStatus();
//         Serial.print("Threshold: ");
//         Serial.println(lineSensor.getThreshold());
//         Serial.println("--------------------\n");
//     }
//     else if (cmd == "sensors") {
//         lineSensor.read();
//         lineSensor.printValues();
//     }
//     else if (cmd == "test") {
//         Serial.println("\n========== SENSOR TEST ==========");
//         Serial.println("Reading all 5 sensors (10 samples)...");
//         Serial.println("Digital sensors: □=BLACK ■=WHITE\n");
//         for (int i = 0; i < 10; i++) {
//             lineSensor.read();
            
//             Serial.print("L_Dig: ");
//             Serial.print(lineSensor.getDigitalSensorValue(0) ? "■" : "□");
//             Serial.print(" | Analog: [");
//             Serial.print(lineSensor.getAnalogSensorValue(0));
//             Serial.print(", ");
//             Serial.print(lineSensor.getAnalogSensorValue(1));
//             Serial.print(", ");
//             Serial.print(lineSensor.getAnalogSensorValue(2));
//             Serial.print("] | R_Dig: ");
//             Serial.println(lineSensor.getDigitalSensorValue(1) ? "■" : "□");
            
//             delay(200);
//         }
//         Serial.println("=================================\n");
//     }
//     else if (cmd.startsWith("stream ")) {
//         String option = cmd.substring(7);
//         if (option == "on") {
//             sensorStreamEnabled = true;
//             Serial.println(">>> Sensor stream ENABLED");
//         } else if (option == "off") {
//             sensorStreamEnabled = false;
//             Serial.println(">>> Sensor stream DISABLED");
//         }
//     }
//     else if (cmd == "calibrate") {
//         Serial.println("\n>>> CALIBRATION MODE <<<");
//         Serial.println("This will calibrate the 3 CENTER analog sensors");
//         Serial.println("Move robot over BLACK and WHITE surfaces");
//         Serial.println("Calibrating in 3 seconds...");
//         delay(3000);
        
//         robot.stop();
//         lineSensor.calibrate();
//         Serial.println(">>> Calibration complete!\n");
//     }
//     else if (cmd.startsWith("speed ")) {
//         int speed = cmd.substring(6).toInt();
//         robot.setSpeed(speed);
//         Serial.print(">>> Speed set to ");
//         Serial.println(speed);
//     }
//     else if (cmd.startsWith("mode ")) {
//         int mode = cmd.substring(5).toInt();
//         switch(mode) {
//             case 1:
//                 robot.setMode(MODE_LINE_FOLLOW);
//                 Serial.println(">>> Mode: LINE_FOLLOW");
//                 break;
//             case 3:
//                 robot.setMode(MODE_MAZE_SOLVE);
//                 Serial.println(">>> Mode: MAZE_SOLVE");
//                 break;
//             default:
//                 Serial.println(">>> ERROR: Invalid mode (use 1 or 3)");
//         }
//     }
//     else if (cmd.startsWith("pid ")) {
//         int space1 = cmd.indexOf(' ', 4);
//         int space2 = cmd.indexOf(' ', space1 + 1);
        
//         if (space1 > 0 && space2 > 0) {
//             float kp = cmd.substring(4, space1).toFloat();
//             float ki = cmd.substring(space1 + 1, space2).toFloat();
//             float kd = cmd.substring(space2 + 1).toFloat();
            
//             robot.setPID(kp, ki, kd);
//             Serial.print(">>> PID set to: Kp=");
//             Serial.print(kp, 3);
//             Serial.print(" Ki=");
//             Serial.print(ki, 3);
//             Serial.print(" Kd=");
//             Serial.println(kd, 3);
//         }
//         else {
//             Serial.println(">>> Usage: pid <kp> <ki> <kd>");
//         }
//     }
//     else if (cmd.startsWith("threshold ")) {
//         int threshold = cmd.substring(10).toInt();
//         lineSensor.setThreshold(threshold);
//         Serial.print(">>> Threshold set to ");
//         Serial.println(threshold);
//     }
//     else if (cmd.startsWith("motors ")) {
//         int space = cmd.indexOf(' ', 7);
//         if (space > 0) {
//             int left = cmd.substring(7, space).toInt();
//             int right = cmd.substring(space + 1).toInt();
            
//             robot.stop();
//             leftMotor.setSpeed(left);
//             rightMotor.setSpeed(right);
            
//             Serial.print(">>> Motors: L=");
//             Serial.print(left);
//             Serial.print(" R=");
//             Serial.println(right);
//             Serial.println("    (Type 'stop' to resume robot control)");
//         }
//         else {
//             Serial.println(">>> Usage: motors <left> <right>");
//         }
//     }
//     else if (cmd.startsWith("left ")) {
//         int speed = cmd.substring(5).toInt();
//         robot.stop();
//         leftMotor.setSpeed(speed);
//         rightMotor.setSpeed(0);
//         Serial.print(">>> Left motor: ");
//         Serial.println(speed);
//     }
//     else if (cmd.startsWith("right ")) {
//         int speed = cmd.substring(6).toInt();
//         robot.stop();
//         leftMotor.setSpeed(0);
//         rightMotor.setSpeed(speed);
//         Serial.print(">>> Right motor: ");
//         Serial.println(speed);
//     }
//     else if (cmd.length() > 0) {
//         Serial.println(">>> Unknown command. Type 'help' for list.");
//     }
// }

// // ============================================================================
// // SETUP
// // ============================================================================

// void setup() {
//     Serial.begin(115200);
//     delay(2000);
    
//     Serial.println("\n\n");
//     Serial.println("========================================");
//     Serial.println(" LINE FOLLOWING ROBOT - SERIAL CONTROL");
//     Serial.println("   Raspberry Pi Pico W + L298N");
//     Serial.println("========================================");
//     Serial.println();
//     Serial.println("Sensor Configuration:");
//     Serial.println("  Digital Left  (IR1): Pin 21");
//     Serial.println("  Analog Left   (IR4): Pin 26");
//     Serial.println("  Analog Center (IR3): Pin 27");
//     Serial.println("  Analog Right  (IR2): Pin 28");
//     Serial.println("  Digital Right (IR5): Pin 22");
//     Serial.println();
    
//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, HIGH);
    
//     Serial.println("[OK] Motors configured");
//     Serial.println("[OK] Sensors configured");
    
//     // Blink LED to show ready
//     for (int i = 0; i < 3; i++) {
//         digitalWrite(LED_BUILTIN, LOW);
//         delay(100);
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(100);
//     }
//     digitalWrite(LED_BUILTIN, LOW);
    
//     delay(500);
    
//     Serial.println();
//     Serial.println("========================================");
//     Serial.println("   SETUP COMPLETE - READY!");
//     Serial.println("========================================");
//     Serial.println();
//     Serial.println("RECOMMENDED: Run 'calibrate' before starting");
//     Serial.println("Type 'help' for available commands");
//     Serial.println("Type 'start' to begin line following");
//     Serial.println();
//     Serial.print("> ");
// }

// // ============================================================================
// // MAIN LOOP
// // ============================================================================

// void loop() {
//     unsigned long currentTime = millis();
    
//     // ========================================================================
//     // CONTROL LOOP (20ms = 50 Hz)
//     // ========================================================================
    
//     if (currentTime - lastLoopTime >= CONTROL_LOOP_MS) {
//         lastLoopTime = currentTime;
//         robot.update();
//     }
    
//     // ========================================================================
//     // SERIAL COMMAND HANDLING
//     // ========================================================================
    
//     if (Serial.available()) {
//         String cmd = Serial.readStringUntil('\n');
//         cmd.trim();
        
//         if (cmd.length() > 0) {
//             Serial.println(cmd);
//             processCommand(cmd);
//             Serial.print("> ");
//         }
//     }
    
//     // ========================================================================
//     // STATUS OUTPUT (1 second)
//     // ========================================================================
    
//     if (currentTime - lastStatusTime >= 200) {
//         lastStatusTime = currentTime;
        
//         if (robot.getState() != STATE_IDLE) {
//             Serial.print("[");
//             switch(robot.getState()) {
//                 case STATE_IDLE: Serial.print("IDLE"); break;
//                 case STATE_LINE_FOLLOW: Serial.print("FOLLOW"); break;
//                 case STATE_SMALL_FORWARD: Serial.print("SMALL_FORWARD"); break;
//                 case STATE_TURN_LEFT: Serial.print("LEFT"); break;
//                 case STATE_TURN_RIGHT: Serial.print("RIGHT"); break;
//                 case STATE_TURN_AROUND: Serial.print("AROUND"); break;
//                 case STATE_LOST: Serial.print("LOST"); break;
//                 default: Serial.print("?"); break;
//             }
//             Serial.print("] L=");
//             Serial.print(leftMotor.getSpeed());
//             Serial.print(" R=");
//             Serial.println(rightMotor.getSpeed());
//         }
//     }
    
//     // ========================================================================
//     // SENSOR STREAM (200ms)
//     // ========================================================================
    
//     if (sensorStreamEnabled && currentTime - lastSensorPrintTime >= 200) {
//         lastSensorPrintTime = currentTime;
//         lineSensor.read();
//         lineSensor.printValues();
//     }
// }