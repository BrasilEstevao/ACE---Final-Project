// #include <Arduino.h>
// #include "config.h"
// #include "L298NMotor.h"
// #include "LineSensor.h"
// #include "RobotStateMachine.h"
// #include "WiFiTerminal.h"

// // ============================================================================
// // GLOBAL OBJECTS
// // ============================================================================

// L298NMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
// L298NMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);
// LineSensor lineSensor;
// RobotStateMachine robot(&leftMotor, &rightMotor, &lineSensor);
// WiFiTerminal terminal;

// // ============================================================================
// // TIMING VARIABLES
// // ============================================================================

// unsigned long lastLoopTime = 0;
// unsigned long lastStatusTime = 0;
// unsigned long lastSensorPrintTime = 0;

// // ============================================================================
// // COMMAND PROCESSING
// // ============================================================================

// void processCommand(String cmd) {
//     cmd.trim();
//     cmd.toLowerCase();
    
//     if (cmd == "help") {
//         terminal.println("\n=== Available Commands ===");
//         terminal.println("help       - Show this help");
//         terminal.println("start      - Start robot");
//         terminal.println("stop       - Stop robot");
//         terminal.println("status     - Show status");
//         terminal.println("sensors    - Show sensor values");
//         terminal.println("calibrate  - Calibrate sensors");
//         terminal.println("speed <n>  - Set speed (0-255)");
//         terminal.println("mode <n>   - Set mode (1=follow, 3=maze)");
//         terminal.println("pid <p> <i> <d> - Set PID gains");
//         terminal.println("threshold <n> - Set sensor threshold");
//         terminal.println("motors <l> <r> - Test motors (PWM -255 to 255)");
//         terminal.println("==========================\n");
//     }
//     else if (cmd == "start") {
//         robot.start();
//         terminal.println("Robot started");
//         Serial.println("CMD: start");
//     }
//     else if (cmd == "stop") {
//         robot.stop();
//         terminal.println("Robot stopped");
//         Serial.println("CMD: stop");
//     }
//     else if (cmd == "status") {
//         robot.printStatus();
//         terminal.print("WiFi clients: ");
//         terminal.println(terminal.getClientCount());
//     }
//     else if (cmd == "sensors") {
//         lineSensor.read();
        
//         terminal.print("S0=");
//         terminal.print(lineSensor.getSensorValue(0));
//         terminal.print(" S1=");
//         terminal.print(lineSensor.getSensorValue(1));
//         terminal.print(" S2=");
//         terminal.print(lineSensor.getSensorValue(2));
//         terminal.print(" S3=");
//         terminal.print(lineSensor.getSensorValue(3));
//         terminal.print(" S4=");
//         terminal.print(lineSensor.getSensorValue(4));
//         terminal.print(" | Pos: ");
//         terminal.print(lineSensor.getPosition());
//         terminal.print(" | Junction: ");
        
//         JunctionType j = lineSensor.detectJunction();
//         switch(j) {
//             case JUNCTION_NONE: terminal.println("NONE"); break;
//             case JUNCTION_LEFT: terminal.println("LEFT"); break;
//             case JUNCTION_RIGHT: terminal.println("RIGHT"); break;
//             case JUNCTION_T: terminal.println("T"); break;
//             case JUNCTION_CROSS: terminal.println("CROSS"); break;
//             case JUNCTION_LOST: terminal.println("LOST"); break;
//         }
//     }
//     else if (cmd == "calibrate") {
//         terminal.println("Calibrating sensors...");
//         terminal.println("Move robot over black and white");
//         robot.stop();
//         lineSensor.calibrate();
//         terminal.println("Calibration complete!");
//     }
//     else if (cmd.startsWith("speed ")) {
//         int speed = cmd.substring(6).toInt();
//         robot.setSpeed(speed);
//         terminal.print("Speed set to ");
//         terminal.println(speed);
//     }
//     else if (cmd.startsWith("mode ")) {
//         int mode = cmd.substring(5).toInt();
//         switch(mode) {
//             case 1:
//                 robot.setMode(MODE_LINE_FOLLOW);
//                 terminal.println("Mode: LINE_FOLLOW");
//                 break;
//             case 3:
//                 robot.setMode(MODE_MAZE_SOLVE);
//                 terminal.println("Mode: MAZE_SOLVE");
//                 break;
//             default:
//                 terminal.println("Invalid mode (1 or 3)");
//         }
//     }
//     else if (cmd.startsWith("pid ")) {
//         // Parse "pid kp ki kd"
//         int space1 = cmd.indexOf(' ', 4);
//         int space2 = cmd.indexOf(' ', space1 + 1);
        
//         if (space1 > 0 && space2 > 0) {
//             float kp = cmd.substring(4, space1).toFloat();
//             float ki = cmd.substring(space1 + 1, space2).toFloat();
//             float kd = cmd.substring(space2 + 1).toFloat();
            
//             robot.setPID(kp, ki, kd);
//             terminal.print("PID set to: Kp=");
//             terminal.print(kp);
//             terminal.print(" Ki=");
//             terminal.print(ki);
//             terminal.print(" Kd=");
//             terminal.println(kd);
//         }
//         else {
//             terminal.println("Usage: pid <kp> <ki> <kd>");
//         }
//     }
//     else if (cmd.startsWith("threshold ")) {
//         int threshold = cmd.substring(10).toInt();
//         lineSensor.setThreshold(threshold);
//         terminal.print("Threshold set to ");
//         terminal.println(threshold);
//     }
//     else if (cmd.startsWith("motors ")) {
//         // Parse "motors left right"
//         int space = cmd.indexOf(' ', 7);
//         if (space > 0) {
//             int left = cmd.substring(7, space).toInt();
//             int right = cmd.substring(space + 1).toInt();
            
//             robot.stop(); // Stop state machine
//             leftMotor.setSpeed(left);
//             rightMotor.setSpeed(right);
            
//             terminal.print("Motors: L=");
//             terminal.print(left);
//             terminal.print(" R=");
//             terminal.println(right);
//             terminal.println("(Type 'stop' to resume control)");
//         }
//         else {
//             terminal.println("Usage: motors <left> <right>");
//         }
//     }
//     else if (cmd.length() > 0) {
//         terminal.println("Unknown command. Type 'help'");
//     }
// }

// // ============================================================================
// // SETUP
// // ============================================================================

// void setup() {
//     // Serial for debugging
//     Serial.begin(115200);
//     delay(2000);
    
//     Serial.println("\n========================================");
//     Serial.println("Line Following Robot - WiFi Control");
//     Serial.println("Raspberry Pi Pico W + L298N");
//     Serial.println("========================================\n");
    
//     // Motors already initialized
//     Serial.println("✓ Motors configured");
    
//     // Sensors already initialized
//     Serial.println("✓ Sensors configured");
    
//     // Initialize WiFi terminal
//     if (terminal.begin(WIFI_SSID, WIFI_PASSWORD)) {
//         Serial.println("✓ WiFi terminal ready");
//         Serial.print("✓ Connect to WiFi: ");
//         Serial.println(WIFI_SSID);
//         Serial.print("✓ Then telnet to: ");
//         Serial.println(terminal.getIP());
//     }
//     else {
//         Serial.println("✗ WiFi terminal failed!");
//     }
    
//     delay(500);
    
//     Serial.println("\n========================================");
//     Serial.println("Setup complete!");
//     Serial.println("Connect via WiFi and send 'help' command");
//     Serial.println("========================================\n");
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
//     // WIFI TERMINAL HANDLING
//     // ========================================================================
    
//     terminal.update();
    
//     // Process incoming commands
//     if (terminal.available()) {
//         String cmd = terminal.readLine();
//         if (cmd.length() > 0) {
//             Serial.print(">> ");
//             Serial.println(cmd);
//             processCommand(cmd);
//         }
//     }
    
//     // ========================================================================
//     // STATUS OUTPUT TO SERIAL (1 second)
//     // ========================================================================
    
//     if (currentTime - lastStatusTime >= 1000) {
//         lastStatusTime = currentTime;
        
//         Serial.print("State: ");
//         switch(robot.getState()) {
//             case STATE_IDLE: Serial.print("IDLE"); break;
//             case STATE_LINE_FOLLOW: Serial.print("FOLLOW"); break;
//             case STATE_SMALL_FORWARD: Serial.print("JUNCTION"); break;
//             case STATE_TURN_LEFT: Serial.print("LEFT"); break;
//             case STATE_TURN_RIGHT: Serial.print("RIGHT"); break;
//             case STATE_TURN_AROUND: Serial.print("AROUND"); break;
//             case STATE_LOST: Serial.print("LOST"); break;
//             default: Serial.print("?"); break;
//         }
        
//         Serial.print(" | L=");
//         Serial.print(leftMotor.getSpeed());
//         Serial.print(" R=");
//         Serial.print(rightMotor.getSpeed());
//         Serial.print(" | Clients=");
//         Serial.println(terminal.getClientCount());
//     }
    
//     // ========================================================================
//     // SENSOR OUTPUT TO TERMINAL (200ms) - only if connected
//     // ========================================================================
    
//     if (terminal.isConnected() && currentTime - lastSensorPrintTime >= 200) {
//         lastSensorPrintTime = currentTime;
        
//         lineSensor.read();
        
//         terminal.print("S: ");
//         for (int i = 0; i < 5; i++) {
//             terminal.print(lineSensor.getSensorValue(i));
//             terminal.print(" ");
//         }
//         terminal.print("| Pos: ");
//         terminal.print(lineSensor.getPosition());
//         terminal.print(" | ");
        
//         JunctionType j = lineSensor.detectJunction();
//         switch(j) {
//             case JUNCTION_NONE: terminal.println("LINE"); break;
//             case JUNCTION_LEFT: terminal.println("LEFT"); break;
//             case JUNCTION_RIGHT: terminal.println("RIGHT"); break;
//             case JUNCTION_T: terminal.println("T"); break;
//             case JUNCTION_CROSS: terminal.println("CROSS"); break;
//             case JUNCTION_LOST: terminal.println("LOST"); break;
//         }
//     }
// }