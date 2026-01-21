#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>
#include "config.h"
#include "robot.h"
#include "LineSensor.h"
#include "follow_mode.h"
#include "maze_solver.h"
#include "commands.h"
#include "WiFiTerminal.h"
#include <HCSR04.h>
#include "Sonar.h"

// ============================================================================
// TIMER INTERRUPT
// ============================================================================

RPI_PICO_Timer ITimer1(1);

#define TEST_PIN 27

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

int enc1, enc2;
int encoder1_state, encoder2_state;

int encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; 

volatile int count;
int act_count;

#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)
#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)

// ============================================================================
// TIMER INTERRUPT HANDLER 
// ============================================================================

bool timer_handler(struct repeating_timer *t)
{
  int next_state, table_input, pins;
  digitalWriteFast(TEST_PIN, 1);

  pins = sio_hw->gpio_in;

  next_state = pinIsHigh(ENC1_A, pins) << 1;
  next_state |= pinIsHigh(ENC1_B, pins);

  table_input = (encoder1_state << 2) | next_state;
  encoder1_pos += encoder_table[table_input];
  encoder1_state = next_state;

  next_state = pinIsHigh(ENC2_A, pins) << 1;
  next_state |= pinIsHigh(ENC2_B, pins);
  
  table_input = (encoder2_state << 2) | next_state;
  encoder2_pos -= encoder_table[table_input];
  encoder2_state = next_state;

  count++;
  digitalWriteFast(TEST_PIN, 0);
  return true;
}

// ============================================================================
// READ ENCODERS 
// ============================================================================

void read_encoders(void)
{
  noInterrupts();
  
  enc1 = encoder1_pos;
  enc2 = encoder2_pos;
  act_count = count;

  encoder1_pos = 0;
  encoder2_pos = 0;
  count = 0;

  interrupts();
}

// ============================================================================
// PWM MOTOR CONTROL 
// ============================================================================

void setMotorPWM(int new_PWM, int pin_a, int pin_b, int pin_en)
{
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) {
    digitalWrite(pin_a, LOW);
    digitalWrite(pin_b, LOW);
    analogWrite(pin_en, 0);
  } else if (new_PWM > 0) {
    digitalWrite(pin_a, HIGH);
    digitalWrite(pin_b, LOW);
    analogWrite(pin_en, abs(new_PWM));
  } else {
    digitalWrite(pin_a, LOW);
    digitalWrite(pin_b, HIGH);
    analogWrite(pin_en, abs(new_PWM));
  }
}

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

robot_t robot;
LineSensor lineSensor;
FollowMode followMode;
Sonar Sonar1;
MazeSolver Mazesolver;
commands_t serial_commands;
WiFiTerminal terminal;

// ============================================================================
// GLOBAL VARIABLES FOR DEBUG
// ============================================================================

unsigned long last_maze_update = 0;
unsigned long last_follow_update = 0;
MazeState last_printed_maze_state = MAZE_IDLE;
FollowState last_printed_follow_state = FOLLOW_IDLE;

// ============================================================================
// TIMING 
// ============================================================================

unsigned long interval, last_cycle;
unsigned long loop_micros;

// ============================================================================
// MODE FLAGS
// ============================================================================

bool follow_mode_active = false;
bool maze_mode_active = false;
bool sensor_stream = false;
bool odom_stream = false;

// ============================================================================
// DUAL OUTPUT (Serial + WiFi)
// ============================================================================

class DualOutput {
public:
  void print(const char* s) { Serial.print(s); terminal.print(s); }
  void println(const char* s) { Serial.println(s); terminal.println(s); }
  void print(String s) { Serial.print(s); terminal.print(s); }
  void println(String s) { Serial.println(s); terminal.println(s); }
  void print(unsigned long v) { 
    Serial.print(v); 
    terminal.print((int)v);
  }
  void println(unsigned long v) { 
    Serial.println(v); 
    terminal.println((int)v); 
  }
  void print(int v) { Serial.print(v); terminal.print(v); }
  void println(int v) { Serial.println(v); terminal.println(v); }
  void print(float v, int d=2) { Serial.print(v,d); terminal.print(v,d); }
  void println(float v, int d=2) { Serial.println(v,d); terminal.println(v,d); }
  void println() { Serial.println(); terminal.println(); }
} out;

// ============================================================================
// DEBUG PRINT FUNCTIONS
// ============================================================================

void printMazeStatus()
{
  char posBuf[64];
  Mazesolver.getPositionString(posBuf, sizeof(posBuf));
  
  out.println("\n=== MAZE SOLVER - LEFT-HAND ===");
  out.print("State: ");
  out.println(Mazesolver.getStateName());
  out.print("Position: ");
  out.println(posBuf);
  out.print("Time: ");
  out.print(Mazesolver.getTimeInState());
  out.println(" ms");
  
  if (Mazesolver.stored_junction != JUNCTION_NONE) {
    out.print("Last junction: ");
    switch (Mazesolver.stored_junction) {
      case JUNCTION_LEFT: out.println("LEFT"); break;
      case JUNCTION_RIGHT: out.println("RIGHT"); break;
      case JUNCTION_T: out.println("T-JUNCTION"); break;
      default: break;
    }
  }
  
  out.println("================================\n");
}

void printFollowStatus()
{
  out.println("\n=== FOLLOW MODE - U-TURN ===");
  out.print("State: ");
  
  switch (followMode.getState()) {
    case FOLLOW_IDLE: out.println("IDLE"); break;
    case FOLLOW_LINE: out.println("FOLLOWING"); break;
    case FOLLOW_LOST: out.println("LOST"); break;
    case FOLLOW_BLOCKED: out.println("BLOCKED"); break;
  }
  
  out.print("Time: ");
  out.print(followMode.getTimeInState());
  out.println(" ms");
  out.println("=============================\n");
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void process_command(frame_data_t frame)
{
  if (frame.command_is("help")) {
    out.println("\n=== ROBOT COMMANDS ===");
    out.println("Basic:");
    out.println("  stop          - Stop robot");
    out.println("  status        - Show status");
    out.println("");
    out.println("Line Following:");
    out.println("  follow        - Start line following");
    out.println("  linepid <p> <i> <d>");
    out.println("  speed <n>     - Base speed");
    out.println("");
    out.println("Maze:");
    out.println("  maze          - Start maze solving");
    out.println("  mazestop      - Stop maze");
    out.println("");
    out.println("Navigation:");
    out.println("  forward <m>   - Move distance");
    out.println("  turn <deg>    - Turn angle");
    out.println("  vel <v> <w>   - Set velocity");
    out.println("");
    out.println("Velocity PID:");
    out.println("  kf <n>        - Set Kf");
    out.println("  kp <n>        - Set Kp");
    out.println("  ki <n>        - Set Ki");
    out.println("");
    out.println("Direct Control:");
    out.println("  m1 <pwm>      - Motor 1 PWM");
    out.println("  m2 <pwm>      - Motor 2 PWM");
    out.println("  mo <n>        - Mode (0=pwm, 1=pid)");
    out.println("");
    out.println("Sensors:");
    out.println("  sensors       - Show sensors");
    out.println("  stream <on|off>");
    out.println("  calibrate");
    out.println("");
    out.println("Odometry:");
    out.println("  odom          - Show odometry");
    out.println("  ostream <on|off>");
    out.println("  resetodom");
    out.println("===================\n");
  }
  
  else if (frame.command_is("stop")) {
    maze_mode_active = false;
    follow_mode_active = false;
    robot.control_mode = cm_pwm;
    robot.v_req = 0;
    robot.w_req = 0;
    robot.PWM_1 = 0;
    robot.PWM_2 = 0;
    followMode.stop();
    Mazesolver.stop();
    out.println(">>> STOPPED");
  }
  
  else if (frame.command_is("status")) {
    out.println("\n--- Status ---");
    out.print("Mode: ");
    switch (robot.control_mode) {
      case cm_pwm: out.println("PWM"); break;
      case cm_pid: out.println("PID"); break;
      case cm_line_follow: out.println("LINE_FOLLOW"); break;
      case cm_goto_distance: out.println("GOTO_DIST"); break;
      case cm_goto_angle: out.println("GOTO_ANGLE"); break;
    }
    
    if (maze_mode_active) {
      out.print("Maze: ");
      switch (Mazesolver.getState()) {
        case MAZE_IDLE: out.println("IDLE"); break;
        case MAZE_FOLLOWING: out.println("FOLLOWING"); break;
        case MAZE_SMALL_FORWARD: out.println("SMALL_FWD"); break;
        case MAZE_TURNING_LEFT: out.println("TURN_LEFT"); break;
        case MAZE_TURNING_RIGHT: out.println("TURN_RIGHT"); break;
        case MAZE_TURNING_AROUND: out.println("U_TURN"); break;
        case MAZE_FINISHED: out.println("FINISHED"); break;
        default: break;
      }
    }
    
    out.print("PWM: L=");
    out.print(robot.PWM_1);
    out.print(" R=");
    out.println(robot.PWM_2);
    
    out.print("Pose: x=");
    out.print(robot.xe, 3);
    out.print("m y=");
    out.print(robot.ye, 3);
    out.print("m θ=");
    out.print(robot.thetae * 180.0f / PI, 1);
    out.println("°");
    
    out.print("Velocity: v=");
    out.print(robot.ve, 3);
    out.print("m/s w=");
    out.print(robot.we, 2);
    out.println("rad/s");
    
    out.println("--------------\n");
  }
  
  else if (frame.command_is("follow")) {
    follow_mode_active = true;
    maze_mode_active = false;
    robot.xe = 0;
    robot.ye = 0;
    robot.thetae = 0;
    robot.rel_s = 0;
    robot.rel_theta = 0;
    followMode.start();
    out.println(">>> LINE FOLLOWING started");
  }
  
  else if (frame.command_is("linepid")) {
    float kp = 0, ki = 0, kd = 0;
    sscanf(frame.text, "%f %f %f", &kp, &ki, &kd);
    robot.setLinePID(kp, ki, kd);
    out.print(">>> Line PID: Kp=");
    out.print(kp, 2);
    out.print(" Ki=");
    out.print(ki, 2);
    out.print(" Kd=");
    out.println(kd, 2);
  }
  
  else if (frame.command_is("speed")) {
    robot.setBaseSpeed((int)frame.value);
    out.print(">>> Speed: ");
    out.println((int)frame.value);
  }
  
  else if (frame.command_is("maze")) {
    maze_mode_active = true;
    follow_mode_active = false;
    robot.xe = 0;
    robot.ye = 0;
    robot.thetae = 0;
    robot.rel_s = 0;
    robot.rel_theta = 0;
    Mazesolver.start();
    out.println(">>> MAZE SOLVING started");
  }
  
  else if (frame.command_is("mazestop")) {
    maze_mode_active = false;
    Mazesolver.stop();
    robot.control_mode = cm_pwm;
    robot.PWM_1 = 0;
    robot.PWM_2 = 0;
    out.println(">>> MAZE stopped");
  }
  
  else if (frame.command_is("forward")) {
    robot.setGotoDistance(frame.value);
    out.print(">>> Moving ");
    out.print(frame.value * 100.0f, 1);
    out.println(" cm");
  }
  
  else if (frame.command_is("turn")) {
    float radians = frame.value * PI / 180.0f;
    robot.setGotoAngle(radians);
    out.print(">>> Turning ");
    out.print(frame.value, 1);
    out.println("°");
  }
  
  else if (frame.command_is("vel")) {
    float v = 0, w = 0;
    sscanf(frame.text, "%f %f", &v, &w);
    robot.setRobotVW(v, w);
    robot.control_mode = cm_pid;
    out.print(">>> Velocity: v=");
    out.print(v, 3);
    out.print(" w=");
    out.println(w, 3);
  }
  
  else if (frame.command_is("m1")) {
    robot.PWM_1_req = frame.value;
    robot.control_mode = cm_pwm;
  }
  
  else if (frame.command_is("m2")) {
    robot.PWM_2_req = frame.value;
    robot.control_mode = cm_pwm;
  }
  
  else if (frame.command_is("mo")) {
    robot.control_mode = (control_mode_t) frame.value;
    out.print(">>> Mode: ");
    out.println((int)frame.value);
  }
  
  else if (frame.command_is("v")) {
    robot.v_req = frame.value;
    robot.control_mode = cm_pid;
  }
  
  else if (frame.command_is("w")) {
    robot.w_req = frame.value;
    robot.control_mode = cm_pid;
  }
  
  else if (frame.command_is("kf")) {
    robot.PID1.Kf = frame.value;
    robot.PID2.Kf = frame.value;
    out.print(">>> Kf: ");
    out.println(frame.value, 2);
  }
  
  else if (frame.command_is("kp")) {
    robot.PID1.Kp = frame.value;
    robot.PID2.Kp = frame.value;
    out.print(">>> Kp: ");
    out.println(frame.value, 2);
  }
  
  else if (frame.command_is("ki")) {
    robot.PID1.Ki = frame.value;
    robot.PID2.Ki = frame.value;
    out.print(">>> Ki: ");
    out.println(frame.value, 2);
  }
  
  else if (frame.command_is("sensors")) {
    lineSensor.printValues();
  }
  
  else if (frame.command_is("stream")) {
    if (strstr(frame.text, "on")) {
      sensor_stream = true;
      out.println(">>> Sensor stream ON");
    } else {
      sensor_stream = false;
      out.println(">>> Sensor stream OFF");
    }
  }
  
  else if (frame.command_is("calibrate")) {
    out.println(">>> CALIBRATING...");
    robot.control_mode = cm_pwm;
    robot.PWM_1 = 0;
    robot.PWM_2 = 0;
    delay(1000);
    lineSensor.calibrate();
    out.println(">>> Complete");
  }
  
  else if (frame.command_is("odom")) {
    out.print("x=");
    out.print(robot.xe, 3);
    out.print("m y=");
    out.print(robot.ye, 3);
    out.print("m θ=");
    out.print(robot.thetae * 180.0f / PI, 1);
    out.print("° | v=");
    out.print(robot.ve, 3);
    out.print("m/s w=");
    out.print(robot.we, 2);
    out.println("rad/s");
  }
  
  else if (frame.command_is("ostream")) {
    if (strstr(frame.text, "on")) {
      odom_stream = true;
      out.println(">>> Odom stream ON");
    } else {
      odom_stream = false;
      out.println(">>> Odom stream OFF");
    }
  }
  
  else if (frame.command_is("resetodom")) {
    robot.xe = 0;
    robot.ye = 0;
    robot.thetae = 0;
    robot.rel_s = 0;
    robot.rel_theta = 0;
    out.println(">>> Odometry reset");
  }
  
  out.print("> ");
}

// ============================================================================
// SETUP 
// ============================================================================

void setup() 
{
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(TEST_PIN, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);

  serial_commands.init(process_command);

  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println(" LINE ROBOT - LEFT-HAND RULE");
  Serial.println("========================================\n");

  if (ITimer1.attachInterrupt(40000, timer_handler))
    Serial.println("[OK] Timer interrupt started");
  else
    Serial.println("[ERROR] Timer failed");

  robot.setLineSensor(&lineSensor);

  // Sonar
  HCSR04.begin(SONAR_TRIG, SONAR_ECHO);
  Sonar1.setTerminal(&terminal);

  Mazesolver.setTerminal(&terminal);
  
  if (terminal.begin(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println("[OK] WiFi ready");
    Serial.print("IP: ");
    Serial.println(terminal.getIP());
  }

  interval = CONTROL_LOOP_MS;
  robot.dt = interval / 1000.0f;
  robot.PID1.dt = robot.dt;
  robot.PID2.dt = robot.dt;

  Serial.println("\n========================================");
  Serial.println(" READY! Type 'help'");
  Serial.println("========================================\n");
  Serial.print("> ");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() 
{
  unsigned long now = millis();
  
  terminal.update();
  
  if (Serial.available()) {
    uint8_t b = Serial.read();
    serial_commands.process_char(b);
  }
  
  String wifi_cmd = terminal.readLine();
  if (wifi_cmd.length() > 0) {
    frame_data_t frame;
    char cmd_buf[128];
    wifi_cmd.toCharArray(cmd_buf, 128);
    
    frame.command = cmd_buf;
    frame.text = cmd_buf;
    
    char* space = strchr(cmd_buf, ' ');
    if (space) {
      *space = 0;
      frame.text = space + 1;
      frame.value = atof(frame.text);
    } else {
      frame.text = cmd_buf + strlen(cmd_buf);
      frame.value = 0;
    }
    
    process_command(frame);
  }
  
  // ========================================================================
  // CONTROL LOOP (40ms = 25Hz)
  // ========================================================================
  if (now - last_cycle > interval) {
    loop_micros = micros();
    last_cycle += interval;

    read_encoders();
    lineSensor.read();
    
    robot.enc1 = enc1;
    robot.enc2 = enc2;
    
    robot.odometry();
    robot.battery_voltage = 7.4;

    // Distance sensor
    Sonar1.update();    
    
    // ======================================================================
    // FOLLOW MODE
    // ======================================================================
    if (follow_mode_active) {
      JunctionType junction = lineSensor.detectJunction();
      FollowState prev_follow_state = followMode.getState();
      
      followMode.update(junction, robot.rel_theta, robot.rel_s);
      
      // Print status on state change or every 2 seconds
      if (followMode.getState() != last_printed_follow_state || 
          now - last_follow_update > 2000) {
        printFollowStatus();
        last_printed_follow_state = followMode.getState();
        last_follow_update = now;
      }
      
      // Reset relative angle when entering U-turn
      if (prev_follow_state != FOLLOW_TURN_AROUND && 
          followMode.getState() == FOLLOW_TURN_AROUND) {
        robot.resetRelative();
        out.println("[FOLLOW] Starting U-turn...");
      }
      
      if (followMode.shouldFollowLine()) {
        robot.control_mode = cm_line_follow;
      }
      else if (followMode.shouldSpiral()) {
        out.println("[FOLLOW] Spiraling");
        robot.setRobotVW(-0.05f, 0.0f);  // Spiral out
        robot.control_mode = cm_pid;
      }
    }

    // ======================================================================
    // MAZE SOLVER - LEFT-HAND RULE
    // ======================================================================
    if (maze_mode_active) {
      JunctionType junction = lineSensor.detectJunction();
      MazeState prev_state = Mazesolver.getState();
      
      // Update maze solver
      Mazesolver.update(junction, robot.rel_theta, robot.rel_s);
      
      if (prev_state != MAZE_SMALL_FORWARD && 
          Mazesolver.getState() == MAZE_SMALL_FORWARD) {
        robot.resetRelative();
        out.println("[MAZE] Reset rel_s for small forward");
      }
      
      // Apply outputs
      if (Mazesolver.shouldFollowLine()) {
        robot.control_mode = cm_line_follow;
      }
      else if (Mazesolver.shouldGoForward()) {
        robot.setRobotVW(0.1f, 0.0f);
        robot.control_mode = cm_pid;
      }
      else if (Mazesolver.shouldTurnLeft()) {
        robot.setGotoAngle(-TURN_90_ANGLE);
      }
      else if (Mazesolver.shouldTurnRight()) {
        robot.setGotoAngle(TURN_90_ANGLE);
      }
      else if (Mazesolver.shouldTurnAround()) {
        robot.setGotoAngle(TURN_180_ANGLE);
      }
      else if (Mazesolver.isFinished()) {
        robot.control_mode = cm_pwm;
        robot.PWM_1 = 0;
        robot.PWM_2 = 0;
        maze_mode_active = false;
        out.println("\n>>> MAZE SOLVED!");
      }
    } 

    // ======================================================================
    // MOTOR CONTROL LOGIC
    // ======================================================================
    robot.accelerationLimit();  
    if (robot.control_mode == cm_goto_distance) {
        robot.gotoDistanceControl(); 
    }
    else if (robot.control_mode == cm_goto_angle) {
        robot.gotoAngleControl();
    }
    else if (robot.control_mode == cm_line_follow) {
        robot.lineFollowControl();
    }
    else {
        robot.v = robot.v_req;
        robot.w = robot.w_req;
        robot.VWToMotorsVoltage();
    }

    // ======================================================================
    // APPLY TO MOTORS
    // ======================================================================
    setMotorPWM(robot.PWM_1, MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN);
    setMotorPWM(robot.PWM_2, MOTOR2_IN3, MOTOR2_IN4, MOTOR2_EN);

    // ======================================================================
    // STREAMING
    // ======================================================================
    if (sensor_stream) {
      out.print("S: ");
      out.print(lineSensor.getDigitalSensorValue(0) ? "B" : "W");
      out.print("|");
      out.print(lineSensor.getAnalogSensorValue(0));
      out.print(",");
      out.print(lineSensor.getAnalogSensorValue(1));
      out.print(",");
      out.print(lineSensor.getAnalogSensorValue(2));
      out.print("|");
      out.print(lineSensor.getDigitalSensorValue(1) ? "B" : "W");
      out.print(" P:");
      out.println(lineSensor.getPosition());
    }
    
    if (odom_stream) {
      out.print("O: x=");
      out.print(robot.xe, 2);
      out.print(" y=");
      out.print(robot.ye, 2);
      out.print(" θ=");
      out.print(robot.thetae * 180.0f / PI, 0);
      out.print("° v=");
      out.print(robot.ve, 2);
      out.print(" w=");
      out.println(robot.we, 1);
    }
  } 
} 