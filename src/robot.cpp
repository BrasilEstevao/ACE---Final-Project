#include <Arduino.h>
#include "robot.h"
#include "config.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

robot_t::robot_t()
{
  
  wheel_dist = WHEEL_DISTANCE;
  wheel_radius = WHEEL_RADIUS;
  dv_max = 5;
  dw_max = 10;
  dt = CONTROL_LOOP_S;
  
  // Initialize odometry state
  enc1 = 0;
  enc2 = 0;
  Senc1 = 0;
  Senc2 = 0;
  w1e = 0;
  w2e = 0;
  v1e = 0;
  v2e = 0;
  ve = 0;
  we = 0;
  ds = 0;
  dtheta = 0;
  xe = 0;
  ye = 0;
  thetae = 0;
  rel_s = 0;
  rel_theta = 0;
  
  // Initialize control
  v = 0;
  w = 0;
  v_req = 0;
  w_req = 0;
  v1ref = 0;
  v2ref = 0;
  w1ref = 0;
  w2ref = 0;
  u1 = 0;
  u2 = 0;
  PWM_1 = 0;
  PWM_2 = 0;
  PWM_1_req = 0;
  PWM_2_req = 0;
  
  control_mode = cm_pwm;
  battery_voltage = 7.4;
  
  // Initialize PIDs 
  PID1.dt = dt;
  PID2.dt = dt;
  
  // Line following 
  line_sensor = NULL;
  line_kp = LINE_KP;
  line_ki = LINE_KI;
  line_kd = LINE_KD;
  line_last_error = 0;
  line_integral = 0;
  base_speed = BASE_SPEED;
  
  // Goto targets (NEW)
  target_distance = 0;
  target_angle = 0;
}

// ============================================================================
// ODOMETRY 
// ============================================================================

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose (midpoint integration)
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement (for maneuvers)
  rel_s += ds;
  rel_theta += dtheta;
}

// ============================================================================
// SET VELOCITY
// ============================================================================

void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}

// ============================================================================
// ACCELERATION LIMIT 
// ============================================================================

void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}

// ============================================================================
// CONVERT V,W TO MOTOR PWM 
// ============================================================================

void robot_t::VWToMotorsVoltage(void)
{
  // Differential drive inverse kinematics
  v1ref = v + w * wheel_dist / 2;
  v2ref = v - w * wheel_dist / 2; 
  
  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  switch (control_mode) {
    case cm_pwm:
        // Direct PWM control 
        PWM_1 = PWM_1_req;  
        PWM_2 = PWM_2_req;  
        break;

    case cm_pid:
        // PID velocity control 
        u1 = 0;
        u2 = 0;      

        if (v1ref != 0) u1 = PID1.calc(v1ref, v1e);
        else PID1.Se = 0;

        if (v2ref != 0) u2 = PID2.calc(v2ref, v2e);
        else PID2.Se = 0;

        PWM_1 = u1 / battery_voltage * 255;
        PWM_2 = u2 / battery_voltage * 255;
        break;

    case cm_line_follow:
        // Line following control 
        lineFollowControl();
        break;

    case cm_goto_distance:
        // Navigate distance 
        gotoDistanceControl();
        break;

    case cm_goto_angle:
        // Navigate angle 
        gotoAngleControl();
        break;

    default:
        PWM_1 = 0;
        PWM_2 = 0;
        break;
  }
}

// ============================================================================
// LINE FOLLOWING CONTROL 
// ============================================================================

void robot_t::lineFollowControl()
{
  if (!line_sensor) {
    PWM_1 = 0;
    PWM_2 = 0;
    return;
  }
  
  int position = line_sensor->getPosition();
  int error = position;
  
  // PID calculation
  line_integral += error;
  line_integral = constrain(line_integral, -10000, 10000);
  
  int derivative = error - line_last_error;
  line_last_error = error;
  
  float correction = line_kp * error + line_ki * line_integral + line_kd * derivative;
  
  // Apply differential steering
  int leftSpeed = base_speed - correction;
  int rightSpeed = base_speed + correction;
  
  PWM_1 = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  PWM_2 = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
}

// ============================================================================
// GOTO DISTANCE CONTROL 
// ============================================================================

void robot_t::gotoDistanceControl()
{
  float traveled = fabs(rel_s);
  float remaining = fabs(target_distance) - traveled;
  
  // Check if complete
  if (remaining <= 0.02f) {
    control_mode = cm_pwm;
    v_req = 0;
    w_req = 0;
    PWM_1 = 0;
    PWM_2 = 0;
    Serial.println("[GOTO] Distance complete");
    return;
  }
  
  // Calculate speed with deceleration
  float speed = 0.30f;  // m/s
  if (remaining < 0.10f) {
    speed = 0.05f + (remaining / 0.10f) * 0.10f;
  }
  
  if (target_distance < 0) speed = -speed;
  
  
  v_req = speed;
  w_req = 0.0f;

  v = v_req;
  w = w_req;
  
  control_mode = cm_pid;  // Switch to PID for execution
  VWToMotorsVoltage();    // Recursive call to execute PID
  control_mode = cm_goto_distance;  // Restore mode
}

// ============================================================================
// GOTO ANGLE CONTROL 
// ============================================================================

void robot_t::gotoAngleControl()
{
  float turned = fabs(rel_theta);
  float remaining = fabs(target_angle) - turned;
  
  // Check if complete
  if (remaining <= 0.05f) {
    control_mode = cm_pwm;
    v_req = 0;
    w_req = 0;
    PWM_1 = 0;
    PWM_2 = 0;
    Serial.println("[GOTO] Angle complete");
    return;
  }
  
  // Calculate angular speed with deceleration
  float angSpeed = 0.8f;  // rad/s
  if (remaining < 0.35f) {
    angSpeed = 0.3f + (remaining / 0.35f) * 0.5f;
  }
  
  if (target_angle < 0) angSpeed = -angSpeed;
  
 
  v_req = 0.0f;
  w_req = angSpeed;

  v = v_req; 
  w = w_req;
  
  control_mode = cm_pid;  // Switch to PID for execution
  VWToMotorsVoltage();    // Recursive call to execute PID
  control_mode = cm_goto_angle;  // Restore mode
}

// ============================================================================
// SETTERS 
// ============================================================================

void robot_t::setLineSensor(LineSensor* sensor)
{
  line_sensor = sensor;
}

void robot_t::setGotoDistance(float distance)
{
  target_distance = distance;
  resetRelative();
  control_mode = cm_goto_distance;
  
  Serial.print("[GOTO] Distance: ");
  Serial.print(distance * 100.0f, 1);
  Serial.println(" cm");
}

void robot_t::setGotoAngle(float angle)
{
  target_angle = angle;
  resetRelative();
  control_mode = cm_goto_angle;
  
  Serial.print("[GOTO] Angle: ");
  Serial.print(angle * 180.0f / PI, 1);
  Serial.println("°");
}

void robot_t::setLinePID(float kp, float ki, float kd)
{
  line_kp = kp;
  line_ki = ki;
  line_kd = kd;
}

void robot_t::setBaseSpeed(int speed)
{
  base_speed = constrain(speed, 0, 255);
}

void robot_t::resetRelative()
{
  rel_s = 0;
  rel_theta = 0;
}