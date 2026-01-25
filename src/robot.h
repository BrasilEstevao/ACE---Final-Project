#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "LineSensor.h"

// Control modes 
typedef enum { 
  cm_pwm,         
  cm_pid,          
  cm_line_follow,  
  cm_goto_distance,
  cm_goto_angle    
} control_mode_t;

class robot_t {
public:
  // ========================================================================
  // SENSORS (
  // ========================================================================
  int enc1, enc2;              
  int Senc1, Senc2;            
  LineSensor* line_sensor;     
  
  // ========================================================================
  // ODOMETRY STATE 
  // ========================================================================
  float w1e, w2e;              // Wheel angular velocities (rad/s)
  float v1e, v2e;              // Wheel linear velocities (m/s)
  float ve, we;                // Robot velocities (m/s, rad/s)
  float ds, dtheta;            // Incremental displacement
  float rel_s, rel_theta;      // Relative displacement (for maneuvers)
  float xe, ye, thetae;        // Estimated pose (x, y, theta)
  
  // ========================================================================
  // CONTROL 
  // ========================================================================
  float dt;                    // Control loop period (s)
  float v, w;                  // Current velocities (after limits)
  float v_req, w_req;          // Requested velocities
  float dv_max, dw_max;        // Max acceleration
  
  // ========================================================================
  // ROBOT PARAMETERS 
  // ========================================================================
  float wheel_radius, wheel_dist;
  
  // ========================================================================
  // MOTOR CONTROL
  // ========================================================================
  float v1ref, v2ref;          // Reference wheel velocities
  float w1ref, w2ref;          // Reference wheel angular velocities
  float u1, u2;                // Control voltages (V)
  int PWM_1, PWM_2;            // Motor PWM outputs
  int PWM_1_req, PWM_2_req;    // Requested PWM (cm_pwm mode)
  control_mode_t control_mode;
  
  // ========================================================================
  // PID CONTROLLERS 
  // ========================================================================
  PID_t PID1, PID2;            // Motor velocity PIDs
  float battery_voltage;
  
  // ========================================================================
  // LINE FOLLOWING 
  // ========================================================================
  float line_kp, line_ki, line_kd;
  int line_last_error;
  int line_integral;
  int base_speed;
  
  // ========================================================================
  // GOTO TARGETS 
  // ========================================================================
  float target_distance;
  float target_angle;
  
  // ========================================================================
  // CONSTRUCTOR
  // ========================================================================
  robot_t();
  
  // ========================================================================
  // MAIN METHODS 
  // ========================================================================
  void odometry(void);                          // Update odometry
  void setRobotVW(float Vnom, float Wnom);     // Set velocity setpoint
  void accelerationLimit(void);                 // Apply acceleration limits
  void VWToMotorsVoltage(void);                // Convert v,w to motor PWM
  
  // ========================================================================
  // NEW METHODS 
  // ========================================================================
  void setLineSensor(LineSensor* sensor);      // Attach line sensor
  void lineFollowControl();                     // Line following PID
  void gotoDistanceControl();                   // Navigate to distance
  void gotoAngleControl();                      // Navigate to angle
  
  void setGotoDistance(float distance);
  void setGotoAngle(float angle);
  void setLinePID(float kp, float ki, float kd);
  void setBaseSpeed(int speed);
  
  void resetRelative();                         // Reset rel_s, rel_theta
};

#endif // ROBOT_H