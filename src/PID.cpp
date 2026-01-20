#include "Arduino.h"
#include "PID.h"

PID_t::PID_t()
{
  dt = 0.04;
  Kp = 8.7;
  Ki = 60;
  Kd = 0;
  Kf = 12.5;
  
  m_max = 5.8;
  m_min = -5.8;
  
  Se = 0;
  e = 0;
  last_e = 0;
  w = 0;
  w_ref = 0;
  m = 0;
}

float PID_t::calc(float new_w_ref, float new_w)
{
  float de;
  w = new_w;
  w_ref = new_w_ref;

  last_e = e;
  e = w_ref - w;
  
  Se += e * dt;
  de = (e - last_e) / dt;
  
  m = Kp * e + Ki * Se + Kd * de + Kf * w_ref;

  // Anti windup
  if ((m > m_max && e < 0) || (m < m_min && e > 0)) {
    Se -= e * dt;
  }
  
  // Saturate
  if (m > m_max) {
    m = m_max;
  } else if (m < m_min) {
    m = m_min;
  }

  return m;
}