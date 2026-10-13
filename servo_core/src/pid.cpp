#include "servo_core/pid.hpp"

namespace servo {

void Pid::configure(const PidConfig& c) {
  c_ = c;
  if (c_.kaw < 0.0f) kaw_ = (c_.kp > 0.0f) ? c_.ki / c_.kp : 0.0f;
  else kaw_ = c_.kaw;
  // first-order low-pass, discretised: y += alpha*(x - y), alpha = dt / (dt + 1/(2*pi*fc))
  const float tau = 1.0f / (6.2831853f * (c_.d_cutoff_hz > 0.0f ? c_.d_cutoff_hz : 1e6f));
  alpha_ = c_.dt / (c_.dt + tau);
  first_ = true;
}

void Pid::reset(float measurement, float integrator) {
  i_ = clampf(integrator, -c_.i_limit, c_.i_limit);
  d_ = 0.0f;
  prev_meas_ = measurement;
  u_ = 0.0f;
  first_ = false;
  saturated_ = false;
}

float Pid::update(float setpoint, float measurement, float feedforward) {
  if (first_) reset(measurement, i_);
  const float e = setpoint - measurement;

  const float d_raw = -(measurement - prev_meas_) / c_.dt;
  prev_meas_ = measurement;
  d_ += alpha_ * (d_raw - d_);

  const float u_unsat = c_.kp * e + i_ + c_.kd * d_ + feedforward;
  const float u = clampf(u_unsat, c_.out_min, c_.out_max);
  saturated_ = (u != u_unsat);

  i_ += (c_.ki * e + kaw_ * (u - u_unsat)) * c_.dt;
  i_ = clampf(i_, -c_.i_limit, c_.i_limit);
  u_ = u;
  return u;
}

}  // namespace servo
