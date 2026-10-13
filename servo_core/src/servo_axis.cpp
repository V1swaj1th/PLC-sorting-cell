#include "servo_core/servo_axis.hpp"

namespace servo {

void ServoAxis::configure(const AxisConfig& c) {
  c_ = c;
  const float tau = 1.0f / (6.2831853f * c_.vel_filter_hz);
  vel_alpha_ = c_.dt / (c_.dt + tau);

  PidConfig v;
  v.kp = c_.kp_vel;
  v.ki = c_.ki_vel;
  v.kd = 0.0f;
  v.out_min = -c_.torque_limit;
  v.out_max = c_.torque_limit;
  v.kaw = c_.anti_windup ? -1.0f : 0.0f;
  v.dt = c_.dt;
  vel_pi_.configure(v);

  PidConfig p;
  p.kp = c_.kp;
  p.ki = c_.ki;
  p.kd = c_.kd;
  p.d_cutoff_hz = c_.d_cutoff_hz;
  p.out_min = -c_.torque_limit;
  p.out_max = c_.torque_limit;
  p.kaw = c_.anti_windup ? -1.0f : 0.0f;
  p.dt = c_.dt;
  pos_pid_.configure(p);
}

void ServoAxis::observe(int32_t counts) {
  const int32_t delta = counts - prev_counts_;
  prev_counts_ = counts;
  pos_ = static_cast<float>(counts) * c_.rad_per_count;
  const float v_raw = static_cast<float>(delta) * c_.rad_per_count / c_.dt;
  vel_ += vel_alpha_ * (v_raw - vel_);
}

void ServoAxis::reset(int32_t counts, float hold_torque) {
  prev_counts_ = counts;
  pos_ = static_cast<float>(counts) * c_.rad_per_count;
  vel_ = 0.0f;
  vel_pi_.reset(0.0f, hold_torque);
  pos_pid_.reset(pos_, hold_torque);
}

AxisOutput ServoAxis::update(float pos_ref, float vel_ref, int32_t counts, float torque_ff) {
  observe(counts);
  AxisOutput o{};
  o.position = pos_;
  o.velocity = vel_;
  o.following_error = pos_ref - pos_;
  if (c_.mode == LoopMode::CASCADE) {
    const float v_set = clampf(c_.vel_ff * vel_ref + c_.kp_pos * o.following_error, -c_.vel_limit, c_.vel_limit);
    o.torque = vel_pi_.update(v_set, vel_, torque_ff);
    o.saturated = vel_pi_.saturated();
  } else {
    // velocity feedforward through the D gain: kd*(vel_ref - vel) with the measured part filtered inside the PID
    o.torque = pos_pid_.update(pos_ref, pos_, torque_ff + c_.kd * vel_ref);
    o.saturated = pos_pid_.saturated();
  }
  return o;
}

}  // namespace servo
