#pragma once
// One servo axis as a drive would run it, at a fixed sample rate:
//   encoder counts -> position, filtered velocity estimate
//   CASCADE:  position P (+ velocity feedforward) -> velocity setpoint -> velocity PI -> torque
//   PID:      position PID (filtered derivative on the measurement, velocity feedforward) -> torque
// The current/torque loop is taken as ideal (it runs much faster than this loop on a real drive).
#include <stdint.h>

#include "servo_core/pid.hpp"

namespace servo {

enum class LoopMode : uint8_t { CASCADE = 0, PID = 1 };

struct AxisConfig {
  LoopMode mode{LoopMode::CASCADE};
  float dt{0.001f};
  float rad_per_count{1e-5f};      // joint radians per encoder count (gear ratio included)
  float vel_filter_hz{200.0f};     // velocity estimate low-pass
  float torque_limit{100.0f};      // [Nm] at the joint
  float vel_limit{3.14f};          // [rad/s] velocity setpoint clamp (cascade)
  // cascade
  float kp_pos{20.0f};             // [1/s]
  float vel_ff{1.0f};              // 0..1, share of the reference velocity fed forward
  float kp_vel{100.0f};            // [Nm/(rad/s)]
  float ki_vel{2000.0f};           // [Nm/rad]
  // single-loop PID (torque out)
  float kp{1000.0f}, ki{0.0f}, kd{50.0f}, d_cutoff_hz{100.0f};
  bool anti_windup{true};
  float following_error_limit{0.05f};  // [rad]
};

struct AxisOutput {
  float torque;           // [Nm] command, already limited
  float position;         // [rad] from the encoder
  float velocity;         // [rad/s] filtered estimate
  float following_error;  // reference - position [rad]
  bool saturated;
};

class ServoAxis {
 public:
  void configure(const AxisConfig& c);
  // Re-initialises from the current encoder reading; the integrator is preloaded with
  // `hold_torque` so that switching on under load does not drop the axis.
  void reset(int32_t encoder_counts, float hold_torque = 0.0f);
  AxisOutput update(float pos_ref, float vel_ref, int32_t encoder_counts, float torque_ff = 0.0f);
  // Only the encoder bookkeeping (used while the torque is off).
  void observe(int32_t encoder_counts);

  float position() const { return pos_; }
  float velocity() const { return vel_; }
  const AxisConfig& config() const { return c_; }

 private:
  AxisConfig c_{};
  Pid vel_pi_{};
  Pid pos_pid_{};
  int32_t prev_counts_{0};
  float pos_{0.0f};
  float vel_{0.0f};
  float vel_alpha_{1.0f};
};

}  // namespace servo
