#pragma once
// Discrete PID for a fixed-rate control loop. Firmware style: float only, no heap, no
// exceptions, no I/O; every call does a bounded amount of work.
//
//   u = kp*e + I + D + ff,   e = setpoint - measurement
//   D: derivative of the MEASUREMENT (no kick on setpoint steps), low-pass filtered
//   I: integrated error with back-calculation anti-windup:
//        I += ki*e*dt + kaw*(u_sat - u_unsat)*dt   (kaw = 0 disables anti-windup)
//   u is clamped to [out_min, out_max].

namespace servo {

struct PidConfig {
  float kp{0.0f};
  float ki{0.0f};
  float kd{0.0f};
  float d_cutoff_hz{100.0f};  // first-order filter on the derivative term
  float out_min{-1.0f};
  float out_max{1.0f};
  float kaw{-1.0f};           // back-calculation gain [1/s]; < 0 means "use ki/kp", 0 disables anti-windup
  float i_limit{1e9f};        // hard clamp on the integrator as a last resort
  float dt{0.001f};
};

class Pid {
 public:
  Pid() = default;
  explicit Pid(const PidConfig& c) { configure(c); }

  void configure(const PidConfig& c);
  // Clears the state. The integrator is preloaded with `integrator` (bumpless start).
  void reset(float measurement, float integrator = 0.0f);
  float update(float setpoint, float measurement, float feedforward = 0.0f);

  float integrator() const { return i_; }
  float lastOutput() const { return u_; }
  bool saturated() const { return saturated_; }
  const PidConfig& config() const { return c_; }

 private:
  PidConfig c_{};
  float kaw_{0.0f};
  float alpha_{1.0f};  // derivative filter coefficient
  float i_{0.0f};
  float d_{0.0f};
  float prev_meas_{0.0f};
  float u_{0.0f};
  bool first_{true};
  bool saturated_{false};
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float absf(float v) { return v < 0.0f ? -v : v; }

}  // namespace servo
