#pragma once
// Simulation side (not firmware): six independent joint plants driven by joint torque.
//
//   J(q) * dv/dt = tau_motor - tau_gravity(q) - b*v - friction - brake/jam
//   J(q) = link inertia from armLoad() + gear_ratio^2 * rotor inertia + a small floor
//
// Coulomb friction with stiction (Karnopp): a joint at rest stays at rest while the net torque is
// below the static friction; an engaged brake or an injected jam acts the same way with a large
// holding torque. The motor torque is limited, and falls to zero above the velocity limit.
// Integration: semi-implicit Euler at the control rate. Encoders: incremental, on the motor.
#include <stdint.h>

#include <algorithm>
#include <cmath>

#include "servo_core/arm_model.hpp"

namespace servo_sim {

struct JointParams {
  double gear_ratio{101.0};
  double rotor_inertia{6.0e-5};     // [kg m^2] motor side
  double inertia_floor{0.02};       // link rotational inertia not in the point-mass model [kg m^2]
  double viscous{4.0};              // [Nm/(rad/s)] joint side
  double coulomb{6.0};              // [Nm]
  double stiction_vel{2e-4};        // [rad/s] below this the joint can stick
  double torque_limit{150.0};       // [Nm] joint side
  double vel_limit{3.14};           // [rad/s]
  int encoder_cpr{4096};            // counts per MOTOR revolution
  double brake_torque{250.0};       // [Nm] holding torque of the brake
  double jam_torque{2000.0};        // [Nm] an injected jam
};

class ArmPlant {
 public:
  void configure(const std::array<JointParams, 6>& p, double dt) {
    p_ = p;
    dt_ = dt;
  }
  void setState(const Arr6& q, const Arr6& v) {
    q_ = q;
    v_ = v;
  }
  void setPayload(double kg) { payload_ = kg; }
  void setBrake(bool engaged) { brake_ = engaged; }
  void setJam(int joint) { jam_ = joint; }  // -1 = none
  double payload() const { return payload_; }

  void step(const double torque_cmd[6]) {
    const ArmLoad L = armLoad(q_, payload_);
    load_ = L;
    for (int j = 0; j < 6; ++j) {
      const JointParams& P = p_[j];
      const double J = L.inertia[j] + P.gear_ratio * P.gear_ratio * P.rotor_inertia + P.inertia_floor;
      double tau = std::clamp(torque_cmd[j], -P.torque_limit, P.torque_limit);
      if (tau * v_[j] > 0.0 && std::abs(v_[j]) > P.vel_limit)  // back-EMF: no drive torque beyond the speed limit
        tau *= std::max(0.0, 1.0 - (std::abs(v_[j]) - P.vel_limit) / (0.2 * P.vel_limit));
      applied_[j] = tau;
      const double drive = tau - L.gravity[j];  // everything except friction and brake
      double hold = P.coulomb;
      if (brake_) hold = std::max(hold, P.brake_torque);
      if (jam_ == j) hold = std::max(hold, P.jam_torque);
      double a;
      if (std::abs(v_[j]) < P.stiction_vel) {
        if (std::abs(drive) <= hold) {
          v_[j] = 0.0;
          continue;
        }
        a = (drive - std::copysign(hold, drive)) / J;
      } else {
        a = (drive - P.viscous * v_[j] - std::copysign(hold, v_[j])) / J;
      }
      const double v_new = v_[j] + a * dt_;
      // friction cannot reverse the motion within one step: stop and let stiction decide next step
      v_[j] = (v_new * v_[j] < 0.0 && std::abs(drive) <= hold) ? 0.0 : v_new;
      q_[j] += v_[j] * dt_;
    }
  }

  int32_t encoder(int j) const {
    const double counts_per_rad = p_[j].gear_ratio * p_[j].encoder_cpr / (2.0 * M_PI);
    return static_cast<int32_t>(std::lround(q_[j] * counts_per_rad));
  }
  double radPerCount(int j) const { return 2.0 * M_PI / (p_[j].gear_ratio * p_[j].encoder_cpr); }
  const Arr6& q() const { return q_; }
  const Arr6& v() const { return v_; }
  const Arr6& appliedTorque() const { return applied_; }
  const ArmLoad& load() const { return load_; }
  const JointParams& params(int j) const { return p_[j]; }

 private:
  std::array<JointParams, 6> p_{};
  double dt_{0.001};
  Arr6 q_{}, v_{}, applied_{};
  ArmLoad load_{};
  double payload_{0.0};
  bool brake_{false};
  int jam_{-1};
};

}  // namespace servo_sim
