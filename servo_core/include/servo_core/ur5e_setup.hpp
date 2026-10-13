#pragma once
// UR5e joint parameters for the simulated plant, drive gains, and DriveSim: drive firmware +
// plant stepped together at the control rate (used by the tests, the evidence tool and the ROS node).
//
// Plant values: torque limits 150/150/150/28/28/28 Nm and 3.14 rad/s from ur_description's
// joint_limits.yaml; 101:1 gears; rotor inertia, friction and brake torque are assumed values of a
// plausible size, not manufacturer data.
#include "servo_core/drive.hpp"
#include "servo_core/joint_plant.hpp"

namespace servo_sim {

inline std::array<JointParams, 6> ur5ePlant() {
  std::array<JointParams, 6> p{};
  for (int j = 0; j < 6; ++j) {
    const bool big = j < 3;
    p[j].gear_ratio = 101.0;
    p[j].rotor_inertia = big ? 6.0e-5 : 1.3e-5;
    p[j].inertia_floor = big ? 0.02 : 0.005;
    p[j].viscous = big ? 4.0 : 0.8;
    p[j].coulomb = big ? 6.0 : 1.2;
    p[j].torque_limit = big ? 150.0 : 28.0;
    p[j].vel_limit = 3.14;
    p[j].encoder_cpr = 4096;
    p[j].brake_torque = big ? 250.0 : 50.0;
  }
  return p;
}

enum class Tuning { CASCADE, PID, P_ONLY };

// Velocity loop: bandwidth about 60 rad/s (wrists 120 rad/s) at a nominal inertia per joint; ki = kp*wv/5;
// position loop kp = wv/4. Single-loop PID / P-only use gains of similar stiffness.
inline servo::DriveConfig ur5eDrive(Tuning t = Tuning::CASCADE, bool anti_windup = true) {
  servo::DriveConfig c{};
  c.dt = 0.001f;
  const std::array<JointParams, 6> P = ur5ePlant();
  const float j_nom[6] = {2.5f, 3.5f, 1.5f, 0.17f, 0.15f, 0.14f};
  for (int j = 0; j < 6; ++j) {
    servo::AxisConfig& a = c.axis[j];
    // the wrists are light and friction-dominated: twice the bandwidth (still kp_vel*dt/J < 0.2)
    const float wv = j < 3 ? 60.0f : 120.0f;
    a.dt = c.dt;
    a.rad_per_count = static_cast<float>(2.0 * M_PI / (P[j].gear_ratio * P[j].encoder_cpr));
    a.vel_filter_hz = 200.0f;
    a.torque_limit = static_cast<float>(P[j].torque_limit);
    a.vel_limit = 2.9f;  // setpoint limit below the 3.14 rad/s joint limit leaves room for overshoot
    a.anti_windup = anti_windup;
    a.kp_vel = j_nom[j] * wv;
    a.ki_vel = a.kp_vel * wv / 5.0f;
    a.kp_pos = wv / 4.0f;
    a.vel_ff = 1.0f;
    // single loop: same proportional stiffness as the cascade (kp_pos*kp_vel), damping kp_vel
    a.kp = a.kp_pos * a.kp_vel;
    a.kd = a.kp_vel;
    a.ki = (t == Tuning::PID) ? a.kp * 4.0f : 0.0f;
    a.d_cutoff_hz = 100.0f;
    a.mode = (t == Tuning::CASCADE) ? servo::LoopMode::CASCADE : servo::LoopMode::PID;
    a.following_error_limit = j < 3 ? 0.03f : 0.05f;
  }
  return c;
}

// Drive firmware + plant, advanced one control period per step().
class DriveSim {
 public:
  void init(const Arr6& q0, const servo::DriveConfig& dc, bool gravity_ff = true) {
    plant_.configure(ur5ePlant(), dc.dt);
    plant_.setState(q0, Arr6{});
    drive_.configure(dc);
    gravity_ff_ = gravity_ff;
  }
  // `in` carries targets and the safety inputs; encoders and torque feedforward are filled here.
  const servo::DriveOutputs& step(servo::DriveInputs in) {
    Arr6 qm{};
    for (int j = 0; j < 6; ++j) {
      in.encoder[j] = plant_.encoder(j);
      qm[j] = in.encoder[j] * plant_.radPerCount(j);
    }
    if (gravity_ff_) {
      const ArmLoad L = armLoad(qm, nominal_payload_);  // the controller's model: payload unknown
      for (int j = 0; j < 6; ++j) in.torque_ff[j] = static_cast<float>(L.gravity[j]);
    }
    const servo::DriveOutputs& o = drive_.step(in);
    plant_.setBrake(o.brake_engaged);
    double tau[6];
    for (int j = 0; j < 6; ++j) tau[j] = o.torque[j];
    plant_.step(tau);
    return o;
  }
  ArmPlant& plant() { return plant_; }
  servo::Drive& drive() { return drive_; }
  void setGravityFeedforward(bool on) { gravity_ff_ = on; }

 private:
  servo::Drive drive_{};
  ArmPlant plant_{};
  bool gravity_ff_{true};
  double nominal_payload_{0.0};
};

}  // namespace servo_sim
