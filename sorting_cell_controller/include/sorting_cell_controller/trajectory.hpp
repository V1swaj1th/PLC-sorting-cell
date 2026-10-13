#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "sorting_cell_controller/ur5e_ik.hpp"

namespace sorting_cell {

// Trapezoidal velocity profile for a normalized path coordinate s in [0, 1]:
// constant acceleration, optional constant-velocity cruise, constant deceleration.
// If the path is too short to reach v_max, the profile is triangular.
class TrapezoidProfile {
 public:
  // v_max: peak ds/dt [1/s]. a_max: |d2s/dt2| [1/s^2].
  TrapezoidProfile(double v_max, double a_max) : a_(std::max(a_max, 1e-9)) {
    v_max = std::max(v_max, 1e-9);
    v_peak_ = (v_max * v_max / a_ >= 1.0) ? std::sqrt(a_) : v_max;
    t_acc_ = v_peak_ / a_;
    const double s_acc = 0.5 * a_ * t_acc_ * t_acc_;
    t_cruise_ = std::max(0.0, (1.0 - 2.0 * s_acc) / v_peak_);
    duration_ = 2.0 * t_acc_ + t_cruise_;
  }

  double duration() const { return duration_; }
  double peakVelocity() const { return v_peak_; }

  double position(double t) const {
    t = std::min(std::max(t, 0.0), duration_);
    if (t < t_acc_) return 0.5 * a_ * t * t;
    if (t < t_acc_ + t_cruise_) return 0.5 * a_ * t_acc_ * t_acc_ + v_peak_ * (t - t_acc_);
    const double td = duration_ - t;
    return 1.0 - 0.5 * a_ * td * td;
  }

  double velocity(double t) const {
    if (t < 0.0 || t > duration_) return 0.0;
    if (t < t_acc_) return a_ * t;
    if (t < t_acc_ + t_cruise_) return v_peak_;
    return a_ * (duration_ - t);
  }

 private:
  double a_, v_peak_{0.0}, t_acc_{0.0}, t_cruise_{0.0}, duration_{0.0};
};

struct CartesianLimits {
  double v_lin = 0.25;  // [m/s]
  double a_lin = 0.50;  // [m/s^2]
  double v_ang = 1.00;  // [rad/s]
  double a_ang = 2.00;  // [rad/s^2]
};

// Straight line in position plus spherical interpolation (slerp) in orientation, both
// driven by one trapezoidal profile so they start and stop together. The profile limits
// are chosen so neither the linear nor the angular speed/acceleration limit is exceeded.
class CartesianPath {
 public:
  CartesianPath(const Eigen::Matrix4d& T0, const Eigen::Matrix4d& T1, const CartesianLimits& lim)
      : p0_(T0.block<3, 1>(0, 3)),
        p1_(T1.block<3, 1>(0, 3)),
        q0_(Eigen::Matrix3d(T0.block<3, 3>(0, 0))),
        q1_(Eigen::Matrix3d(T1.block<3, 3>(0, 0))),
        profile_(1.0, 1.0) {
    const double lp = (p1_ - p0_).norm();
    const double lr = q0_.angularDistance(q1_);
    degenerate_ = (lp < 1e-9 && lr < 1e-9);
    double v_s = 1e9, a_s = 1e9;
    if (lp >= 1e-9) {
      v_s = std::min(v_s, lim.v_lin / lp);
      a_s = std::min(a_s, lim.a_lin / lp);
    }
    if (lr >= 1e-9) {
      v_s = std::min(v_s, lim.v_ang / lr);
      a_s = std::min(a_s, lim.a_ang / lr);
    }
    if (!degenerate_) profile_ = TrapezoidProfile(v_s, a_s);
  }

  double duration() const { return degenerate_ ? 0.0 : profile_.duration(); }

  Eigen::Matrix4d poseAt(double t) const {
    const double s = degenerate_ ? 1.0 : profile_.position(t);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q0_.slerp(s, q1_).toRotationMatrix();
    T.block<3, 1>(0, 3) = p0_ + s * (p1_ - p0_);
    return T;
  }

 private:
  Eigen::Vector3d p0_, p1_;
  Eigen::Quaterniond q0_, q1_;
  TrapezoidProfile profile_;
  bool degenerate_{false};
};

struct JointTrajectory {
  bool ok{false};
  int fail_index{-1};            // sample at which IK failed, if !ok
  double dt{0.02};               // [s] between samples
  std::vector<Vector6d> q;       // q[k] is the joint vector at time k*dt
};

// Plan a straight-line Cartesian move from q_start to T_goal. Each sample is solved with
// DLS IK, seeded from the previous sample. Fails (ok = false) if any IK solve does not converge.
inline JointTrajectory planCartesianMove(const Vector6d& q_start, const Eigen::Matrix4d& T_goal,
                                         const CartesianLimits& lim = CartesianLimits(),
                                         double dt = 0.02, const IkOptions& ik = IkOptions()) {
  JointTrajectory traj;
  traj.dt = dt;
  const CartesianPath path(forwardKinematics(q_start), T_goal, lim);
  const int n = std::max(1, static_cast<int>(std::ceil(path.duration() / dt)));
  traj.q.reserve(static_cast<size_t>(n) + 1);
  traj.q.push_back(q_start);
  Vector6d q = q_start;
  for (int k = 1; k <= n; ++k) {
    const double t = std::min(k * dt, path.duration());
    const IkResult r = solveIk(path.poseAt(t), q, ik);
    if (!r.converged) {
      traj.fail_index = k;
      return traj;
    }
    q = r.q;
    traj.q.push_back(q);
  }
  traj.ok = true;
  return traj;
}

}  // namespace sorting_cell
