#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>

#include "sorting_cell_controller/ur5e_kinematics.hpp"

namespace sorting_cell {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Joint limits [rad]. ur_description uses +-2pi, except the elbow, limited to +-pi.
constexpr std::array<double, 6> kQMin = {-2 * M_PI, -2 * M_PI, -M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI};
constexpr std::array<double, 6> kQMax = { 2 * M_PI,  2 * M_PI,  M_PI,  2 * M_PI,  2 * M_PI,  2 * M_PI};

// Geometric Jacobian [Jv; Jw], expressed in the base frame.
// Joint i rotates about the z axis of frame i-1, so z_{i-1} and p_{i-1} are taken
// BEFORE link i's transform is applied:  Jv_i = z_{i-1} x (p_e - p_{i-1}),  Jw_i = z_{i-1}.
inline Matrix6d geometricJacobian(const Vector6d& q) {
  std::array<Eigen::Vector3d, 6> z, p;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 6; ++i) {
    z[i] = T.block<3, 1>(0, 2);
    p[i] = T.block<3, 1>(0, 3);
    T *= dhTransform(q[i], kD[i], kA[i], kAlpha[i]);
  }
  const Eigen::Vector3d pe = T.block<3, 1>(0, 3);
  Matrix6d J;
  for (int i = 0; i < 6; ++i) {
    J.block<3, 1>(0, i) = z[i].cross(pe - p[i]);
    J.block<3, 1>(3, i) = z[i];
  }
  return J;
}

// Yoshikawa manipulability sqrt(det(J J^T)). For a square 6x6 Jacobian this is |det J|.
// It tends to 0 at a singularity. Units are mixed (m and rad), so only compare values
// between poses of the same robot.
inline double manipulability(const Matrix6d& J) { return std::abs(J.determinant()); }
inline double manipulability(const Vector6d& q) { return manipulability(geometricJacobian(q)); }

// Pose error [position; orientation as a rotation vector], desired minus current,
// expressed in the base frame (matches the Jacobian's angular rows).
inline Vector6d poseError(const Eigen::Matrix4d& T_cur, const Eigen::Matrix4d& T_des) {
  Vector6d e;
  e.head<3>() = T_des.block<3, 1>(0, 3) - T_cur.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_err = T_des.block<3, 3>(0, 0) * T_cur.block<3, 3>(0, 0).transpose();
  const Eigen::AngleAxisd aa(R_err);
  e.tail<3>() = aa.axis() * aa.angle();
  return e;
}

struct IkOptions {
  double lambda_min = 0.01;   // damping far from singularities
  double lambda_max = 0.10;   // damping at a singularity (plan: 0.05 to 0.1)
  double w_threshold = 0.02;  // manipulability below which damping ramps up
  int max_iter = 500;
  double pos_tol = 1e-5;      // [m]
  double rot_tol = 1e-4;      // [rad]
  double max_step = 0.2;      // [rad] per iteration, largest joint
};

// One damped-least-squares step: dq = J^T (J J^T + lambda^2 I)^-1 e.
// lambda^2 is lambda_min^2 normally and ramps to lambda_max^2 as the manipulability w
// drops below w_threshold (Nakamura-Hanafusa style adaptive damping).
inline Vector6d dlsStep(const Matrix6d& J, const Vector6d& e, const IkOptions& o) {
  const double w = manipulability(J);
  double l2 = o.lambda_min * o.lambda_min;
  if (w < o.w_threshold) {
    const double r = w / o.w_threshold;
    l2 += (o.lambda_max * o.lambda_max - l2) * (1.0 - r * r);
  }
  const Matrix6d A = J * J.transpose() + l2 * Matrix6d::Identity();
  return J.transpose() * A.ldlt().solve(e);
}

struct IkResult {
  bool converged{false};
  int iterations{0};
  double pos_err{0.0};  // [m]
  double rot_err{0.0};  // [rad]
  double manip{0.0};    // manipulability at the returned solution
  Vector6d q{Vector6d::Zero()};
};

// Iterative DLS IK seeded from q_seed (use the previous solution along a path).
// Steps are clamped and the result is clamped to the joint limits.
inline IkResult solveIk(const Eigen::Matrix4d& T_des, const Vector6d& q_seed,
                        const IkOptions& o = IkOptions()) {
  IkResult res;
  res.q = q_seed;
  for (int it = 0; it <= o.max_iter; ++it) {
    const Vector6d e = poseError(forwardKinematics(res.q), T_des);
    res.pos_err = e.head<3>().norm();
    res.rot_err = e.tail<3>().norm();
    res.iterations = it;
    res.converged = (res.pos_err < o.pos_tol && res.rot_err < o.rot_tol);
    if (res.converged || it == o.max_iter) break;

    const Matrix6d J = geometricJacobian(res.q);
    Vector6d dq = dlsStep(J, e, o);
    const double m = dq.cwiseAbs().maxCoeff();
    if (m > o.max_step) dq *= o.max_step / m;
    res.q += dq;
    for (int i = 0; i < 6; ++i) {
      res.q[i] = std::min(std::max(res.q[i], kQMin[i]), kQMax[i]);
    }
  }
  res.manip = manipulability(res.q);
  return res;
}

}  // namespace sorting_cell
