#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace sorting_cell {

using Vector6d = Eigen::Matrix<double, 6, 1>;

// UR5e standard DH parameters; lengths cross-checked against
// ur_description/config/ur5e/default_kinematics.yaml
constexpr std::array<double, 6> kD     = {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996};
constexpr std::array<double, 6> kA     = {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kAlpha = {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0};

// Standard DH link transform: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
inline Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
  const double ct = std::cos(theta), st = std::sin(theta);
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T;
  T << ct, -st * ca,  st * sa, a * ct,
       st,  ct * ca, -ct * sa, a * st,
      0.0,       sa,       ca,      d,
      0.0,      0.0,      0.0,    1.0;
  return T;
}

// Pose of the tool flange in the robot base frame
inline Eigen::Matrix4d forwardKinematics(const Vector6d& q) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 6; ++i) {
    T *= dhTransform(q[i], kD[i], kA[i], kAlpha[i]);
  }
  return T;
}

}  // namespace sorting_cell
