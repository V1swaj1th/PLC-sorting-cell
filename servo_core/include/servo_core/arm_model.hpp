#pragma once
// Simulation side (not firmware): UR5e gravity torques and the diagonal of the mass matrix,
// from point masses at the link centres of mass. Masses and centres of mass are UR's published
// dynamics parameters for the UR5e (approximate; expressed in the UR DH frames). Link rotational
// inertia, off-diagonal coupling and Coriolis terms are not modelled: with 101:1 gears the
// reflected rotor inertia dominates, which is the usual case for independent joint control.
#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace servo_sim {

using Arr6 = std::array<double, 6>;

constexpr Arr6 kDhD = {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996};
constexpr Arr6 kDhA = {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0};
constexpr Arr6 kDhAlpha = {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0};
constexpr double kGravity = 9.81;

struct LinkMass {
  double m;
  double cx, cy, cz;  // centre of mass in the link's DH frame [m]
};
constexpr std::array<LinkMass, 6> kUr5eLinks = {{{3.761, 0.0, -0.02561, 0.00193},
                                                 {8.058, 0.2125, 0.0, 0.11336},
                                                 {2.846, 0.15, 0.0, 0.0265},
                                                 {1.37, 0.0, -0.0018, 0.01634},
                                                 {1.3, 0.0, 0.0018, 0.01634},
                                                 {0.365, 0.0, 0.0, -0.001159}}};

inline Eigen::Matrix4d dh(double theta, double d, double a, double alpha) {
  const double ct = std::cos(theta), st = std::sin(theta), ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d T;
  T << ct, -st * ca, st * sa, a * ct, st, ct * ca, -ct * sa, a * st, 0, sa, ca, d, 0, 0, 0, 1;
  return T;
}

struct ArmLoad {
  Arr6 inertia{};  // link part of the mass-matrix diagonal [kg m^2]
  Arr6 gravity{};  // torque each joint must supply to hold the arm still [Nm]
};

// payload_kg is a point mass at the tool flange.
inline ArmLoad armLoad(const Arr6& q, double payload_kg) {
  Eigen::Matrix4d T[7];
  T[0].setIdentity();
  for (int i = 0; i < 6; ++i) T[i + 1] = T[i] * dh(q[i], kDhD[i], kDhA[i], kDhAlpha[i]);
  // point masses: 6 links + payload
  Eigen::Vector3d p[7];
  double m[7];
  for (int k = 0; k < 6; ++k) {
    p[k] = (T[k + 1] * Eigen::Vector4d(kUr5eLinks[k].cx, kUr5eLinks[k].cy, kUr5eLinks[k].cz, 1.0)).head<3>();
    m[k] = kUr5eLinks[k].m;
  }
  p[6] = T[6].block<3, 1>(0, 3);
  m[6] = payload_kg;
  ArmLoad out;
  for (int i = 0; i < 6; ++i) {  // joint i turns about z of frame i, through the origin of frame i
    const Eigen::Vector3d z = T[i].block<3, 1>(0, 2), o = T[i].block<3, 1>(0, 3);
    double inertia = 0.0, grav = 0.0;
    for (int k = i; k < 7; ++k) {  // masses moved by joint i: links i..5 and the payload
      if (m[k] == 0.0) continue;
      const Eigen::Vector3d jv = z.cross(p[k] - o);
      inertia += m[k] * jv.squaredNorm();
      grav += m[k] * kGravity * jv.z();
    }
    out.inertia[i] = inertia;
    out.gravity[i] = grav;
  }
  return out;
}

// Potential energy of the point masses (for checking the gravity torques numerically).
inline double potentialEnergy(const Arr6& q, double payload_kg) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  double v = 0.0;
  for (int k = 0; k < 6; ++k) {
    T = T * dh(q[k], kDhD[k], kDhA[k], kDhAlpha[k]);
    const Eigen::Vector4d c = T * Eigen::Vector4d(kUr5eLinks[k].cx, kUr5eLinks[k].cy, kUr5eLinks[k].cz, 1.0);
    v += kUr5eLinks[k].m * kGravity * c.z();
  }
  return v + payload_kg * kGravity * T(2, 3);
}

}  // namespace servo_sim
