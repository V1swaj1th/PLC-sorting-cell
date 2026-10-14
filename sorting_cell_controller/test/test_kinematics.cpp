// Independent checks of the kinematics, IK, velocity profile and pick-and-place planner.
//
// The FK is compared with the joint chain of the official UR5e description
// (ur_description/config/ur5e/default_kinematics.yaml and the fixed frames of ur_macro.xacro),
// which is a different parameterisation from the DH table the controller uses.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <random>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;

namespace {

Eigen::Matrix4d xyzRpy(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return T;
}

Vector6d randomJoints(std::mt19937& rng) {
  Vector6d q;
  for (int j = 0; j < 6; ++j) q[j] = std::uniform_real_distribution<double>(kQMin[j], kQMax[j])(rng);
  return q;
}

double rotationAngle(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
  return Eigen::AngleAxisd(Eigen::Matrix3d(A.block<3, 3>(0, 0).transpose() * B.block<3, 3>(0, 0))).angle();
}

}  // namespace

TEST(Kinematics, ForwardKinematicsMatchesUrdfChain) {
#ifndef UR_KINEMATICS_YAML
  GTEST_SKIP() << "ur_description not found at build time";
#else
  std::ifstream probe(UR_KINEMATICS_YAML);
  if (!probe) GTEST_SKIP() << "cannot read " << UR_KINEMATICS_YAML;
  const YAML::Node k = YAML::LoadFile(UR_KINEMATICS_YAML)["kinematics"];
  const char* links[6] = {"shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"};
  Eigen::Matrix4d joint_origin[6];
  for (int i = 0; i < 6; ++i) {
    const YAML::Node n = k[links[i]];
    joint_origin[i] = xyzRpy(n["x"].as<double>(), n["y"].as<double>(), n["z"].as<double>(),
                             n["roll"].as<double>(), n["pitch"].as<double>(), n["yaw"].as<double>());
  }
  // 'base' and 'base_link_inertia' are both base_link rotated by pi about z, so the chain starts at
  // the identity in 'base'. tool0 = wrist_3_link * flange * tool0 (fixed joints of ur_macro.xacro).
  const Eigen::Matrix4d flange = xyzRpy(0, 0, 0, 0, -M_PI / 2, -M_PI / 2);
  const Eigen::Matrix4d tool0 = xyzRpy(0, 0, 0, M_PI / 2, 0, M_PI / 2);
  std::mt19937 rng(7);
  double max_pos = 0.0, max_rot = 0.0;
  for (int n = 0; n < 20000; ++n) {
    const Vector6d q = randomJoints(rng);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) T = T * joint_origin[i] * xyzRpy(0, 0, 0, 0, 0, q[i]);
    T = T * flange * tool0;
    const Eigen::Matrix4d F = forwardKinematics(q);
    max_pos = std::max(max_pos, (F.block<3, 1>(0, 3) - T.block<3, 1>(0, 3)).norm());
    max_rot = std::max(max_rot, rotationAngle(F, T));
  }
  RecordProperty("max_position_error_m", std::to_string(max_pos));
  RecordProperty("max_rotation_error_rad", std::to_string(max_rot));
  EXPECT_LT(max_pos, 1e-9);
  EXPECT_LT(max_rot, 1e-8);
#endif
}

TEST(Kinematics, JacobianMatchesFiniteDifferences) {
  std::mt19937 rng(3);
  double worst = 0.0;
  for (int n = 0; n < 2000; ++n) {
    const Vector6d q = randomJoints(rng);
    const Matrix6d J = geometricJacobian(q);
    for (int j = 0; j < 6; ++j) {
      const double h = 1e-6;
      Vector6d a = q, b = q;
      a[j] += h;
      b[j] -= h;
      const Eigen::Matrix4d Ta = forwardKinematics(a), Tb = forwardKinematics(b);
      Vector6d col;
      col.head<3>() = (Ta.block<3, 1>(0, 3) - Tb.block<3, 1>(0, 3)) / (2 * h);
      const Eigen::AngleAxisd d(Eigen::Matrix3d(Ta.block<3, 3>(0, 0) * Tb.block<3, 3>(0, 0).transpose()));
      col.tail<3>() = d.axis() * d.angle() / (2 * h);
      worst = std::max(worst, (col - J.col(j)).cwiseAbs().maxCoeff());
    }
  }
  EXPECT_LT(worst, 1e-6);
}

TEST(InverseKinematics, ToolDownPosesOverTheTableAllConverge) {
  // The cell's actual use: tool pointing down anywhere over the table, seeded from home.
  std::mt19937 rng(5);
  int converged = 0;
  const int n = 1000;
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d p(std::uniform_real_distribution<double>(-0.77, -0.23)(rng),
                            std::uniform_real_distribution<double>(-0.40, 0.40)(rng),
                            std::uniform_real_distribution<double>(0.02, 0.25)(rng));
    const double yaw = std::uniform_real_distribution<double>(-M_PI / 4, M_PI / 4)(rng);
    const IkResult r = solveIk(toolDownPose(p, yaw), homeJoints());
    if (r.converged) {
      ++converged;
      EXPECT_LT(r.pos_err, 1e-5);
      EXPECT_LT(r.rot_err, 1e-4);
    }
  }
  EXPECT_EQ(converged, n);
}

TEST(InverseKinematics, LocalSolverFromANearbySeed) {
  // A local DLS solver: from a seed within ~0.1 rad of a solution it should almost always converge.
  // Targets come from uniformly random joint vectors, including near-singular ones.
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.1);
  int converged = 0;
  const int n = 2000;
  for (int i = 0; i < n; ++i) {
    const Vector6d q = randomJoints(rng);
    Vector6d seed = q;
    for (int j = 0; j < 6; ++j) seed[j] += noise(rng);
    const IkResult r = solveIk(forwardKinematics(q), seed);
    converged += r.converged;
    for (int j = 0; j < 6; ++j) {
      EXPECT_GE(r.q[j], kQMin[j]);
      EXPECT_LE(r.q[j], kQMax[j]);
    }
  }
  RecordProperty("converged_fraction", std::to_string(static_cast<double>(converged) / n));
  EXPECT_GE(converged, static_cast<int>(0.85 * n));
}

TEST(InverseKinematics, DampedStepStaysBoundedAtTheWristSingularity) {
  Vector6d q = homeJoints();
  q[4] = 1e-4;  // wrist_2 at zero aligns the wrist_1 and wrist_3 axes
  Vector6d e;
  e << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  const Vector6d dq = dlsStep(geometricJacobian(q), e, IkOptions());
  EXPECT_LT(dq.norm(), 0.2);
}

TEST(InverseKinematics, UnreachableTargetFailsInsideTheLimits) {
  Eigen::Matrix4d T = forwardKinematics(homeJoints());
  T(0, 3) += 2.0;
  const IkResult r = solveIk(T, homeJoints());
  EXPECT_FALSE(r.converged);
  for (int j = 0; j < 6; ++j) {
    EXPECT_GE(r.q[j], kQMin[j]);
    EXPECT_LE(r.q[j], kQMax[j]);
  }
}

TEST(Trajectory, TrapezoidRespectsLimitsIncludingTriangularProfiles) {
  for (double v : {0.1, 0.5, 1.0, 3.0, 10.0}) {
    for (double a : {0.1, 0.5, 2.0, 20.0}) {
      const TrapezoidProfile p(v, a);
      const double T = p.duration();
      ASSERT_GT(T, 0.0);
      EXPECT_NEAR(p.position(0.0), 0.0, 1e-12);
      EXPECT_NEAR(p.position(T), 1.0, 1e-12);
      EXPECT_NEAR(p.velocity(0.0), 0.0, 1e-12);
      EXPECT_NEAR(p.velocity(T), 0.0, 1e-9);
      const int n = 20000;
      const double dt = T / n;
      double prev = 0.0;
      for (int i = 1; i <= n; ++i) {
        const double vel = (p.position(i * dt) - p.position((i - 1) * dt)) / dt;
        EXPECT_LE(vel, v * (1 + 1e-9));
        EXPECT_LE(std::abs(vel - prev) / dt, a * (1 + 1e-6) + 1e-9);
        prev = vel;
      }
      if (v * v / a >= 1.0) {  // short move: no cruise phase, the profile is triangular
        EXPECT_NEAR(p.peakVelocity(), std::sqrt(a), 1e-12);
      }
    }
  }
}

TEST(Trajectory, CartesianMoveFollowsTheLineAtTheSpeedLimit) {
  const Eigen::Matrix4d T0 = forwardKinematics(homeJoints());
  Eigen::Matrix4d T1 = T0;
  T1(0, 3) += 0.20;
  T1(2, 3) -= 0.10;
  const CartesianLimits lim;
  const JointTrajectory tr = planCartesianMove(homeJoints(), T1, lim, 0.02);
  ASSERT_TRUE(tr.ok);
  const Eigen::Vector3d p0 = T0.block<3, 1>(0, 3), dir = (T1.block<3, 1>(0, 3) - p0).normalized();
  double vmax = 0.0;
  for (size_t k = 1; k < tr.q.size(); ++k) {
    const Eigen::Vector3d a = forwardKinematics(tr.q[k - 1]).block<3, 1>(0, 3);
    const Eigen::Vector3d b = forwardKinematics(tr.q[k]).block<3, 1>(0, 3);
    vmax = std::max(vmax, (b - a).norm() / 0.02);
    const Eigen::Vector3d rel = b - p0;
    EXPECT_LT((rel - dir * rel.dot(dir)).norm(), 1e-4);
  }
  EXPECT_LE(vmax, lim.v_lin * 1.001);
  EXPECT_LT(poseError(forwardKinematics(tr.q.back()), T1).norm(), 1e-4);
}

TEST(Planner, MatchingRuleAndGrayContainer) {
  PickPlaceConfig cfg;  // bins: red, green, blue, yellow
  const std::vector<CubeTarget> cubes = {{"red", {-0.45, -0.15, 0.02}, 0.0}, {"purple", {-0.55, 0.10, 0.02}, 0.0}};
  const PickPlacePlan p = planPickPlace(cubes, cfg);
  ASSERT_TRUE(p.ok) << p.error;
  EXPECT_EQ(p.dest[0], "red");
  EXPECT_EQ(p.dest[1], kGrayName);
  cfg.has_gray_bin = false;
  EXPECT_FALSE(planPickPlace(cubes, cfg).ok);
}

TEST(Planner, RandomScenesStayWithinTheJointSpeedLimit) {
  // The planner limits Cartesian speed only; this checks that the joint speeds it produces for
  // accepted scenes stay below the UR5e limit of 3.14 rad/s.
  SceneModel model(17);
  double worst = 0.0;
  int planned = 0;
  for (int n = 0; n < 100; ++n) {
    if (!model.execute("random all").ok) continue;
    const Scene& s = model.scene();
    const PickPlacePlan p = planPickPlace(cubeTargets(s), planConfig(s));
    ASSERT_TRUE(p.ok) << p.error;
    ++planned;
    for (size_t f = 1; f < p.q.size(); ++f) worst = std::max(worst, (p.q[f] - p.q[f - 1]).cwiseAbs().maxCoeff() / 0.02);
    for (size_t i = 0; i < s.objects.size(); ++i) {  // the tool reaches each cube and each drop place
      const Eigen::Vector3d g = forwardKinematics(p.q[static_cast<size_t>(p.grasp_frame[i] - 1)]).block<3, 1>(0, 3);
      EXPECT_LT((g - Eigen::Vector3d(s.objects[i].x, s.objects[i].y, 0.02)).norm(), 1e-4);
      const Eigen::Vector3d r = forwardKinematics(p.q[static_cast<size_t>(p.release_frame[i] - 1)]).block<3, 1>(0, 3);
      EXPECT_LT((r - p.place_pos[i]).norm(), 1e-4);
    }
  }
  RecordProperty("peak_joint_speed_rad_s", std::to_string(worst));
  EXPECT_GE(planned, 90);
  EXPECT_LT(worst, 3.14);
}
