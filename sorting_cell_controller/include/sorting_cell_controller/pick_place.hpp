#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "sorting_cell_controller/trajectory.hpp"

namespace sorting_cell {

struct CubeTarget {
  std::string color;
  Eigen::Vector3d p;  // cube centre in the base frame [m]
  double yaw{0.0};    // rotation about base z [rad], any value (cube is 90 deg symmetric)
};

struct PickPlaceConfig {
  double hover_z = 0.20;      // tool height for all transport moves [m]
  double grasp_z = 0.02;      // tool height at pick and place (cube centre height) [m]
  int dwell_frames = 15;      // hold at grasp and release (0.3 s at 50 Hz)
  double dt = 0.02;
  CartesianLimits slow;       // descend / ascend
  CartesianLimits fast;       // transport between hover points
  std::map<std::string, Eigen::Vector3d> bins;  // colour -> bin centre
  PickPlaceConfig() {
    fast.v_lin = 0.40;
    fast.a_lin = 0.80;
    fast.v_ang = 1.5;
    fast.a_ang = 3.0;
    bins["red"] = Eigen::Vector3d(-0.30, -0.30, 0.02);
    bins["green"] = Eigen::Vector3d(-0.30, -0.10, 0.02);
    bins["blue"] = Eigen::Vector3d(-0.30, 0.10, 0.02);
    bins["yellow"] = Eigen::Vector3d(-0.30, 0.30, 0.02);
  }
};

struct PickPlacePlan {
  bool ok{false};
  std::string error;
  std::vector<Vector6d> q;                 // joint vector per 50 Hz frame
  std::vector<int> grasp_frame;            // per input cube: carried from this frame on (-1 = never)
  std::vector<int> release_frame;          // per input cube: placed from this frame on (-1 = never)
  std::vector<Eigen::Vector3d> place_pos;  // per input cube
};

inline Vector6d homeJoints() {
  Vector6d q;
  q << 0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0;
  return q;
}

// Tool-down pose with the home orientation rotated by yaw about the base z axis.
inline Eigen::Matrix4d toolDownPose(const Eigen::Vector3d& p, double yaw) {
  const Eigen::Matrix4d T_home = forwardKinematics(homeJoints());
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T_home.block<3, 3>(0, 0);
  T.block<3, 1>(0, 3) = p;
  return T;
}

// Cube yaw is 90 deg symmetric: fold into (-pi/4, pi/4] so the wrist turns as little as possible.
inline double foldCubeYaw(double yaw) {
  double y = std::fmod(yaw, M_PI / 2.0);
  if (y > M_PI / 4.0) y -= M_PI / 2.0;
  if (y <= -M_PI / 4.0) y += M_PI / 2.0;
  return y;
}

// Yaw of a tool pose about the base z axis relative to the home orientation.
inline double toolYaw(const Eigen::Matrix4d& T) {
  const Eigen::Matrix4d T_home = forwardKinematics(homeJoints());
  const Eigen::Matrix3d M = Eigen::Matrix3d(T.block<3, 3>(0, 0)) *
                            Eigen::Matrix3d(T_home.block<3, 3>(0, 0)).transpose();
  return std::atan2(M(1, 0), M(0, 0));
}

// Plans the complete sort: for each cube hover -> descend -> grasp -> ascend -> hover over bin
// -> descend -> release -> ascend, then back to the home pose. Everything is chained from the
// previous joint vector, so the whole plan is one continuous joint-space trajectory.
inline PickPlacePlan planPickPlace(const std::vector<CubeTarget>& cubes,
                                   const PickPlaceConfig& cfg = PickPlaceConfig()) {
  PickPlacePlan plan;
  plan.grasp_frame.assign(cubes.size(), -1);
  plan.release_frame.assign(cubes.size(), -1);
  plan.place_pos.assign(cubes.size(), Eigen::Vector3d::Zero());

  Vector6d q = homeJoints();
  plan.q.push_back(q);
  std::map<std::string, int> bin_count;

  auto move = [&](const Eigen::Matrix4d& T, const CartesianLimits& lim, const std::string& what) {
    const JointTrajectory tr = planCartesianMove(q, T, lim, cfg.dt);
    if (!tr.ok) {
      plan.error = "IK failed during '" + what + "' at sample " + std::to_string(tr.fail_index);
      return false;
    }
    plan.q.insert(plan.q.end(), tr.q.begin() + 1, tr.q.end());
    q = tr.q.back();
    return true;
  };
  auto dwell = [&]() { plan.q.insert(plan.q.end(), cfg.dwell_frames, q); };

  for (size_t i = 0; i < cubes.size(); ++i) {
    const CubeTarget& c = cubes[i];
    const auto bin = cfg.bins.find(c.color);
    if (bin == cfg.bins.end()) {
      plan.error = "no bin for colour '" + c.color + "'";
      return plan;
    }
    const int k = bin_count[c.color]++;
    Eigen::Vector3d place = bin->second;
    if (k > 0) place.y() += ((k % 2) ? 1.0 : -1.0) * 0.045 * ((k + 1) / 2);
    plan.place_pos[i] = place;

    const double yaw = foldCubeYaw(c.yaw);
    const Eigen::Vector3d pick(c.p.x(), c.p.y(), cfg.grasp_z);
    const Eigen::Vector3d pick_hover(c.p.x(), c.p.y(), cfg.hover_z);
    const Eigen::Vector3d place_hover(place.x(), place.y(), cfg.hover_z);
    const std::string tag = c.color + " cube " + std::to_string(i);

    if (!move(toolDownPose(pick_hover, yaw), cfg.fast, "hover over " + tag)) return plan;
    if (!move(toolDownPose(pick, yaw), cfg.slow, "descend to " + tag)) return plan;
    dwell();
    plan.grasp_frame[i] = static_cast<int>(plan.q.size());
    if (!move(toolDownPose(pick_hover, yaw), cfg.slow, "lift " + tag)) return plan;
    if (!move(toolDownPose(place_hover, 0.0), cfg.fast, "transport " + tag)) return plan;
    if (!move(toolDownPose(Eigen::Vector3d(place.x(), place.y(), cfg.grasp_z), 0.0), cfg.slow,
              "lower " + tag))
      return plan;
    dwell();
    plan.release_frame[i] = static_cast<int>(plan.q.size());
    if (!move(toolDownPose(place_hover, 0.0), cfg.slow, "retreat from " + tag)) return plan;
  }
  if (!move(forwardKinematics(homeJoints()), cfg.fast, "return home")) return plan;
  plan.ok = true;
  return plan;
}

}  // namespace sorting_cell
