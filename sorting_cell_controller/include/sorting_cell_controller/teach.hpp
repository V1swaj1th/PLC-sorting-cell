#pragma once
// Teach-and-playback core (ROS-free): waypoints, program steps, YAML file format, jogging math
// and the program compiler that turns a program into one continuous joint trajectory.
//
// Program file (YAML):
//   version: 1
//   name: demo
//   waypoints:
//     pick_a:
//       position: [-0.40, -0.20, 0.20]            # tool0 in the base frame [m]
//       orientation_xyzw: [0.0, 1.0, 0.0, 0.0]    # quaternion in the base frame
//   steps:
//     - move: pick_a            # straight line in Cartesian space; optional "speed: fast"
//     - gripper: close          # open | close (visual only)
//     - dwell: 0.5              # seconds
//     - home: true              # straight line back to the home pose
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sorting_cell_controller/path_util.hpp"
#include "sorting_cell_controller/pick_place.hpp"
#include "sorting_cell_controller/trajectory.hpp"

namespace sorting_cell {

struct Waypoint {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};

enum class StepType { MOVE, DWELL, HOME, GRIPPER };

struct Step {
  StepType type{StepType::MOVE};
  std::string name;      // MOVE: waypoint name
  double seconds{0.0};   // DWELL
  bool close{false};     // GRIPPER: true = close
  bool fast{false};      // MOVE: use the fast limits
};

class TeachProgram {
 public:
  std::string name{"program"};
  std::vector<Step> steps;

  bool has(const std::string& n) const { return wps_.count(n) > 0; }
  const Waypoint* find(const std::string& n) const {
    const auto it = wps_.find(n);
    return it == wps_.end() ? nullptr : &it->second;
  }
  // Adds or replaces; a new name goes to the end of the listing order.
  void set(const std::string& n, const Waypoint& w) {
    if (!has(n)) order_.push_back(n);
    wps_[n] = w;
  }
  bool erase(const std::string& n) {
    if (!has(n)) return false;
    wps_.erase(n);
    order_.erase(std::remove(order_.begin(), order_.end(), n), order_.end());
    return true;
  }
  bool used(const std::string& n) const {
    for (const Step& s : steps)
      if (s.type == StepType::MOVE && s.name == n) return true;
    return false;
  }
  const std::vector<std::string>& names() const { return order_; }
  void clear() {
    wps_.clear();
    order_.clear();
    steps.clear();
  }

 private:
  std::map<std::string, Waypoint> wps_;
  std::vector<std::string> order_;
};

// ---------------------------------------------------------------- poses and jogging

inline Eigen::Matrix4d poseMatrix(const Waypoint& w) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = w.q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = w.p;
  return T;
}

inline Waypoint waypointFromPose(const Eigen::Matrix4d& T) {
  Waypoint w;
  w.p = T.block<3, 1>(0, 3);
  w.q = Eigen::Quaterniond(Eigen::Matrix3d(T.block<3, 3>(0, 0))).normalized();
  if (w.q.w() < 0.0) w.q.coeffs() *= -1.0;  // canonical sign, so files do not flip signs
  return w;
}

// Jog increment d = (dx, dy, dz, rx, ry, rz): a translation and a rotation vector, both
// expressed in the base frame and applied to the current pose.
inline Eigen::Matrix4d applyJog(const Eigen::Matrix4d& T, const Vector6d& d) {
  Eigen::Matrix4d out = T;
  out.block<3, 1>(0, 3) += d.head<3>();
  const Eigen::Vector3d r = d.tail<3>();
  const double a = r.norm();
  if (a > 1e-12) {
    const Eigen::Matrix3d R = Eigen::AngleAxisd(a, r / a).toRotationMatrix() *
                              Eigen::Matrix3d(T.block<3, 3>(0, 0));
    out.block<3, 3>(0, 0) = Eigen::Quaterniond(R).normalized().toRotationMatrix();
  }
  return out;
}

inline bool poseNear(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B, double pos_tol = 1e-6,
                     double rot_tol = 1e-6) {
  const double dp = (A.block<3, 1>(0, 3) - B.block<3, 1>(0, 3)).norm();
  const Eigen::Quaterniond qa(Eigen::Matrix3d(A.block<3, 3>(0, 0)));
  const Eigen::Quaterniond qb(Eigen::Matrix3d(B.block<3, 3>(0, 0)));
  return dp <= pos_tol && qa.angularDistance(qb) <= rot_tol;
}

// ---------------------------------------------------------------- YAML

inline double cleanNumber(double v) { return std::abs(v) < 1e-12 ? 0.0 : v; }

inline std::string programToYaml(const TeachProgram& prog) {
  YAML::Emitter out;
  out.SetDoublePrecision(12);
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 1;
  out << YAML::Key << "name" << YAML::Value << prog.name;
  out << YAML::Key << "waypoints" << YAML::Value;
  if (prog.names().empty()) {
    out << YAML::Flow << YAML::BeginMap << YAML::EndMap;
  } else {
    out << YAML::BeginMap;
    for (const std::string& n : prog.names()) {
      const Waypoint* w = prog.find(n);
      out << YAML::Key << n << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << cleanNumber(w->p.x()) << cleanNumber(w->p.y()) << cleanNumber(w->p.z()) << YAML::EndSeq;
      out << YAML::Key << "orientation_xyzw" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << cleanNumber(w->q.x()) << cleanNumber(w->q.y()) << cleanNumber(w->q.z())
          << cleanNumber(w->q.w()) << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::Key << "steps" << YAML::Value;
  if (prog.steps.empty()) {
    out << YAML::Flow << YAML::BeginSeq << YAML::EndSeq;
  } else {
    out << YAML::BeginSeq;
    for (const Step& s : prog.steps) {
      out << YAML::BeginMap;
      switch (s.type) {
        case StepType::MOVE:
          out << YAML::Key << "move" << YAML::Value << s.name;
          if (s.fast) out << YAML::Key << "speed" << YAML::Value << "fast";
          break;
        case StepType::DWELL:
          out << YAML::Key << "dwell" << YAML::Value << s.seconds;
          break;
        case StepType::HOME:
          out << YAML::Key << "home" << YAML::Value << true;
          break;
        case StepType::GRIPPER:
          out << YAML::Key << "gripper" << YAML::Value << (s.close ? "close" : "open");
          break;
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

inline bool programFromYaml(const std::string& text, TeachProgram& out, std::string& err) {
  try {
    const YAML::Node root = YAML::Load(text);
    if (!root.IsMap()) {
      err = "the file is not a YAML mapping";
      return false;
    }
    if (root["version"] && root["version"].as<int>() != 1) {
      err = "unsupported version " + root["version"].as<std::string>();
      return false;
    }
    TeachProgram prog;
    if (root["name"]) prog.name = root["name"].as<std::string>();

    if (root["waypoints"] && !root["waypoints"].IsNull()) {
      if (!root["waypoints"].IsMap()) {
        err = "'waypoints' must be a mapping";
        return false;
      }
      for (const auto& kv : root["waypoints"]) {
        const std::string n = kv.first.as<std::string>();
        const YAML::Node w = kv.second;
        if (!w.IsMap() || !w["position"] || !w["position"].IsSequence() || w["position"].size() != 3) {
          err = "waypoint '" + n + "': 'position' must be [x, y, z]";
          return false;
        }
        Waypoint wp;
        wp.p = Eigen::Vector3d(w["position"][0].as<double>(), w["position"][1].as<double>(),
                               w["position"][2].as<double>());
        if (!w["orientation_xyzw"] || !w["orientation_xyzw"].IsSequence() ||
            w["orientation_xyzw"].size() != 4) {
          err = "waypoint '" + n + "': 'orientation_xyzw' must be [x, y, z, w]";
          return false;
        }
        const YAML::Node o = w["orientation_xyzw"];
        Eigen::Quaterniond q(o[3].as<double>(), o[0].as<double>(), o[1].as<double>(),
                             o[2].as<double>());
        if (q.norm() < 1e-9) {
          err = "waypoint '" + n + "': orientation has zero length";
          return false;
        }
        wp.q = q.normalized();
        prog.set(n, wp);
      }
    }

    if (root["steps"] && !root["steps"].IsNull()) {
      if (!root["steps"].IsSequence()) {
        err = "'steps' must be a list";
        return false;
      }
      int idx = 0;
      for (const YAML::Node& sn : root["steps"]) {
        ++idx;
        if (!sn.IsMap() || sn.size() < 1) {
          err = "step " + std::to_string(idx) + ": must be a mapping such as 'move: name'";
          return false;
        }
        Step s;
        if (sn["move"]) {
          s.type = StepType::MOVE;
          s.name = sn["move"].as<std::string>();
          if (sn["speed"]) {
            const std::string sp = sn["speed"].as<std::string>();
            if (sp != "fast" && sp != "slow") {
              err = "step " + std::to_string(idx) + ": speed must be 'slow' or 'fast'";
              return false;
            }
            s.fast = (sp == "fast");
          }
        } else if (sn["dwell"]) {
          s.type = StepType::DWELL;
          s.seconds = sn["dwell"].as<double>();
          if (!(s.seconds >= 0.0) || s.seconds > 3600.0) {
            err = "step " + std::to_string(idx) + ": dwell must be between 0 and 3600 s";
            return false;
          }
        } else if (sn["home"]) {
          s.type = StepType::HOME;
        } else if (sn["gripper"]) {
          s.type = StepType::GRIPPER;
          const std::string g = sn["gripper"].as<std::string>();
          if (g != "open" && g != "close") {
            err = "step " + std::to_string(idx) + ": gripper must be 'open' or 'close'";
            return false;
          }
          s.close = (g == "close");
        } else {
          err = "step " + std::to_string(idx) + ": expected one of move, dwell, home, gripper";
          return false;
        }
        prog.steps.push_back(s);
      }
    }
    out = prog;
    return true;
  } catch (const YAML::Exception& e) {
    err = std::string("YAML error: ") + e.what();
    return false;
  }
}

inline bool saveProgram(const TeachProgram& prog, const std::string& path, std::string& err) {
  std::ofstream f(expandUser(path));
  if (!f) {
    err = "cannot write '" + path + "'";
    return false;
  }
  f << programToYaml(prog);
  f.flush();
  if (!f) {
    err = "write to '" + path + "' failed";
    return false;
  }
  return true;
}

inline bool loadProgram(const std::string& path, TeachProgram& out, std::string& err) {
  std::ifstream f(expandUser(path));
  if (!f) {
    err = "cannot read '" + path + "'";
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return programFromYaml(ss.str(), out, err);
}

// ---------------------------------------------------------------- compiler

struct TeachConfig {
  double dt = 0.02;
  CartesianLimits slow;  // default 0.25 m/s
  CartesianLimits fast;
  TeachConfig() {
    fast.v_lin = 0.40;
    fast.a_lin = 0.80;
    fast.v_ang = 1.5;
    fast.a_ang = 3.0;
  }
};

struct TeachPlan {
  bool ok{false};
  std::string error;
  int fail_step{-1};                           // 1-based step number of the failure
  std::vector<Vector6d> q;                     // one joint vector per frame
  std::vector<int> step_end_frame;             // last frame of each step
  std::vector<std::pair<int, bool>> gripper;   // (frame, closed): state is applied from that frame on
};

// Plans the whole program from q_start before anything moves, so a bad waypoint is reported
// with its step number instead of failing half way through.
inline TeachPlan compileProgram(const TeachProgram& prog, const Vector6d& q_start,
                                const TeachConfig& cfg = TeachConfig()) {
  TeachPlan plan;
  if (prog.steps.empty()) {
    plan.error = "the program has no steps";
    return plan;
  }
  Vector6d q = q_start;
  plan.q.push_back(q);
  const Eigen::Matrix4d T_home = forwardKinematics(homeJoints());

  for (size_t i = 0; i < prog.steps.size(); ++i) {
    const Step& s = prog.steps[i];
    const std::string tag = "step " + std::to_string(i + 1);
    if (s.type == StepType::MOVE || s.type == StepType::HOME) {
      Eigen::Matrix4d T;
      if (s.type == StepType::HOME) {
        T = T_home;
      } else {
        const Waypoint* w = prog.find(s.name);
        if (!w) {
          plan.fail_step = static_cast<int>(i + 1);
          plan.error = tag + ": unknown waypoint '" + s.name + "'";
          return plan;
        }
        T = poseMatrix(*w);
      }
      const JointTrajectory tr =
          planCartesianMove(q, T, s.fast ? cfg.fast : cfg.slow, cfg.dt);
      if (!tr.ok) {
        plan.fail_step = static_cast<int>(i + 1);
        plan.error = tag + ": " + (s.type == StepType::HOME ? "home" : "'" + s.name + "'") +
                     " is not reachable by a straight line from the previous pose (IK failed at sample " +
                     std::to_string(tr.fail_index) + ")";
        return plan;
      }
      plan.q.insert(plan.q.end(), tr.q.begin() + 1, tr.q.end());
      q = tr.q.back();
    } else if (s.type == StepType::DWELL) {
      const int n = static_cast<int>(std::ceil(s.seconds / cfg.dt - 1e-9));
      plan.q.insert(plan.q.end(), std::max(n, 0), q);
    } else {  // GRIPPER: takes effect after a short settle time
      plan.gripper.emplace_back(static_cast<int>(plan.q.size()) - 1, s.close);
      plan.q.insert(plan.q.end(), static_cast<size_t>(std::round(0.3 / cfg.dt)), q);
    }
    plan.step_end_frame.push_back(static_cast<int>(plan.q.size()) - 1);
  }
  plan.ok = true;
  return plan;
}

}  // namespace sorting_cell
