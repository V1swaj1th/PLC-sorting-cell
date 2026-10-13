#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sorting_cell_controller/teach.hpp"

using namespace sorting_cell;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {
constexpr double kDt = 0.02;
constexpr double kStopTime = 0.30;   // ramp to zero on pause / stop [s]
constexpr double kStartTime = 0.50;  // ramp back up on resume [s]

std::vector<std::string> split(const std::string& s) {
  std::istringstream is(s);
  std::vector<std::string> out;
  std::string t;
  while (is >> t) out.push_back(t);
  return out;
}

bool validName(const std::string& n) {
  if (n.empty() || n.size() > 40) return false;
  for (char c : n)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) return false;
  return true;
}
}  // namespace

// Teach pendant: jog the end effector in Cartesian steps, save named waypoints, record a
// program (moves, gripper, dwell, home) and play it back. Commands arrive as text on
// teach/command (see teach_keyboard for a key-driven client); replies go to teach/status.
//
//   jog dx dy dz rx ry rz   Cartesian increment in the base frame [m, rad]
//   save NAME               store the current pose as a waypoint and append "move NAME"
//   gripper open|close      append a gripper step (and apply it)
//   dwell SECONDS           append a dwell step
//   addhome                 append a "home" step
//   undo                    remove the last step
//   goto NAME | home        move there now (not recorded)
//   delete NAME | new       delete a waypoint (if unused) | clear everything
//   list | steps | status   print waypoints | program | current pose
//   write PATH | load PATH  save / read the YAML program file
//   play | pause | resume | stop
class TeachPendant : public rclcpp::Node {
 public:
  enum class Mode { IDLE, MOVING, PLAYING };

  TeachPendant() : Node("teach_pendant") {
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("teach/status", 10);
    markers_pub_ = create_publisher<MarkerArray>("teach_markers", 10);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "teach/command", 50, [this](std_msgs::msg::String::SharedPtr m) { onCommand(m->data); });

    q_ = homeJoints();
    target_ = forwardKinematics(q_);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { onTimer(); });
    say("Teach pendant ready. Jog with the keyboard client or send commands to teach/command.");
  }

 private:
  // ------------------------------------------------------------ messages
  void say(const std::string& msg) {
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    std_msgs::msg::String s;
    s.data = msg;
    status_pub_->publish(s);
  }

  // ------------------------------------------------------------ commands
  void onCommand(const std::string& line) {
    const std::vector<std::string> t = split(line);
    if (t.empty()) return;
    const std::string& c = t[0];
    const bool moving = (mode_ != Mode::IDLE);

    if (c == "jog") {
      if (t.size() != 7) return say("usage: jog dx dy dz rx ry rz");
      if (mode_ == Mode::PLAYING) return say("jog ignored: a program is playing (stop it first)");
      Vector6d d;
      try {
        for (int i = 0; i < 6; ++i) d[i] = std::stod(t[i + 1]);
      } catch (...) {
        return say("jog: arguments must be numbers");
      }
      if (!d.allFinite() || d.head<3>().norm() > 0.2 || d.tail<3>().norm() > 1.0)
        return say("jog rejected: increment too large (max 0.2 m, 1 rad)");
      jog(d);
    } else if (c == "save") {
      if (t.size() != 2 || !validName(t[1])) return say("usage: save NAME  (letters, digits, _ and - only)");
      if (moving) return say("save refused: wait until the arm has stopped");
      const bool replaced = prog_.has(t[1]);
      prog_.set(t[1], waypointFromPose(forwardKinematics(q_)));
      Step s;
      s.type = StepType::MOVE;
      s.name = t[1];
      prog_.steps.push_back(s);
      std::ostringstream o;
      o << (replaced ? "Waypoint '" : "Saved waypoint '") << t[1] << "'" << (replaced ? " updated" : "")
        << " and appended step " << prog_.steps.size() << " (move " << t[1] << ")";
      say(o.str());
    } else if (c == "gripper") {
      if (t.size() != 2 || (t[1] != "open" && t[1] != "close")) return say("usage: gripper open|close");
      if (moving) return say("gripper refused: wait until the arm has stopped");
      Step s;
      s.type = StepType::GRIPPER;
      s.close = (t[1] == "close");
      prog_.steps.push_back(s);
      gripper_closed_ = s.close;
      say("Gripper " + t[1] + " (step " + std::to_string(prog_.steps.size()) + ")");
    } else if (c == "dwell") {
      double sec = 0.0;
      try {
        sec = (t.size() == 2) ? std::stod(t[1]) : -1.0;
      } catch (...) {
        sec = -1.0;
      }
      if (!(sec >= 0.0) || sec > 3600.0) return say("usage: dwell SECONDS  (0 to 3600)");
      if (moving) return say("dwell refused: wait until the arm has stopped");
      Step s;
      s.type = StepType::DWELL;
      s.seconds = sec;
      prog_.steps.push_back(s);
      say("Dwell " + t[1] + " s (step " + std::to_string(prog_.steps.size()) + ")");
    } else if (c == "addhome") {
      if (moving) return say("addhome refused: wait until the arm has stopped");
      Step s;
      s.type = StepType::HOME;
      prog_.steps.push_back(s);
      say("Home step appended (step " + std::to_string(prog_.steps.size()) + ")");
    } else if (c == "undo") {
      if (moving) return say("undo refused: wait until the arm has stopped");
      if (prog_.steps.empty()) return say("nothing to undo");
      prog_.steps.pop_back();
      say("Removed the last step; " + std::to_string(prog_.steps.size()) + " steps left");
    } else if (c == "goto") {
      if (t.size() != 2) return say("usage: goto NAME");
      if (moving) return say("goto refused: the arm is busy");
      const Waypoint* w = prog_.find(t[1]);
      if (!w) return say("unknown waypoint '" + t[1] + "'");
      startMove(poseMatrix(*w), "waypoint '" + t[1] + "'");
    } else if (c == "home") {
      if (moving) return say("home refused: the arm is busy");
      startMove(forwardKinematics(homeJoints()), "home");
    } else if (c == "delete") {
      if (t.size() != 2) return say("usage: delete NAME");
      if (moving) return say("delete refused: the arm is busy");
      if (!prog_.has(t[1])) return say("unknown waypoint '" + t[1] + "'");
      if (prog_.used(t[1])) return say("'" + t[1] + "' is used by the program; use undo first");
      prog_.erase(t[1]);
      say("Deleted waypoint '" + t[1] + "'");
    } else if (c == "new") {
      if (moving) return say("new refused: the arm is busy");
      prog_.clear();
      say("Cleared all waypoints and steps");
    } else if (c == "list") {
      listWaypoints();
    } else if (c == "steps") {
      listSteps();
    } else if (c == "status") {
      printStatus();
    } else if (c == "write") {
      if (t.size() != 2) return say("usage: write PATH");
      std::string err;
      if (saveProgram(prog_, t[1], err))
        say("Wrote " + std::to_string(prog_.names().size()) + " waypoints and " +
            std::to_string(prog_.steps.size()) + " steps to " + t[1]);
      else
        say("write failed: " + err);
    } else if (c == "load") {
      if (t.size() != 2) return say("usage: load PATH");
      if (moving) return say("load refused: the arm is busy");
      TeachProgram p;
      std::string err;
      if (!loadProgram(t[1], p, err)) return say("load failed: " + err);
      prog_ = p;
      say("Loaded '" + prog_.name + "': " + std::to_string(prog_.names().size()) + " waypoints, " +
          std::to_string(prog_.steps.size()) + " steps");
    } else if (c == "play") {
      play();
    } else if (c == "pause") {
      if (mode_ != Mode::PLAYING) return say("nothing is playing");
      paused_ = true;
      say("Paused");
    } else if (c == "resume") {
      if (mode_ != Mode::PLAYING || !paused_) return say("nothing to resume");
      paused_ = false;
      say("Resumed");
    } else if (c == "stop") {
      if (mode_ == Mode::IDLE) return say("nothing is moving");
      stopping_ = true;
      say("Stopping");
    } else {
      say("unknown command '" + c + "'");
    }
  }

  // ------------------------------------------------------------ jogging and moves
  void jog(const Vector6d& d) {
    const Eigen::Matrix4d next = applyJog(target_, d);
    if (mode_ == Mode::IDLE) {
      // plan right away, so an unreachable jog is rejected at once
      const JointTrajectory tr = planCartesianMove(q_, next, limits_, kDt);
      if (!tr.ok) return say("jog rejected: that pose is out of reach");
      target_ = next;
      beginSegment(tr.q);
      mode_ = Mode::MOVING;
    } else {
      // key repeat while moving: accumulate; the next segment is planned when this one ends
      target_ = next;
    }
  }

  void startMove(const Eigen::Matrix4d& T, const std::string& what) {
    const JointTrajectory tr = planCartesianMove(q_, T, limits_, kDt);
    if (!tr.ok) return say("cannot move to " + what + ": not reachable by a straight line");
    target_ = T;
    beginSegment(tr.q);
    mode_ = Mode::MOVING;
    say("Moving to " + what);
  }

  void beginSegment(const std::vector<Vector6d>& frames) {
    frames_ = frames;
    pos_ = 0.0;
    speed_ = 1.0;
  }

  void play() {
    if (mode_ != Mode::IDLE) return say("play refused: the arm is busy");
    const TeachPlan plan = compileProgram(prog_, q_);
    if (!plan.ok) return say("cannot play: " + plan.error);
    plan_ = plan;
    event_idx_ = 0;
    paused_ = stopping_ = false;
    beginSegment(plan.q);
    mode_ = Mode::PLAYING;
    say("Playing " + std::to_string(prog_.steps.size()) + " steps, " +
        fmt(static_cast<double>(plan.q.size()) * kDt) + " s");
  }

  // ------------------------------------------------------------ timer
  void onTimer() {
    if (mode_ != Mode::IDLE && !frames_.empty()) {
      const double target_speed = (paused_ || stopping_) ? 0.0 : 1.0;
      if (target_speed > speed_) speed_ = std::min(target_speed, speed_ + kDt / kStartTime);
      else speed_ = std::max(target_speed, speed_ - kDt / kStopTime);
      pos_ += speed_;
      const double last = static_cast<double>(frames_.size() - 1);
      if (stopping_ && speed_ <= 1e-9) {
        finishStopped();
      } else if (pos_ >= last) {
        pos_ = last;
        q_ = frames_.back();
        segmentDone();
      } else {
        const size_t i0 = static_cast<size_t>(pos_);
        const double f = pos_ - static_cast<double>(i0);
        q_ = (1.0 - f) * frames_[i0] + f * frames_[i0 + 1];
      }
      if (mode_ == Mode::PLAYING) applyGripperEvents();
    }
    publishJoints();
    if (++tick_ % 5 == 0) publishMarkers();
  }

  void applyGripperEvents() {
    const int frame = static_cast<int>(pos_);
    while (event_idx_ < plan_.gripper.size() && plan_.gripper[event_idx_].first <= frame) {
      gripper_closed_ = plan_.gripper[event_idx_].second;
      ++event_idx_;
    }
  }

  void finishStopped() {
    frames_.clear();
    speed_ = 0.0;
    stopping_ = paused_ = false;
    mode_ = Mode::IDLE;
    target_ = forwardKinematics(q_);
    say("Stopped");
  }

  void segmentDone() {
    frames_.clear();
    speed_ = 0.0;
    if (mode_ == Mode::PLAYING) {
      event_idx_ = plan_.gripper.size();
      for (const auto& e : plan_.gripper) gripper_closed_ = e.second;
      mode_ = Mode::IDLE;
      target_ = forwardKinematics(q_);
      say("Program finished");
      return;
    }
    // accumulated jogs: plan the rest of the way
    const Eigen::Matrix4d now_T = forwardKinematics(q_);
    if (!poseNear(now_T, target_, 1e-6, 1e-6)) {
      const JointTrajectory tr = planCartesianMove(q_, target_, limits_, kDt);
      if (tr.ok && tr.q.size() > 1) {
        beginSegment(tr.q);
        return;
      }
      target_ = now_T;
      say("jog stopped: the rest of the move is out of reach");
    }
    mode_ = Mode::IDLE;
  }

  // ------------------------------------------------------------ output
  static std::string fmt(double v, int prec = 3) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
  }

  void listWaypoints() {
    if (prog_.names().empty()) return say("No waypoints yet. Jog the arm and press save.");
    std::ostringstream o;
    o << prog_.names().size() << " waypoints:";
    for (const std::string& n : prog_.names()) {
      const Waypoint* w = prog_.find(n);
      o << "\n  " << n << "  [" << fmt(w->p.x()) << ", " << fmt(w->p.y()) << ", " << fmt(w->p.z()) << "]";
    }
    say(o.str());
  }

  void listSteps() {
    if (prog_.steps.empty()) return say("The program has no steps yet.");
    std::ostringstream o;
    o << prog_.steps.size() << " steps:";
    int i = 0;
    for (const Step& s : prog_.steps) {
      o << "\n  " << ++i << ". ";
      switch (s.type) {
        case StepType::MOVE: o << "move " << s.name << (s.fast ? " (fast)" : ""); break;
        case StepType::DWELL: o << "dwell " << fmt(s.seconds, 2) << " s"; break;
        case StepType::HOME: o << "home"; break;
        case StepType::GRIPPER: o << "gripper " << (s.close ? "close" : "open"); break;
      }
    }
    say(o.str());
  }

  void printStatus() {
    const Eigen::Matrix4d T = forwardKinematics(q_);
    const double tilt =
        std::acos(std::max(-1.0, std::min(1.0, -T(2, 2)))) * 180.0 / M_PI;  // tool z vs straight down
    std::ostringstream o;
    o << "pose [" << fmt(T(0, 3)) << ", " << fmt(T(1, 3)) << ", " << fmt(T(2, 3)) << "] m, tool yaw "
      << fmt(toolYaw(T) * 180.0 / M_PI, 1) << " deg, tilt " << fmt(tilt, 1) << " deg, gripper "
      << (gripper_closed_ ? "closed" : "open") << ", "
      << (mode_ == Mode::IDLE ? "idle" : mode_ == Mode::MOVING ? "moving" : (paused_ ? "paused" : "playing"))
      << ", " << prog_.names().size() << " waypoints, " << prog_.steps.size() << " steps";
    say(o.str());
  }

  void publishJoints() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
    msg.position.assign(q_.data(), q_.data() + 6);
    js_pub_->publish(msg);
  }

  Marker base(int id, const std::string& ns, int type) const {
    Marker m;
    m.header.frame_id = "base";
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.6);  // markers of deleted waypoints expire
    return m;
  }

  void publishMarkers() {
    MarkerArray arr;
    Marker table = base(0, "table", Marker::CUBE);
    table.pose.position.x = -0.50;
    table.pose.position.z = -0.001;
    table.scale.x = 0.60;
    table.scale.y = 0.90;
    table.scale.z = 0.002;
    table.color.r = table.color.g = table.color.b = 0.37f;
    table.color.a = 1.0f;
    arr.markers.push_back(table);

    int id = 0;
    for (const std::string& n : prog_.names()) {
      const Waypoint* w = prog_.find(n);
      Marker s = base(id, "waypoints", Marker::SPHERE);
      s.pose.position.x = w->p.x();
      s.pose.position.y = w->p.y();
      s.pose.position.z = w->p.z();
      s.scale.x = s.scale.y = s.scale.z = 0.025;
      s.color.r = 0.16f; s.color.g = 0.47f; s.color.b = 0.84f; s.color.a = 1.0f;
      arr.markers.push_back(s);
      Marker label = base(id, "labels", Marker::TEXT_VIEW_FACING);
      label.pose.position.x = w->p.x();
      label.pose.position.y = w->p.y();
      label.pose.position.z = w->p.z() + 0.045;
      label.scale.z = 0.035;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = n;
      arr.markers.push_back(label);
      ++id;
    }

    // program path through the recorded moves
    Marker path = base(0, "path", Marker::LINE_STRIP);
    path.scale.x = 0.004;
    path.color.r = 0.92f; path.color.g = 0.41f; path.color.b = 0.20f; path.color.a = 0.9f;
    const Eigen::Vector3d home_p = forwardKinematics(homeJoints()).block<3, 1>(0, 3);
    auto pt = [](const Eigen::Vector3d& p) {
      geometry_msgs::msg::Point g;
      g.x = p.x(); g.y = p.y(); g.z = p.z();
      return g;
    };
    path.points.push_back(pt(home_p));
    for (const Step& s : prog_.steps) {
      if (s.type == StepType::HOME) path.points.push_back(pt(home_p));
      if (s.type == StepType::MOVE) {
        const Waypoint* w = prog_.find(s.name);
        if (w) path.points.push_back(pt(w->p));
      }
    }
    if (path.points.size() > 1) arr.markers.push_back(path);

    // tool marker: red = gripper open, green = closed
    const Eigen::Matrix4d T = forwardKinematics(q_);
    Marker tool = base(0, "tool", Marker::SPHERE);
    tool.pose.position.x = T(0, 3);
    tool.pose.position.y = T(1, 3);
    tool.pose.position.z = T(2, 3);
    tool.scale.x = tool.scale.y = tool.scale.z = 0.035;
    if (gripper_closed_) { tool.color.r = 0.20f; tool.color.g = 0.75f; tool.color.b = 0.25f; }
    else { tool.color.r = 0.88f; tool.color.g = 0.20f; tool.color.b = 0.20f; }
    tool.color.a = 1.0f;
    arr.markers.push_back(tool);
    markers_pub_->publish(arr);
  }

  // ------------------------------------------------------------ state
  Mode mode_{Mode::IDLE};
  TeachProgram prog_;
  TeachPlan plan_;
  size_t event_idx_{0};
  CartesianLimits limits_;  // jog and goto use the slow limits (0.25 m/s)
  Vector6d q_;
  Eigen::Matrix4d target_;
  std::vector<Vector6d> frames_;
  double pos_{0.0}, speed_{0.0};
  bool paused_{false}, stopping_{false}, gripper_closed_{false};
  unsigned long tick_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeachPendant>());
  rclcpp::shutdown();
  return 0;
}
