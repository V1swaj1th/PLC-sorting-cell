#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {
constexpr double kCubeSize = kCubeSizeM;
constexpr double kBinSize = kBinSizeM;
constexpr double kDt = 0.02;          // timer period [s]
constexpr double kStopTime = 0.30;    // time to ramp the motion down to zero on stop / E-stop [s]
constexpr double kStartTime = 0.50;   // time to ramp back up on resume [s]

// "rev" and "cubes" from the vision node's detections message (JSON, which YAML reads).
bool parseDetections(const std::string& text, int& rev, std::vector<CubeTarget>& out) {
  try {
    const YAML::Node n = YAML::Load(text);
    rev = n["rev"].as<int>();
    out.clear();
    for (const auto& c : n["cubes"])
      out.push_back({c["color"].as<std::string>(), Eigen::Vector3d(c["x"].as<double>(), c["y"].as<double>(), kCubeSize / 2),
                     c["yaw"].as<double>()});
    return true;
  } catch (const YAML::Exception&) {
    return false;
  }
}
}  // namespace

// Receives the scene (containers) from the scene server and the detected cubes from the vision
// node, plans the whole sort with the own FK/IK/trapezoidal planner, then plays it back at 50 Hz
// on joint_states. A cube goes to the container of its colour, or to the gray container.
//
// use_plc = false: starts on cell/start, or by itself after start_delay_s when auto_start is true.
// use_plc = true : moves only while the PLC says run && !estop. The motion is ramped to zero
//                  in 0.3 s when permission is removed and ramped back up on resume. After an
//                  E-stop the PLC reset must be acknowledged before motion is allowed again.
class SortingSequencer : public rclcpp::Node {
 public:
  enum class State { WAITING, RUNNING, DONE, FAULT };

  SortingSequencer() : Node("sorting_sequencer") {
    declare_parameter("start_delay_s", 3.0);
    declare_parameter("use_plc", false);
    declare_parameter("auto_start", true);
    auto_start_ = get_parameter("auto_start").as_bool();
    start_delay_ = get_parameter("start_delay_s").as_double();
    use_plc_ = get_parameter("use_plc").as_bool();
    if (use_plc_) estop_ = true;  // fail-safe until the PLC says otherwise

    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    markers_pub_ = create_publisher<MarkerArray>("sorted_cubes", 10);
    busy_pub_ = create_publisher<std_msgs::msg::Bool>("arm_busy", 10);
    home_pub_ = create_publisher<std_msgs::msg::Bool>("at_home", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("cycle_done", 10);
    fault_pub_ = create_publisher<std_msgs::msg::Bool>("fault", 10);
    phase_pub_ = create_publisher<std_msgs::msg::String>("arm_phase", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("sequencer/state", 10);

    scene_sub_ = create_subscription<std_msgs::msg::String>(
        "scene/config", rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::SharedPtr m) { onScene(m->data); });
    det_sub_ = create_subscription<std_msgs::msg::String>(
        "vision/detections", 10, [this](std_msgs::msg::String::SharedPtr m) { onDetections(m->data); });
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
        "cell/start", 10, [this](std_msgs::msg::Empty::SharedPtr) { onStartRequest(); });
    run_sub_ = create_subscription<std_msgs::msg::Bool>(
        "plc/run", 10, [this](std_msgs::msg::Bool::SharedPtr m) { run_ = m->data; });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "plc/estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
          estop_ = m->data;
          if (estop_) estop_seen_ = true;
          handleReset();
        });
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "plc/reset", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
          reset_level_ = m->data;
          if (!reset_level_) reset_handled_ = false;
          handleReset();
        });

    q_ = homeJoints();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "sorting_sequencer started (use_plc=%s, auto_start=%s), waiting for scene/config and vision/detections",
                use_plc_ ? "true" : "false", auto_start_ ? "true" : "false");
  }

 private:
  void onScene(const std::string& text) {
    Scene sc;
    std::string err;
    if (!sceneFromYaml(text, sc, err)) {
      RCLCPP_ERROR(get_logger(), "ignoring scene/config: %s", err.c_str());
      return;
    }
    const bool changed = !have_scene_ || sc.revision != scene_.revision;
    scene_ = sc;
    have_scene_ = true;
    if (!changed) return;
    det_valid_ = false;  // detections of the old scene no longer count
    RCLCPP_INFO(get_logger(), "scene revision %d: %zu containers, gray container at (%.2f, %.2f)", scene_.revision,
                scene_.containers.size(), scene_.gray_x, scene_.gray_y);
    if (moving()) {
      RCLCPP_WARN(get_logger(), "scene changed while the arm is moving: it applies after this cycle");
      return;
    }
    if (!use_plc_ || state_ != State::DONE) discardPlan();  // with a PLC, a finished cycle waits for its reset
  }

  void onDetections(const std::string& text) {
    int rev = -1;
    std::vector<CubeTarget> cubes;
    if (!parseDetections(text, rev, cubes)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring malformed vision/detections");
      return;
    }
    if (!have_scene_ || rev != scene_.revision) return;  // vision has not caught up with the scene yet
    det_cubes_ = cubes;
    det_valid_ = true;
    tryPlan();
  }

  bool moving() const { return state_ == State::RUNNING && (pos_ > 1e-9 || speed_ > 1e-9); }

  // Drops a plan that has not started, or that finished, so the next detections are planned afresh.
  void discardPlan() {
    if (state_ == State::RUNNING || state_ == State::FAULT || state_ == State::DONE) {
      state_ = State::WAITING;
      started_ = false;
      pos_ = speed_ = 0.0;
      q_ = homeJoints();
    }
  }

  // cell/start: begin (or repeat) the sort. Only meaningful without a PLC; with one, the PLC start button decides.
  void onStartRequest() {
    if (use_plc_) {
      RCLCPP_WARN(get_logger(), "cell/start ignored: the PLC controls the start (use panel/start)");
      return;
    }
    if (moving()) return;
    if (state_ == State::DONE) discardPlan();  // run the same scene again
    if (state_ == State::FAULT) {
      RCLCPP_WARN(get_logger(), "cell/start ignored: the scene could not be planned");
      return;
    }
    if (state_ == State::RUNNING) {
      started_ = true;
      start_pending_ = false;
    } else {
      start_pending_ = true;  // applies as soon as the plan exists
      tryPlan();
    }
  }

  void tryPlan() {
    if (state_ != State::WAITING || !have_scene_ || !det_valid_ || det_cubes_.empty()) return;
    if (det_cubes_.size() != scene_.objects.size())
      RCLCPP_WARN(get_logger(), "vision found %zu of %zu objects: planning the ones it found", det_cubes_.size(),
                  scene_.objects.size());
    cubes_ = det_cubes_;
    const auto t0 = std::chrono::steady_clock::now();
    plan_ = planPickPlace(cubes_, planConfig(scene_));
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!plan_.ok) {
      state_ = State::FAULT;
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", plan_.error.c_str());
      return;
    }
    pos_ = 0.0;
    speed_ = 0.0;
    started_ = start_pending_;
    start_pending_ = false;
    planned_rev_ = scene_.revision;
    plan_time_ = now();
    state_ = State::RUNNING;
    std::string dests;
    for (size_t i = 0; i < cubes_.size(); ++i) dests += (i ? ", " : "") + cubes_[i].color + "->" + plan_.dest[i];
    RCLCPP_INFO(get_logger(), "Planned %zu cubes (%s): %zu frames (%.1f s) in %.0f ms. %s", cubes_.size(), dests.c_str(),
                plan_.q.size(), static_cast<double>(plan_.q.size()) * kDt, ms,
                use_plc_ ? "Waiting for PLC run."
                         : (started_ ? "Starting now."
                                     : (auto_start_ ? "Starting after the start delay." : "Waiting for cell/start.")));
  }

  // A reset push is handled once per push, as soon as the E-stop signal is clear. The PLC can
  // report reset and "E-stop latched" together for one scan, so this must look at the level
  // of both signals and not only at the edge of the reset pulse.
  void handleReset() {
    if (!reset_level_ || reset_handled_ || estop_) return;
    reset_handled_ = true;
    onReset();
  }

  // Reset from the PLC: acknowledge an E-stop; after a finished or failed cycle, start over.
  void onReset() {
    if (estop_seen_) {
      estop_seen_ = false;
      RCLCPP_INFO(get_logger(), "E-stop acknowledged by reset");
    }
    if (state_ == State::DONE || state_ == State::FAULT) {
      RCLCPP_INFO(get_logger(), "Reset: re-homing and waiting for a new cube scan");
      discardPlan();
      det_valid_ = false;  // wait for fresh detections
    }
  }

  bool motionAllowed() const {
    if (state_ != State::RUNNING) return false;
    if (use_plc_) return run_ && !estop_ && !estop_seen_;
    if (started_) return true;
    return auto_start_ && (now() - plan_time_).seconds() >= start_delay_;
  }

  void onTimer() {
    if (state_ == State::RUNNING) {
      const bool allowed = motionAllowed();
      if (use_plc_ && run_ && !allowed && pos_ < 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "PLC run is on but motion is held: plc/estop=%d, E-stop not yet acknowledged by a reset=%d",
                             static_cast<int>(estop_), static_cast<int>(estop_seen_));
      }
      const double target = allowed ? 1.0 : 0.0;
      if (target > speed_) speed_ = std::min(target, speed_ + kDt / kStartTime);
      else speed_ = std::max(target, speed_ - kDt / kStopTime);
      if (speed_ > 0.0) pos_ += speed_;  // 1 frame per 20 ms tick at full speed
      const double last = static_cast<double>(plan_.q.size() - 1);
      if (pos_ >= last) {
        pos_ = last;
        speed_ = 0.0;
        state_ = State::DONE;
        RCLCPP_INFO(get_logger(), "Cycle done: %zu cubes sorted", cubes_.size());
        if (!use_plc_ && scene_.revision != planned_rev_) discardPlan();  // edited during the cycle
      }
      const size_t i0 = static_cast<size_t>(pos_);
      const size_t i1 = std::min(i0 + 1, plan_.q.size() - 1);
      const double f = pos_ - static_cast<double>(i0);
      q_ = (1.0 - f) * plan_.q[i0] + f * plan_.q[i1];
    }
    publishJoints();
    if (++tick_ % 2 == 0) publishMarkers();
    if (tick_ % 5 == 0) publishStatus();
  }

  std::string phase() const {
    if (state_ != State::RUNNING || pos_ < 1.0) return "idle";
    const int f = static_cast<int>(pos_);
    for (size_t i = 0; i < cubes_.size(); ++i) {
      if (f < plan_.release_frame[i]) return (f < plan_.grasp_frame[i]) ? "pick" : "place";
    }
    return "idle";
  }

  void publishJoints() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
    msg.position.assign(q_.data(), q_.data() + 6);
    js_pub_->publish(msg);
  }

  void publishStatus() {
    std_msgs::msg::Bool busy, home, done, fault;
    busy.data = (state_ == State::RUNNING) && speed_ > 1e-3;
    home.data = (state_ == State::DONE) || (state_ == State::WAITING) ||
                (state_ == State::RUNNING && pos_ < 0.5);
    done.data = (state_ == State::DONE);
    fault.data = (state_ == State::FAULT);
    busy_pub_->publish(busy);
    home_pub_->publish(home);
    done_pub_->publish(done);
    fault_pub_->publish(fault);
    std_msgs::msg::String ph;
    ph.data = phase();
    phase_pub_->publish(ph);
    std_msgs::msg::String st;
    switch (state_) {
      case State::WAITING: st.data = "waiting"; break;
      case State::RUNNING: st.data = moving() ? "running" : "ready"; break;
      case State::DONE: st.data = "done"; break;
      case State::FAULT: st.data = "fault"; break;
    }
    state_pub_->publish(st);
  }

  Marker box(const std::string& ns, int id, double sx, double sy, double sz, double x, double y,
             double z, double yaw, const std::string& color, float alpha) const {
    Marker m;
    m.header.frame_id = "base";
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = Marker::CUBE;
    m.action = Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    colorRgb(color, m.color.r, m.color.g, m.color.b);
    m.color.a = alpha;
    m.lifetime = rclcpp::Duration::from_seconds(0.6);  // markers of removed objects expire
    return m;
  }

  void publishMarkers() {
    MarkerArray arr;
    Marker table = box("table", 0, 0.60, 0.90, 0.002, -0.50, 0.0, -0.001, 0.0, "", 1.0f);
    table.color.r = table.color.g = table.color.b = 0.37f;
    arr.markers.push_back(table);
    int id = 0;
    if (have_scene_) {
      for (const auto& c : scene_.containers)
        arr.markers.push_back(box("bins", id++, kBinSize, kBinSize, 0.004, c.x, c.y, 0.002, 0.0, c.color, 0.55f));
      arr.markers.push_back(
          box("bins", id++, kBinSize, kBinSize, 0.004, scene_.gray_x, scene_.gray_y, 0.002, 0.0, kGrayName, 0.55f));
      Marker label = box("labels", 0, 0.0, 0.0, 0.03, scene_.gray_x, scene_.gray_y, 0.07, 0.0, kGrayName, 1.0f);
      label.type = Marker::TEXT_VIEW_FACING;
      label.text = "no match";
      label.color.r = label.color.g = label.color.b = 0.9f;
      arr.markers.push_back(label);
    }
    if (state_ == State::WAITING || state_ == State::FAULT) {
      if (state_ == State::WAITING && det_valid_) {  // show what vision found while the scene is being edited
        for (size_t i = 0; i < det_cubes_.size(); ++i)
          arr.markers.push_back(box("cubes", static_cast<int>(i), kCubeSize, kCubeSize, kCubeSize, det_cubes_[i].p.x(),
                                    det_cubes_[i].p.y(), kCubeSize / 2.0, det_cubes_[i].yaw, det_cubes_[i].color, 1.0f));
      }
      markers_pub_->publish(arr);
      return;
    }
    const int frame = static_cast<int>(pos_);
    for (size_t i = 0; i < cubes_.size(); ++i) {
      double x, y, z, yaw;
      if (plan_.release_frame[i] >= 0 && frame >= plan_.release_frame[i]) {
        x = plan_.place_pos[i].x();
        y = plan_.place_pos[i].y();
        z = kCubeSize / 2.0;
        yaw = 0.0;
      } else if (plan_.grasp_frame[i] >= 0 && frame >= plan_.grasp_frame[i]) {
        const Eigen::Matrix4d T = forwardKinematics(q_);
        x = T(0, 3);
        y = T(1, 3);
        z = T(2, 3);
        yaw = toolYaw(T);
      } else {
        x = cubes_[i].p.x();
        y = cubes_[i].p.y();
        z = kCubeSize / 2.0;
        yaw = cubes_[i].yaw;
      }
      arr.markers.push_back(box("cubes", static_cast<int>(i), kCubeSize, kCubeSize, kCubeSize, x, y, z,
                                yaw, cubes_[i].color, 1.0f));
    }
    markers_pub_->publish(arr);
  }

  State state_{State::WAITING};
  double start_delay_{3.0};
  bool use_plc_{false};
  bool run_{false}, estop_{false}, estop_seen_{false};
  bool reset_level_{false}, reset_handled_{false};
  bool auto_start_{true}, started_{false}, start_pending_{false};
  bool have_scene_{false}, det_valid_{false};
  int planned_rev_{-1};
  Scene scene_;
  std::vector<CubeTarget> det_cubes_;  // latest detections that belong to the current scene
  std::vector<CubeTarget> cubes_;      // the ones in the running plan
  PickPlacePlan plan_;
  Vector6d q_;
  double pos_{0.0};    // fractional frame index into plan_.q
  double speed_{0.0};  // 0..1 playback speed (ramped)
  unsigned long tick_{0};
  rclcpp::Time plan_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_pub_, home_pub_, done_pub_, fault_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_, state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_sub_, det_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_sub_, estop_sub_, reset_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortingSequencer>());
  rclcpp::shutdown();
  return 0;
}
