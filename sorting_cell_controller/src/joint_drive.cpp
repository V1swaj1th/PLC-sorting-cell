#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "servo_core/ur5e_setup.hpp"
#include "sorting_cell_controller/pick_place.hpp"

using servo::DriveInputs;
using servo::DriveOutputs;
using servo::DriveState;

namespace {
const char* kJointNames[6] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                              "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
}

// Thin ROS 2 wrapper around the servo firmware (servo_core) and the simulated UR5e joints.
// All control logic lives in servo_core; this node only moves data in and out.
//
//   in   arm/joint_command   sensor_msgs/JointState  position + velocity targets (50 Hz, sequencer)
//        plc/estop, plc/reset std_msgs/Bool          with use_plc (E-stop goes to the drive directly)
//        drive/reset          std_msgs/Empty         stand-alone only: acknowledge a fault
//        gripper/payload_kg   std_msgs/Float64       mass in the gripper (plant only: the controller
//                                                    does not know it, the integrators reject it)
//        drive/inject         std_msgs/String        "jam N" (joint 1..6) or "clear": fault injection
//   out  joint_states         the ACTUAL simulated joint state (position, velocity, torque as effort)
//        drive/ready, drive/fault  std_msgs/Bool
//        drive/state          std_msgs/String        e.g. "RUNNING", "FAULT FOLLOWING_ERROR joint 2"
//        drive/following_error std_msgs/Float64MultiArray [rad] per joint
//
// The control law runs at a fixed 1 kHz of simulated time. A 100 Hz timer advances the simulation
// by however many 1 ms steps are due on the node clock (at most 50 per tick), so the discrete
// controller is the same whatever the wall-clock jitter is.
class JointDrive : public rclcpp::Node {
 public:
  JointDrive() : Node("joint_drive") {
    declare_parameter("use_plc", false);
    declare_parameter("target_period_s", 0.02);
    declare_parameter("publish_hz", 100.0);
    use_plc_ = get_parameter("use_plc").as_bool();
    target_period_ = static_cast<float>(get_parameter("target_period_s").as_double());

    const sorting_cell::Vector6d h = sorting_cell::homeJoints();
    servo_sim::Arr6 q0;
    for (int j = 0; j < 6; ++j) q0[j] = h[j];
    sim_.init(q0, servo_sim::ur5eDrive());
    in_.estop = use_plc_;  // fail-safe until the PLC reports otherwise
    in_.enable = true;
    for (int j = 0; j < 6; ++j) in_.target_pos[j] = static_cast<float>(q0[j]);
    in_.target_seq = 1;
    in_.target_period = target_period_;

    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("drive/ready", 10);
    fault_pub_ = create_publisher<std_msgs::msg::Bool>("drive/fault", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("drive/state", 10);
    fe_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("drive/following_error", 10);

    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "arm/joint_command", 10, [this](sensor_msgs::msg::JointState::SharedPtr m) { onCommand(*m); });
    if (use_plc_) {
      estop_sub_ = create_subscription<std_msgs::msg::Bool>(
          "plc/estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) { in_.estop = m->data; });
      reset_sub_ = create_subscription<std_msgs::msg::Bool>(
          "plc/reset", 10, [this](std_msgs::msg::Bool::SharedPtr m) { in_.reset = m->data; });
    } else {
      reset_btn_sub_ = create_subscription<std_msgs::msg::Empty>(
          "drive/reset", 10, [this](std_msgs::msg::Empty::SharedPtr) { reset_steps_ = 50; });
    }
    payload_sub_ = create_subscription<std_msgs::msg::Float64>(
        "gripper/payload_kg", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
          if (std::isfinite(m->data) && m->data >= 0.0 && m->data <= 10.0) sim_.plant().setPayload(m->data);
        });
    inject_sub_ = create_subscription<std_msgs::msg::String>(
        "drive/inject", 10, [this](std_msgs::msg::String::SharedPtr m) { onInject(m->data); });

    last_ = now();
    const double hz = get_parameter("publish_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "joint_drive started (use_plc=%s): 1 kHz servo loop, cascaded P/PI with feedforward",
                use_plc_ ? "true" : "false");
  }

 private:
  void onCommand(const sensor_msgs::msg::JointState& m) {
    // match by name; accept the plain 6-joint order when names are missing
    for (int j = 0; j < 6; ++j) {
      int k = j;
      if (!m.name.empty()) {
        k = -1;
        for (size_t i = 0; i < m.name.size(); ++i)
          if (m.name[i] == kJointNames[j]) k = static_cast<int>(i);
      }
      if (k < 0 || static_cast<size_t>(k) >= m.position.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "arm/joint_command without %s: ignored", kJointNames[j]);
        return;
      }
      in_.target_pos[j] = static_cast<float>(m.position[static_cast<size_t>(k)]);
      in_.target_vel[j] = static_cast<size_t>(k) < m.velocity.size() ? static_cast<float>(m.velocity[static_cast<size_t>(k)]) : 0.0f;
    }
    ++in_.target_seq;
  }

  void onInject(const std::string& text) {
    std::istringstream is(text);
    std::string what;
    int joint = 0;
    is >> what >> joint;
    if (what == "jam" && joint >= 1 && joint <= 6) {
      sim_.plant().setJam(joint - 1);
      RCLCPP_WARN(get_logger(), "Fault injection: joint %d jammed", joint);
    } else if (what == "clear") {
      sim_.plant().setJam(-1);
      RCLCPP_INFO(get_logger(), "Fault injection cleared");
    } else {
      RCLCPP_WARN(get_logger(), "drive/inject: use 'jam N' (N = 1..6) or 'clear'");
    }
  }

  void onTimer() {
    const rclcpp::Time t = now();
    const double elapsed = (t - last_).seconds();
    int steps = static_cast<int>(elapsed / 0.001);
    if (steps <= 0) return;
    last_ = last_ + rclcpp::Duration::from_seconds(steps * 0.001);
    if (steps > 50) {  // the node was starved: do not try to catch up all at once
      steps = 50;
      last_ = t;
    }
    for (int k = 0; k < steps; ++k) {
      if (!use_plc_) in_.reset = reset_steps_ > 0;
      if (reset_steps_ > 0) --reset_steps_;
      sim_.step(in_);
    }
    publish();
  }

  void publish() {
    const DriveOutputs& o = sim_.drive().outputs();
    const auto& P = sim_.plant();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.assign(kJointNames, kJointNames + 6);
    js.position.assign(P.q().begin(), P.q().end());
    js.velocity.assign(P.v().begin(), P.v().end());
    js.effort.assign(P.appliedTorque().begin(), P.appliedTorque().end());
    js_pub_->publish(js);

    if (++tick_ % 5 != 0) return;  // status at 20 Hz
    std_msgs::msg::Bool ready, fault;
    ready.data = o.ready;
    fault.data = (o.state == DriveState::FAULT);
    ready_pub_->publish(ready);
    fault_pub_->publish(fault);
    std_msgs::msg::Float64MultiArray fe;
    for (int j = 0; j < 6; ++j) fe.data.push_back(o.following_error[j]);
    fe_pub_->publish(fe);

    std::string st = servo::toString(o.state);
    if (o.state == DriveState::FAULT) {
      char buf[96];
      std::snprintf(buf, sizeof buf, " %s joint %d (%.1f mrad)", servo::toString(o.fault), o.fault_axis + 1,
                    o.fault_axis >= 0 ? 1e3 * std::abs(o.following_error[o.fault_axis]) : 0.0);
      st += buf;
    } else if (o.state == DriveState::DISABLED && use_plc_ && in_.estop) {
      st += " (E-stop)";
    } else if (o.state == DriveState::DISABLED) {
      st += o.stop_timed_out ? " (stop timeout, waiting for reset)" : " (stopped, waiting for reset)";
    }
    if (st != last_state_) {
      if (o.state == DriveState::FAULT) RCLCPP_ERROR(get_logger(), "Drive: %s - torque off, brakes applied", st.c_str());
      else RCLCPP_INFO(get_logger(), "Drive: %s", st.c_str());
      last_state_ = st;
    }
    std_msgs::msg::String s;
    s.data = st;
    state_pub_->publish(s);
  }

  servo_sim::DriveSim sim_;
  DriveInputs in_{};
  bool use_plc_{false};
  float target_period_{0.02f};
  int reset_steps_{0};
  unsigned long tick_{0};
  std::string last_state_;
  rclcpp::Time last_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_, fault_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fe_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_, reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_btn_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr payload_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inject_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointDrive>());
  rclcpp::shutdown();
  return 0;
}
