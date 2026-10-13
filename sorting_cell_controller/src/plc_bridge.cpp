#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "sorting_cell_controller/plc_link.hpp"

using namespace sorting_cell;

// Modbus TCP master between the PLC (OpenPLC) and the ROS 2 nodes.
//   PLC -> ROS : plc/run, plc/estop, plc/reset, plc/drive_alarm (Bool), plc/state (String)
//   ROS -> PLC : arm_busy, at_home, cycle_done, fault (Bool), arm_phase (String) from the sequencer;
//                drive/fault (Bool) from the servo drive (latched in the PLC as drive_alarm);
//                operator panel: panel/start, panel/stop, panel/reset (Empty = momentary push button),
//                panel/estop (Bool, true = E-stop pressed, latching)
// Fail-safe: while the PLC is unreachable the bridge publishes run=false, estop=true.
class PlcBridge : public rclcpp::Node {
 public:
  PlcBridge() : Node("plc_bridge") {
    declare_parameter("host", std::string("127.0.0.1"));
    declare_parameter("port", 5020);
    declare_parameter("rate_hz", 20.0);
    declare_parameter("button_pulse_s", 0.5);
    link_ = std::make_unique<PlcLink>(get_parameter("host").as_string(),
                                      static_cast<int>(get_parameter("port").as_int()));
    pulse_s_ = get_parameter("button_pulse_s").as_double();

    run_pub_ = create_publisher<std_msgs::msg::Bool>("plc/run", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("plc/estop", 10);
    reset_pub_ = create_publisher<std_msgs::msg::Bool>("plc/reset", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("plc/state", 10);
    conn_pub_ = create_publisher<std_msgs::msg::Bool>("plc/connected", 10);
    alarm_pub_ = create_publisher<std_msgs::msg::Bool>("plc/drive_alarm", 10);

    auto bool_sub = [this](const char* topic, bool PlcInputs::*field) {
      return create_subscription<std_msgs::msg::Bool>(
          topic, 10, [this, field](std_msgs::msg::Bool::SharedPtr m) { in_.*field = m->data; });
    };
    subs_bool_.push_back(bool_sub("arm_busy", &PlcInputs::arm_busy));
    subs_bool_.push_back(bool_sub("at_home", &PlcInputs::at_home));
    subs_bool_.push_back(bool_sub("cycle_done", &PlcInputs::cycle_done));
    subs_bool_.push_back(bool_sub("fault", &PlcInputs::fault));
    subs_bool_.push_back(bool_sub("drive/fault", &PlcInputs::drive_fault));
    phase_sub_ = create_subscription<std_msgs::msg::String>(
        "arm_phase", 10, [this](std_msgs::msg::String::SharedPtr m) {
          in_.ph_pick = (m->data == "pick");
          in_.ph_place = (m->data == "place");
        });

    auto pulse_sub = [this](const char* topic, rclcpp::Time* until) {
      return create_subscription<std_msgs::msg::Empty>(
          topic, 10, [this, until](std_msgs::msg::Empty::SharedPtr) {
            *until = now() + rclcpp::Duration::from_seconds(pulse_s_);
          });
    };
    subs_empty_.push_back(pulse_sub("panel/start", &start_until_));
    subs_empty_.push_back(pulse_sub("panel/stop", &stop_until_));
    subs_empty_.push_back(pulse_sub("panel/reset", &reset_until_));
    estop_panel_sub_ = create_subscription<std_msgs::msg::Bool>(
        "panel/estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
          estop_pressed_ = m->data;
          RCLCPP_WARN(get_logger(), "Operator panel: E-stop %s", m->data ? "PRESSED" : "released");
        });

    const double hz = get_parameter("rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "plc_bridge started: %s:%ld", get_parameter("host").as_string().c_str(),
                get_parameter("port").as_int());
  }

 private:
  void onTimer() {
    if (!link_->connected()) {
      if (now() < next_connect_) {
        publish(PlcOutputs{}, false);
        return;
      }
      next_connect_ = now() + rclcpp::Duration::from_seconds(1.0);
      if (!link_->connect()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PLC not reachable (%s)",
                             link_->lastError().c_str());
        publish(PlcOutputs{}, false);
        return;
      }
      RCLCPP_INFO(get_logger(), "Connected to the PLC");
    }
    PlcInputs in = in_;
    const rclcpp::Time t = now();
    in.start_pb = t < start_until_;
    in.stop_pb = t < stop_until_;
    in.reset_pb = t < reset_until_;
    in.estop_ok = !estop_pressed_;  // normally-closed loop: 1 = healthy
    PlcOutputs out;
    if (!link_->exchange(in, out)) {
      RCLCPP_ERROR(get_logger(), "Modbus error, dropping the connection (%s)", link_->lastError().c_str());
      publish(PlcOutputs{}, false);
      return;
    }
    publish(out, true);
  }

  void publish(const PlcOutputs& o, bool connected) {
    std_msgs::msg::Bool run, estop, reset, conn, alarm;
    // Fail-safe defaults when not connected: no run permission, E-stop asserted.
    run.data = connected && o.run;
    estop.data = connected ? (o.estop_active || o.estop_latch) : true;
    reset.data = connected && o.reset_cmd;
    conn.data = connected;
    alarm.data = connected && o.drive_alarm;
    alarm_pub_->publish(alarm);
    run_pub_->publish(run);
    estop_pub_->publish(estop);
    reset_pub_->publish(reset);
    conn_pub_->publish(conn);
    std_msgs::msg::String st;
    st.data = connected ? plcStateName(o) : "NO_PLC";
    if (st.data != last_state_) {
      RCLCPP_INFO(get_logger(), "PLC state: %s", st.data.c_str());
      last_state_ = st.data;
    }
    state_pub_->publish(st);
  }

  std::unique_ptr<PlcLink> link_;
  PlcInputs in_;
  bool estop_pressed_{false};
  double pulse_s_{0.5};
  rclcpp::Time start_until_{0, 0, RCL_ROS_TIME}, stop_until_{0, 0, RCL_ROS_TIME},
      reset_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_connect_{0, 0, RCL_ROS_TIME};
  std::string last_state_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr run_pub_, estop_pub_, reset_pub_, conn_pub_, alarm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subs_bool_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> subs_empty_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_panel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlcBridge>());
  rclcpp::shutdown();
  return 0;
}
