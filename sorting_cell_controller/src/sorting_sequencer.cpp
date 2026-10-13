#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sorting_cell_controller/pick_place.hpp"

using namespace sorting_cell;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {
constexpr double kCubeSize = 0.04;

void colorOf(const std::string& name, float& r, float& g, float& b) {
  r = g = b = 0.6f;
  if (name == "red") { r = 0.86f; g = 0.16f; b = 0.16f; }
  if (name == "green") { r = 0.24f; g = 0.75f; b = 0.24f; }
  if (name == "blue") { r = 0.16f; g = 0.35f; b = 0.86f; }
  if (name == "yellow") { r = 0.90f; g = 0.86f; b = 0.16f; }
}
}  // namespace

// Receives the cube list from the vision node, plans the whole sort with the own
// FK/IK/trapezoidal planner, then plays it back at 50 Hz on joint_states.
class SortingSequencer : public rclcpp::Node {
 public:
  enum class State { WAITING, STARTING, RUNNING, DONE, FAULT };

  SortingSequencer() : Node("sorting_sequencer") {
    declare_parameter("start_delay_s", 3.0);
    start_delay_ = get_parameter("start_delay_s").as_double();

    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    markers_pub_ = create_publisher<MarkerArray>("sorted_cubes", 10);
    busy_pub_ = create_publisher<std_msgs::msg::Bool>("arm_busy", 10);
    home_pub_ = create_publisher<std_msgs::msg::Bool>("at_home", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("cycle_done", 10);
    fault_pub_ = create_publisher<std_msgs::msg::Bool>("fault", 10);

    poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "cube_poses", 10, [this](geometry_msgs::msg::PoseArray::SharedPtr m) {
          poses_ = *m;
          have_poses_ = true;
          tryPlan();
        });
    colors_sub_ = create_subscription<std_msgs::msg::String>(
        "cube_colors", 10, [this](std_msgs::msg::String::SharedPtr m) {
          colors_.clear();
          std::string cur;
          for (char ch : m->data + ",") {
            if (ch == ',') {
              if (!cur.empty()) colors_.push_back(cur);
              cur.clear();
            } else {
              cur += ch;
            }
          }
          have_colors_ = true;
          tryPlan();
        });

    q_ = homeJoints();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "sorting_sequencer waiting for cube_poses and cube_colors");
  }

 private:
  void tryPlan() {
    if (state_ != State::WAITING || !have_poses_ || !have_colors_) return;
    if (poses_.poses.empty() || poses_.poses.size() != colors_.size()) return;

    cubes_.clear();
    for (size_t i = 0; i < colors_.size(); ++i) {
      const auto& p = poses_.poses[i];
      CubeTarget c;
      c.color = colors_[i];
      c.p = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
      c.yaw = 2.0 * std::atan2(p.orientation.z, p.orientation.w);
      cubes_.push_back(c);
    }
    const auto t0 = std::chrono::steady_clock::now();
    plan_ = planPickPlace(cubes_);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!plan_.ok) {
      state_ = State::FAULT;
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", plan_.error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Planned %zu cubes: %zu frames (%.1f s) in %.0f ms, starting in %.1f s",
                cubes_.size(), plan_.q.size(), static_cast<double>(plan_.q.size()) * 0.02, ms,
                start_delay_);
    start_time_ = now();
    state_ = State::STARTING;
  }

  void onTimer() {
    switch (state_) {
      case State::STARTING:
        if ((now() - start_time_).seconds() >= start_delay_) {
          state_ = State::RUNNING;
          cycle_start_ = now();
          RCLCPP_INFO(get_logger(), "Sorting started");
        }
        break;
      case State::RUNNING:
        q_ = plan_.q[idx_];
        if (++idx_ >= plan_.q.size()) {
          idx_ = plan_.q.size() - 1;
          state_ = State::DONE;
          RCLCPP_INFO(get_logger(), "Cycle done: %zu cubes sorted in %.1f s", cubes_.size(),
                      (now() - cycle_start_).seconds());
        }
        break;
      default:
        break;
    }
    publishJoints();
    if (++tick_ % 2 == 0) publishMarkers();
    if (tick_ % 5 == 0) publishStatus();
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
    busy.data = (state_ == State::RUNNING);
    home.data = (state_ != State::RUNNING) && (state_ != State::FAULT);
    done.data = (state_ == State::DONE);
    fault.data = (state_ == State::FAULT);
    busy_pub_->publish(busy);
    home_pub_->publish(home);
    done_pub_->publish(done);
    fault_pub_->publish(fault);
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
    colorOf(color, m.color.r, m.color.g, m.color.b);
    m.color.a = alpha;
    return m;
  }

  void publishMarkers() {
    MarkerArray arr;
    // table and bins are always shown
    Marker table = box("table", 0, 0.60, 0.90, 0.002, -0.50, 0.0, -0.001, 0.0, "", 1.0f);
    table.color.r = table.color.g = table.color.b = 0.37f;
    arr.markers.push_back(table);
    int id = 0;
    for (const auto& kv : PickPlaceConfig().bins) {
      arr.markers.push_back(
          box("bins", id++, 0.10, 0.16, 0.004, kv.second.x(), kv.second.y(), 0.002, 0.0, kv.first, 0.55f));
    }
    if (state_ == State::WAITING || state_ == State::FAULT) {
      markers_pub_->publish(arr);
      return;
    }
    const int frame = (state_ == State::STARTING) ? 0 : static_cast<int>(idx_);
    for (size_t i = 0; i < cubes_.size(); ++i) {
      double x, y, z, yaw;
      if (plan_.release_frame[i] >= 0 && frame >= plan_.release_frame[i]) {
        x = plan_.place_pos[i].x();
        y = plan_.place_pos[i].y();
        z = kCubeSize / 2.0;
        yaw = 0.0;
      } else if (plan_.grasp_frame[i] >= 0 && frame >= plan_.grasp_frame[i]) {
        const Eigen::Matrix4d T = forwardKinematics(plan_.q[frame]);
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
  bool have_poses_{false}, have_colors_{false};
  geometry_msgs::msg::PoseArray poses_;
  std::vector<std::string> colors_;
  std::vector<CubeTarget> cubes_;
  PickPlacePlan plan_;
  Vector6d q_;
  size_t idx_{0};
  unsigned long tick_{0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME}, cycle_start_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_pub_, home_pub_, done_pub_, fault_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr colors_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortingSequencer>());
  rclcpp::shutdown();
  return 0;
}
