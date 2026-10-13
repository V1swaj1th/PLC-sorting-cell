#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sorting_cell_controller/ur5e_kinematics.hpp"

class FkChecker : public rclcpp::Node {
 public:
  FkChecker() : Node("fk_checker"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    tool_frame_ = declare_parameter<std::string>("tool_frame", "tool0");
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&FkChecker::onJointState, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::seconds(2),
                               std::bind(&FkChecker::report, this));
    RCLCPP_INFO(get_logger(), "Comparing own FK against TF %s -> %s",
                base_frame_.c_str(), tool_frame_.c_str());
  }

 private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    static const std::array<std::string, 6> kNames = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
    sorting_cell::Vector6d q;
    for (size_t i = 0; i < 6; ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), kNames[i]);
      if (it == msg->name.end()) return;
      const size_t idx = static_cast<size_t>(it - msg->name.begin());
      if (idx >= msg->position.size()) return;
      q[static_cast<int>(i)] = msg->position[idx];
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(base_frame_, tool_frame_, msg->header.stamp,
                                      rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException& e) {
      ++lookup_failures_;
      last_error_ = e.what();
      return;
    }

    const Eigen::Matrix4d T = sorting_cell::forwardKinematics(q);
    const Eigen::Vector3d p_tf(tf.transform.translation.x, tf.transform.translation.y,
                               tf.transform.translation.z);
    const Eigen::Quaterniond quat(tf.transform.rotation.w, tf.transform.rotation.x,
                                  tf.transform.rotation.y, tf.transform.rotation.z);
    const Eigen::Matrix3d R_tf = quat.normalized().toRotationMatrix();

    const double pos_err = (T.block<3, 1>(0, 3) - p_tf).norm();
    double c = ((T.block<3, 3>(0, 0).transpose() * R_tf).trace() - 1.0) / 2.0;
    c = std::max(-1.0, std::min(1.0, c));
    const double rot_err = std::acos(c);

    max_pos_err_ = std::max(max_pos_err_, pos_err);
    max_rot_err_ = std::max(max_rot_err_, rot_err);
    ++samples_;
  }

  void report() {
    if (samples_ == 0) {
      RCLCPP_WARN(get_logger(), "No samples yet (lookup failures: %zu). Last error: %s",
                  lookup_failures_, last_error_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(),
                "samples=%zu  max position error=%.3e m  max rotation error=%.3e rad  "
                "(lookup failures=%zu)",
                samples_, max_pos_err_, max_rot_err_, lookup_failures_);
  }

  std::string base_frame_, tool_frame_, last_error_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t samples_{0}, lookup_failures_{0};
  double max_pos_err_{0.0}, max_rot_err_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkChecker>());
  rclcpp::shutdown();
  return 0;
}
