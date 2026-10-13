#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "sorting_cell_controller/trajectory.hpp"

using namespace sorting_cell;

namespace {

constexpr double kDt = 0.02;  // 50 Hz, matches the timer below

// Pose offset from T: translate by (dx, dy, dz) and rotate by yaw about the base z axis.
Eigen::Matrix4d offsetPose(const Eigen::Matrix4d& T, double dx, double dy, double dz, double yaw) {
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  R.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix4d out = R * T;
  out(0, 3) = T(0, 3) + dx;
  out(1, 3) = T(1, 3) + dy;
  out(2, 3) = T(2, 3) + dz;
  return out;
}

}  // namespace

// Plans home -> A -> B -> home with the own FK/IK/trapezoidal planner, then loops the
// result forever, publishing joint_states at 50 Hz.
class CartesianMoveDemo : public rclcpp::Node {
 public:
  CartesianMoveDemo() : Node("cartesian_move_demo") {
    pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    Vector6d q;
    q << 0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0;
    const Eigen::Matrix4d T_home = forwardKinematics(q);
    const std::vector<Eigen::Matrix4d> goals = {
        offsetPose(T_home, 0.20, 0.00, -0.10, 0.0),
        offsetPose(T_home, 0.20, 0.20, -0.10, 0.6),
        T_home};

    const CartesianLimits limits;
    const size_t dwell_frames = 25;  // 0.5 s hold at each goal
    frames_.insert(frames_.end(), dwell_frames, q);
    for (size_t i = 0; i < goals.size(); ++i) {
      const JointTrajectory tr = planCartesianMove(q, goals[i], limits, kDt);
      if (!tr.ok) {
        RCLCPP_ERROR(get_logger(), "Planning failed for goal %zu at sample %d (IK did not converge)",
                     i, tr.fail_index);
        return;
      }
      frames_.insert(frames_.end(), tr.q.begin() + 1, tr.q.end());
      q = tr.q.back();
      frames_.insert(frames_.end(), dwell_frames, q);
      RCLCPP_INFO(get_logger(), "Goal %zu planned: %zu samples (%.2f s)", i, tr.q.size(),
                  static_cast<double>(tr.q.size() - 1) * kDt);
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&CartesianMoveDemo::onTimer, this));
    RCLCPP_INFO(get_logger(), "Playing %zu frames (%.1f s per cycle)", frames_.size(),
                static_cast<double>(frames_.size()) * kDt);
  }

 private:
  void onTimer() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
    const Vector6d& q = frames_[idx_];
    msg.position.assign(q.data(), q.data() + 6);
    pub_->publish(msg);
    if (++idx_ >= frames_.size()) {
      idx_ = 0;
      ++cycles_;
      RCLCPP_INFO(get_logger(), "Cycle %zu complete, repeating", cycles_);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Vector6d> frames_;
  size_t idx_{0}, cycles_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianMoveDemo>());
  rclcpp::shutdown();
  return 0;
}
