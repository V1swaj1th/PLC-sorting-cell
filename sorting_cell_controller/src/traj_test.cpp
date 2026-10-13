#include <cstdio>
#include <string>

#include "sorting_cell_controller/trajectory.hpp"

using namespace sorting_cell;

static Eigen::Matrix4d offsetPose(const Eigen::Matrix4d& T, double dx, double dy, double dz,
                                  double yaw) {
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  R.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix4d out = R * T;  // yaw about the base z axis
  out(0, 3) = T(0, 3) + dx;
  out(1, 3) = T(1, 3) + dy;
  out(2, 3) = T(2, 3) + dz;
  return out;
}

static void profileCheck(double v, double a) {
  const TrapezoidProfile p(v, a);
  const double T = p.duration();
  const int n = 20000;
  double max_v = 0.0, max_acc = 0.0, prev_s = 0.0, prev_v = 0.0, max_v_jump = 0.0;
  bool monotonic = true;
  for (int k = 0; k <= n; ++k) {
    const double t = T * k / n;
    const double s = p.position(t), vel = p.velocity(t);
    if (s < prev_s - 1e-12) monotonic = false;
    max_v = std::max(max_v, vel);
    if (k > 0) {
      max_acc = std::max(max_acc, std::abs(vel - prev_v) / (T / n));
      max_v_jump = std::max(max_v_jump, std::abs(vel - prev_v));
    }
    prev_s = s;
    prev_v = vel;
  }
  std::printf("[1] profile v=%.2f a=%.2f (%s): T=%.3f s, s(0)=%.1e, s(T)-1=%.1e, peak v=%.3f, "
              "max |accel|=%.3f, monotonic=%d, max v step=%.1e\n",
              v, a, (v * v / a >= 1.0 ? "triangular" : "trapezoid"), T, p.position(0.0),
              p.position(T) - 1.0, max_v, max_acc, monotonic, max_v_jump);
}

int main() {
  // 1. Velocity profile: trapezoid (long path) and triangular (short path / high v_max)
  profileCheck(0.5, 2.0);
  profileCheck(2.0, 1.0);
  profileCheck(1.0, 4.0);

  // 2. Scripted move home -> A -> B -> home, planned with the real IK
  Vector6d q_home;
  q_home << 0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0;
  const Eigen::Matrix4d T_home = forwardKinematics(q_home);
  const CartesianLimits lim;
  const double dt = 0.02;

  struct Goal { std::string name; Eigen::Matrix4d T; };
  const std::vector<Goal> goals = {
      {"A (+0.20 x, -0.10 z)", offsetPose(T_home, 0.20, 0.00, -0.10, 0.0)},
      {"B (+0.20 x, +0.20 y, -0.10 z, yaw 0.6)", offsetPose(T_home, 0.20, 0.20, -0.10, 0.6)},
      {"home", T_home}};

  Vector6d q_start = q_home;
  for (const Goal& g : goals) {
    const Eigen::Matrix4d T_start = forwardKinematics(q_start);
    const JointTrajectory tr = planCartesianMove(q_start, g.T, lim, dt);
    if (!tr.ok) {
      std::printf("[2] -> %s: PLAN FAILED at sample %d\n", g.name.c_str(), tr.fail_index);
      return 1;
    }
    double max_lin = 0.0, max_ang = 0.0, max_qd = 0.0, max_qdd = 0.0, min_w = 1e9, max_dev = 0.0;
    bool in_limits = true;
    const Eigen::Vector3d p0 = T_start.block<3, 1>(0, 3), p1 = g.T.block<3, 1>(0, 3);
    const Eigen::Vector3d dir = (p1 - p0).norm() > 1e-9 ? Eigen::Vector3d((p1 - p0).normalized())
                                                       : Eigen::Vector3d::Zero();
    for (size_t k = 0; k < tr.q.size(); ++k) {
      min_w = std::min(min_w, manipulability(tr.q[k]));
      const Eigen::Matrix4d T = forwardKinematics(tr.q[k]);
      const Eigen::Vector3d rel = T.block<3, 1>(0, 3) - p0;
      max_dev = std::max(max_dev, (rel - dir * rel.dot(dir)).norm());
      for (int i = 0; i < 6; ++i) {
        in_limits = in_limits && tr.q[k][i] >= kQMin[i] && tr.q[k][i] <= kQMax[i];
      }
      if (k == 0) continue;
      const Vector6d qd = (tr.q[k] - tr.q[k - 1]) / dt;
      max_qd = std::max(max_qd, qd.cwiseAbs().maxCoeff());
      if (k > 1) {
        const Vector6d qd_prev = (tr.q[k - 1] - tr.q[k - 2]) / dt;
        max_qdd = std::max(max_qdd, ((qd - qd_prev) / dt).cwiseAbs().maxCoeff());
      }
      const Eigen::Matrix4d Tp = forwardKinematics(tr.q[k - 1]);
      max_lin = std::max(max_lin, (T.block<3, 1>(0, 3) - Tp.block<3, 1>(0, 3)).norm() / dt);
      const Eigen::AngleAxisd da(Eigen::Matrix3d(T.block<3, 3>(0, 0) * Tp.block<3, 3>(0, 0).transpose()));
      max_ang = std::max(max_ang, da.angle() / dt);
    }
    const Vector6d e = poseError(forwardKinematics(tr.q.back()), g.T);
    std::printf("[2] -> %s\n    samples=%zu (%.2f s)  end err: %.1e m, %.1e rad  line deviation: %.1e m\n"
                "    max tool speed %.3f m/s (limit %.2f), max rot speed %.3f rad/s (limit %.2f)\n"
                "    max joint speed %.3f rad/s (limit 3.14), max joint accel %.2f rad/s^2, "
                "min manipulability %.3f, joints in limits=%d\n",
                g.name.c_str(), tr.q.size(), (tr.q.size() - 1) * dt, e.head<3>().norm(),
                e.tail<3>().norm(), max_dev, max_lin, lim.v_lin, max_ang, lim.v_ang, max_qd,
                max_qdd, min_w, in_limits);
    q_start = tr.q.back();
  }

  // 3. Pure rotation about the base z axis (no translation)
  {
    const JointTrajectory tr = planCartesianMove(q_home, offsetPose(T_home, 0, 0, 0, 0.8), lim, dt);
    std::printf("[3] pure rotation 0.8 rad: ok=%d, samples=%zu\n", tr.ok, tr.q.size());
  }

  // 4. Degenerate move (goal == start)
  {
    const JointTrajectory tr = planCartesianMove(q_home, T_home, lim, dt);
    std::printf("[4] zero-length move: ok=%d, samples=%zu\n", tr.ok, tr.q.size());
  }

  // 5. Unreachable goal must fail cleanly, not crash or hang
  {
    const JointTrajectory tr = planCartesianMove(q_home, offsetPose(T_home, 2.0, 0, 0, 0.0), lim, dt);
    std::printf("[5] unreachable goal (+2 m): ok=%d, fail_index=%d\n", tr.ok, tr.fail_index);
  }
  return 0;
}
