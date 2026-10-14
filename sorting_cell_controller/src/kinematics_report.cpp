// Kinematics report: numbers for the README and the audit, from checks that do not rely on the
// controller's own model where possible.
//   A  own DH FK vs the URDF joint chain of ur_description (default_kinematics.yaml)
//   B  geometric Jacobian vs central finite differences
//   C  IK success rates: random joint vectors (far and near seeds), tool-down table poses,
//      near the wrist and elbow singularities, at the elbow joint limit
//   D  planned joint speed / acceleration over random scenes
//   E  trapezoidal profile limits, including short (triangular) moves
// Usage: kinematics_report $(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/default_kinematics.yaml
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <random>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;

static Eigen::Matrix4d xyzrpy(double x, double y, double z, double r, double p, double yw) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(yw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return T;
}
static Eigen::Matrix4d rotz(double a) { return xyzrpy(0, 0, 0, 0, 0, a); }

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: kinematics_report PATH/TO/ur5e/default_kinematics.yaml\n");
    return 2;
  }
  // ---------- A: own DH FK vs the URDF chain (ur_description default_kinematics.yaml)
  const YAML::Node k = YAML::LoadFile(argv[1])["kinematics"];
  const char* names[6] = {"shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"};
  Eigen::Matrix4d O[6];
  for (int i = 0; i < 6; ++i) {
    const YAML::Node n = k[names[i]];
    O[i] = xyzrpy(n["x"].as<double>(), n["y"].as<double>(), n["z"].as<double>(), n["roll"].as<double>(),
                  n["pitch"].as<double>(), n["yaw"].as<double>());
  }
  // base == base_link_inertia (both are base_link rotated by pi about z); tool0 = wrist_3 * flange * tool0
  const Eigen::Matrix4d flange = xyzrpy(0, 0, 0, 0, -M_PI / 2, -M_PI / 2), tool0 = xyzrpy(0, 0, 0, M_PI / 2, 0, M_PI / 2);
  auto urdfFk = [&](const Vector6d& q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) T = T * O[i] * rotz(q[i]);
    return T * flange * tool0;
  };
  std::mt19937 rng(7);
  auto randomQ = [&]() {
    Vector6d q;
    for (int j = 0; j < 6; ++j) q[j] = std::uniform_real_distribution<double>(kQMin[j], kQMax[j])(rng);
    return q;
  };
  double pmax = 0, rmax = 0;
  for (int n = 0; n < 20000; ++n) {
    const Vector6d q = randomQ();
    const Eigen::Matrix4d A = forwardKinematics(q), B = urdfFk(q);
    pmax = std::max(pmax, (A.block<3, 1>(0, 3) - B.block<3, 1>(0, 3)).norm());
    rmax = std::max(rmax, Eigen::AngleAxisd(Eigen::Matrix3d(A.block<3, 3>(0, 0).transpose() * B.block<3, 3>(0, 0))).angle());
  }
  std::printf("A FK vs URDF chain, 20000 random q in limits: max pos err %.2e m, max rot err %.2e rad\n", pmax, rmax);

  // ---------- B: geometric Jacobian vs central finite differences of the FK (position + rotation)
  double jmax = 0;
  for (int n = 0; n < 5000; ++n) {
    const Vector6d q = randomQ();
    const Matrix6d J = geometricJacobian(q);
    const double h = 1e-6;
    for (int j = 0; j < 6; ++j) {
      Vector6d a = q, b = q;
      a[j] += h;
      b[j] -= h;
      const Eigen::Matrix4d Ta = forwardKinematics(a), Tb = forwardKinematics(b);
      Vector6d col;
      col.head<3>() = (Ta.block<3, 1>(0, 3) - Tb.block<3, 1>(0, 3)) / (2 * h);
      const Eigen::AngleAxisd aa(Eigen::Matrix3d(Ta.block<3, 3>(0, 0) * Tb.block<3, 3>(0, 0).transpose()));
      col.tail<3>() = aa.axis() * aa.angle() / (2 * h);
      jmax = std::max(jmax, (col - J.col(j)).cwiseAbs().maxCoeff());
    }
  }
  std::printf("B Jacobian vs central FD (pos and rot rows), 5000 random q: max abs diff %.2e\n", jmax);

  // ---------- C: IK over random reachable targets
  auto ikCampaign = [&](const char* label, auto genQ, auto seedOf, int N) {
    int ok = 0, inlim = 0;
    double worst_p = 0, worst_r = 0, it = 0, worst_fail_p = 0;
    for (int n = 0; n < N; ++n) {
      const Vector6d q = genQ();
      const IkResult r = solveIk(forwardKinematics(q), seedOf(q));
      if (r.converged) {
        ++ok;
        worst_p = std::max(worst_p, r.pos_err);
        worst_r = std::max(worst_r, r.rot_err);
        it += r.iterations;
      } else {
        worst_fail_p = std::max(worst_fail_p, r.pos_err);
      }
      bool lim = true;
      for (int j = 0; j < 6; ++j) lim = lim && r.q[j] >= kQMin[j] - 1e-12 && r.q[j] <= kQMax[j] + 1e-12;
      inlim += lim;
    }
    std::printf("C %-46s %5d/%d converged (%.1f%%), mean iters %.1f, residual max %.1e m / %.1e rad, worst failed residual %.3f m, "
                "solutions in limits %d/%d\n",
                label, ok, N, 100.0 * ok / N, ok ? it / ok : 0.0, worst_p, worst_r, worst_fail_p, inlim, N);
  };
  std::normal_distribution<double> g01(0.0, 1.0);
  auto near = [&](double s) {
    return [&, s](const Vector6d& q) {
      Vector6d d;
      for (int j = 0; j < 6; ++j) d[j] = s * g01(rng);
      return Vector6d(q + d);
    };
  };
  auto home = [&](const Vector6d&) { return homeJoints(); };
  ikCampaign("uniform q in limits, seed = home", randomQ, home, 3000);
  ikCampaign("uniform q in limits, seed = q + N(0,0.1)", randomQ, near(0.1), 3000);
  ikCampaign("uniform q in limits, seed = q + N(0,0.5)", randomQ, near(0.5), 3000);
  // tool-down over the table, the cell's actual use: target from FK of an IK-feasible table pose
  auto tableQ = [&]() {
    for (;;) {
      const double x = std::uniform_real_distribution<double>(-0.80, -0.20)(rng);
      const double y = std::uniform_real_distribution<double>(-0.45, 0.45)(rng);
      const double z = std::uniform_real_distribution<double>(0.02, 0.25)(rng);
      const double yaw = std::uniform_real_distribution<double>(-M_PI / 4, M_PI / 4)(rng);
      const IkResult r = solveIk(toolDownPose(Eigen::Vector3d(x, y, z), yaw), homeJoints());
      if (r.converged) return r.q;
    }
  };
  ikCampaign("tool-down table poses, seed = home", tableQ, home, 2000);
  // near singularities
  auto wristSing = [&]() {
    Vector6d q = randomQ();
    q[4] = 0.01 * g01(rng);
    return q;
  };
  auto elbowSing = [&]() {
    Vector6d q = randomQ();
    q[2] = 0.01 * g01(rng);
    return q;
  };
  ikCampaign("wrist singular (q5 ~ 0), seed = q + N(0,0.1)", wristSing, near(0.1), 2000);
  ikCampaign("elbow singular (q3 ~ 0), seed = q + N(0,0.1)", elbowSing, near(0.1), 2000);
  // joint limit handling: a target only reachable outside the elbow limit is never returned outside it
  {
    int out = 0;
    for (int n = 0; n < 2000; ++n) {
      Vector6d q = randomQ();
      q[2] = std::copysign(M_PI - 0.02, q[2]);
      const IkResult r = solveIk(forwardKinematics(q), Vector6d(q + Vector6d::Constant(0.2)));
      for (int j = 0; j < 6; ++j) out += (r.q[j] < kQMin[j] || r.q[j] > kQMax[j]);
    }
    std::printf("C targets at the elbow limit (|q3| = pi - 0.02): IK results outside the joint limits: %d\n", out);
  }

  // ---------- D: planned joint speed / acceleration / step over random scenes
  {
    SceneModel m(11);
    int planned = 0, over_v = 0, over_vset = 0, refused = 0;
    double vmax = 0, amax = 0, stepmax = 0, wmin = 1e9;
    std::vector<double> vs;
    for (int n = 0; n < 300; ++n) {
      if (!m.execute("random all").ok) {
        ++refused;
        continue;
      }
      const PickPlacePlan p = planPickPlace(cubeTargets(m.scene()), planConfig(m.scene()));
      if (!p.ok) continue;
      ++planned;
      double v_this = 0;
      for (size_t f = 1; f < p.q.size(); ++f) {
        const Vector6d d = p.q[f] - p.q[f - 1];
        stepmax = std::max(stepmax, d.cwiseAbs().maxCoeff());
        v_this = std::max(v_this, d.cwiseAbs().maxCoeff() / 0.02);
        if (f + 1 < p.q.size()) {
          const Vector6d a = (p.q[f + 1] - 2 * p.q[f] + p.q[f - 1]) / (0.02 * 0.02);
          amax = std::max(amax, a.cwiseAbs().maxCoeff());
        }
        wmin = std::min(wmin, manipulability(p.q[f]));
      }
      vs.push_back(v_this);
      vmax = std::max(vmax, v_this);
      over_v += v_this > 3.14;
      over_vset += v_this > 2.9;
    }
    std::sort(vs.begin(), vs.end());
    std::printf("D 300 random scenes (random all): %d planned, %d refused; peak joint speed max %.2f rad/s, median %.2f, "
                "p95 %.2f; scenes over 3.14: %d, over the drive's 2.9 setpoint limit: %d; max joint accel %.1f rad/s^2; "
                "max frame step %.3f rad; min manipulability %.4f\n",
                planned, refused, vmax, vs.empty() ? 0 : vs[vs.size() / 2], vs.empty() ? 0 : vs[vs.size() * 95 / 100], over_v,
                over_vset, amax, stepmax, wmin);
  }
  // ---------- E: trapezoid profile: short move (triangular), continuity, limits
  {
    double worst_v = 0, worst_a = 0, worst_jump = 0;
    for (double vmaxp : {0.1, 0.5, 1.0, 3.0, 10.0})
      for (double amaxp : {0.1, 0.5, 2.0, 20.0}) {
        TrapezoidProfile P(vmaxp, amaxp);
        const double T = P.duration(), dt = T / 20000;
        double prev_v = 0;
        for (int i = 1; i <= 20000; ++i) {
          const double t = i * dt;
          const double v = (P.position(t) - P.position(t - dt)) / dt;
          worst_v = std::max(worst_v, v / vmaxp);
          worst_a = std::max(worst_a, std::abs(v - prev_v) / dt / amaxp);
          worst_jump = std::max(worst_jump, std::abs(P.velocity(t) - P.velocity(t - dt)));
          prev_v = v;
        }
        if (std::abs(P.position(T) - 1.0) > 1e-12) std::printf("E end position wrong for v %.1f a %.1f\n", vmaxp, amaxp);
      }
    std::printf("E trapezoid, 20 (v,a) pairs incl. triangular: max v/v_max %.4f, max a/a_max %.4f (FD), max velocity jump "
                "per step %.2e, end position exact\n",
                worst_v, worst_a, worst_jump);
  }
  return 0;
}
