#include <chrono>
#include <cstdio>
#include <random>

#include "sorting_cell_controller/ur5e_ik.hpp"

using namespace sorting_cell;

// Central finite-difference Jacobian (spatial angular velocity in the base frame).
static Matrix6d numericJacobian(const Vector6d& q, double h = 1e-6) {
  Matrix6d J;
  for (int i = 0; i < 6; ++i) {
    Vector6d qp = q, qm = q;
    qp[i] += h;
    qm[i] -= h;
    const Eigen::Matrix4d Tp = forwardKinematics(qp), Tm = forwardKinematics(qm);
    J.block<3, 1>(0, i) = (Tp.block<3, 1>(0, 3) - Tm.block<3, 1>(0, 3)) / (2.0 * h);
    const Eigen::AngleAxisd aa(Eigen::Matrix3d(Tp.block<3, 3>(0, 0) * Tm.block<3, 3>(0, 0).transpose()));
    J.block<3, 1>(3, i) = aa.axis() * aa.angle() / (2.0 * h);
  }
  return J;
}

static Vector6d randomNear(const Vector6d& c, double spread, std::mt19937& rng) {
  std::uniform_real_distribution<double> d(-spread, spread);
  Vector6d q = c;
  for (int i = 0; i < 6; ++i) q[i] += d(rng);
  return q;
}

int main() {
  Vector6d q_home;
  q_home << 0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0;
  std::mt19937 rng(42);
  const IkOptions opt;
  int failures = 0;  // checks with a pass criterion; the rest of the output is a report

  // 1. Jacobian vs finite differences
  double max_jac_err = 0.0;
  for (int n = 0; n < 200; ++n) {
    const Vector6d q = randomNear(q_home, 1.0, rng);
    max_jac_err = std::max(max_jac_err, (geometricJacobian(q) - numericJacobian(q)).cwiseAbs().maxCoeff());
  }
  std::printf("[1] Jacobian vs finite difference, 200 poses: max abs diff = %.2e\n", max_jac_err);
  failures += max_jac_err > 1e-6;

  // 2. IK on reachable targets: far seed (home) and near seed (true + noise)
  const int N = 500;
  for (int regime = 0; regime < 2; ++regime) {
    for (double spread : {0.3, 0.6, 1.0}) {
      int ok = 0;
      long iters = 0;
      double max_pos = 0.0, max_rot = 0.0;
      for (int n = 0; n < N; ++n) {
        const Vector6d q_true = randomNear(q_home, spread, rng);
        const Vector6d seed = (regime == 0) ? q_home : randomNear(q_true, 0.1, rng);
        const IkResult r = solveIk(forwardKinematics(q_true), seed, opt);
        if (r.converged) {
          ++ok;
          iters += r.iterations;
          max_pos = std::max(max_pos, r.pos_err);
          max_rot = std::max(max_rot, r.rot_err);
        }
      }
      std::printf("[2] %s seed, targets from home +-%.1f rad: converged %d/%d, mean iters %.1f, "
                  "max pos err %.1e m, max rot err %.1e rad\n",
                  regime == 0 ? "far (home)" : "near      ", spread, ok, N,
                  ok ? static_cast<double>(iters) / ok : 0.0, max_pos, max_rot);
    }
  }

  // 3. Cartesian line with chained seeds (how the sequencer will use the IK)
  {
    const Eigen::Matrix4d T0 = forwardKinematics(q_home);
    const int steps = 100;
    Vector6d q = q_home;
    int fails = 0;
    long iters = 0;
    double max_jump = 0.0, min_w = 1e9;
    for (int k = 1; k <= steps; ++k) {
      Eigen::Matrix4d T = T0;
      const double s = static_cast<double>(k) / steps;
      T(0, 3) += 0.20 * s;
      T(2, 3) -= 0.10 * s;
      const IkResult r = solveIk(T, q, opt);
      if (!r.converged) ++fails;
      iters += r.iterations;
      max_jump = std::max(max_jump, (r.q - q).cwiseAbs().maxCoeff());
      min_w = std::min(min_w, r.manip);
      q = r.q;
    }
    std::printf("[3] 100-point Cartesian line (+0.20 m x, -0.10 m z): failures %d, mean iters %.1f, "
                "max joint jump %.4f rad, min manipulability %.4f\n",
                fails, static_cast<double>(iters) / steps, max_jump, min_w);
    failures += fails > 0;
  }

  // 4. Approaching a wrist singularity (wrist_2 -> 0 aligns the wrist axes).
  //    The plain pseudo-inverse step grows like 1/sigma_min; adaptive DLS stays bounded.
  {
    Vector6d e;
    e << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    IkOptions undamped;
    undamped.lambda_min = undamped.lambda_max = 1e-6;
    std::printf("[4] manipulability at home = %.4f; step norm for a fixed small error as wrist_2 -> 0:\n",
                manipulability(q_home));
    for (double q5 : {0.2, 0.05, 0.02, 0.005}) {
      Vector6d q = q_home;
      q[4] = q5;
      const Matrix6d J = geometricJacobian(q);
      std::printf("    wrist_2 = %.3f rad: w = %.1e, undamped step = %.2e rad, adaptive DLS step = %.2e rad\n",
                  q5, manipulability(J), dlsStep(J, e, undamped).norm(), dlsStep(J, e, opt).norm());
      failures += dlsStep(J, e, opt).norm() > 0.2;
    }
  }

  // 5. Unreachable target: must stop, report failure, and respect joint limits
  {
    Eigen::Matrix4d T = forwardKinematics(q_home);
    T(0, 3) += 2.0;
    const IkResult r = solveIk(T, q_home, opt);
    bool in_limits = true;
    for (int i = 0; i < 6; ++i) in_limits = in_limits && r.q[i] >= kQMin[i] && r.q[i] <= kQMax[i];
    std::printf("[5] unreachable target (+2 m): converged=%d, iterations=%d, final pos err %.3f m, "
                "joints in limits=%d\n", r.converged, r.iterations, r.pos_err, in_limits);
    failures += r.converged || !in_limits;
  }

  // 6. Timing (near seed)
  {
    const int M = 2000;
    std::vector<Eigen::Matrix4d> targets;
    std::vector<Vector6d> seeds;
    for (int n = 0; n < M; ++n) {
      const Vector6d q_true = randomNear(q_home, 0.6, rng);
      targets.push_back(forwardKinematics(q_true));
      seeds.push_back(randomNear(q_true, 0.1, rng));
    }
    const auto t0 = std::chrono::steady_clock::now();
    int ok = 0;
    for (int n = 0; n < M; ++n) ok += solveIk(targets[n], seeds[n], opt).converged ? 1 : 0;
    const auto t1 = std::chrono::steady_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / M;
    std::printf("[6] near-seed solve time: %.1f us per IK (converged %d/%d)\n", us, ok, M);
  }
  std::printf("%s (%d failed checks)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
