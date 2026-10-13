#include <cstdio>
#include <random>

#include "sorting_cell_controller/pick_place.hpp"

using namespace sorting_cell;

static void check(const std::vector<CubeTarget>& cubes, bool verbose) {
  const PickPlaceConfig cfg;
  const PickPlacePlan plan = planPickPlace(cubes, cfg);
  if (!plan.ok) {
    std::printf("PLAN FAILED: %s\n", plan.error.c_str());
    return;
  }
  double max_qd = 0.0, max_jump = 0.0, min_w = 1e9, max_grasp_err = 0.0, max_place_err = 0.0;
  bool in_limits = true;
  for (size_t k = 0; k < plan.q.size(); ++k) {
    min_w = std::min(min_w, manipulability(plan.q[k]));
    for (int i = 0; i < 6; ++i)
      in_limits = in_limits && plan.q[k][i] >= kQMin[i] && plan.q[k][i] <= kQMax[i];
    if (k > 0) {
      const Vector6d d = plan.q[k] - plan.q[k - 1];
      max_qd = std::max(max_qd, d.cwiseAbs().maxCoeff() / cfg.dt);
      max_jump = std::max(max_jump, d.cwiseAbs().maxCoeff());
    }
  }
  for (size_t i = 0; i < cubes.size(); ++i) {
    const Eigen::Vector3d g = forwardKinematics(plan.q[plan.grasp_frame[i] - 1]).block<3, 1>(0, 3);
    max_grasp_err = std::max(max_grasp_err,
                             (g - Eigen::Vector3d(cubes[i].p.x(), cubes[i].p.y(), cfg.grasp_z)).norm());
    const Eigen::Vector3d r = forwardKinematics(plan.q[plan.release_frame[i] - 1]).block<3, 1>(0, 3);
    max_place_err = std::max(max_place_err, (r - plan.place_pos[i]).norm());
  }
  const Vector6d e_home = plan.q.back() - homeJoints();
  if (verbose) {
    std::printf("frames=%zu (%.1f s), %.1f s per cube\n", plan.q.size(), plan.q.size() * cfg.dt,
                plan.q.size() * cfg.dt / cubes.size());
    for (size_t i = 0; i < cubes.size(); ++i)
      std::printf("  %-6s grasp@%d release@%d -> bin (%.2f, %.2f)\n", cubes[i].color.c_str(),
                  plan.grasp_frame[i], plan.release_frame[i], plan.place_pos[i].x(),
                  plan.place_pos[i].y());
    std::printf("max joint speed %.3f rad/s (UR5e limit 3.14), max joint step %.4f rad, "
                "min manipulability %.3f, joints in limits=%d\n",
                max_qd, max_jump, min_w, in_limits);
    std::printf("grasp position error %.1e m, place position error %.1e m, "
                "final joint distance from home %.1e rad\n",
                max_grasp_err, max_place_err, e_home.cwiseAbs().maxCoeff());
  }
}

int main() {
  // Same scene as vision_node
  const std::vector<CubeTarget> scene = {{"red", {-0.40, -0.20, 0.02}, 0.0},
                                         {"green", {-0.45, 0.15, 0.02}, 0.5},
                                         {"blue", {-0.58, -0.10, 0.02}, 1.0},
                                         {"yellow", {-0.62, 0.25, 0.02}, 0.3}};
  std::printf("[1] demo scene\n");
  check(scene, true);

  // Random scenes: 4 cubes, one per colour, random order, positions and yaw
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(-0.66, -0.36), uy(-0.30, 0.30), uyaw(0.0, M_PI / 2);
  const char* colors[4] = {"red", "green", "blue", "yellow"};
  int fails = 0;
  const int N = 300;
  for (int n = 0; n < N; ++n) {
    std::vector<CubeTarget> cs;
    std::vector<int> idx = {0, 1, 2, 3};
    std::shuffle(idx.begin(), idx.end(), rng);
    while (cs.size() < 4) {
      CubeTarget c{colors[idx[cs.size()]], {ux(rng), uy(rng), 0.02}, uyaw(rng)};
      bool far = true;
      for (const auto& o : cs) far = far && (c.p - o.p).norm() > 0.09;
      if (far) cs.push_back(c);
    }
    const PickPlacePlan p = planPickPlace(cs);
    if (!p.ok) {
      ++fails;
      if (fails <= 3) std::printf("    fail %d: %s\n", fails, p.error.c_str());
    }
  }
  std::printf("[2] random 4-cube scenes: %d/%d planned, %d failed\n", N - fails, N, fails);
  return 0;
}
