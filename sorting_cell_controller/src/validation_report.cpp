// M6 validation: plans the demo scene with the real planner and writes CSV files for plotting.
//   manip.csv  - per-frame joints, manipulability, tool speed, joint speed
//   cycle.csv  - per-cube cycle time
//   sweep.csv  - transport speed vs. total cycle time / peak joint speed / min manipulability
// Usage: validation_report [output_dir]
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

#include "sorting_cell_controller/pick_place.hpp"

using namespace sorting_cell;

int main(int argc, char** argv) {
  const std::string dir = (argc > 1) ? argv[1] : ".";
  const std::vector<CubeTarget> scene = {{"red", {-0.40, -0.20, 0.02}, 0.0},
                                         {"green", {-0.45, 0.15, 0.02}, 0.5},
                                         {"blue", {-0.58, -0.10, 0.02}, 1.0},
                                         {"yellow", {-0.62, 0.25, 0.02}, 0.3}};
  const PickPlaceConfig cfg;
  const PickPlacePlan plan = planPickPlace(scene, cfg);
  if (!plan.ok) {
    std::printf("plan failed: %s\n", plan.error.c_str());
    return 1;
  }
  const double dt = cfg.dt;

  std::ofstream manip(dir + "/manip.csv");
  manip << "t,q1,q2,q3,q4,q5,q6,manipulability,tool_speed,max_joint_speed,cube\n";
  double min_w = 1e9, min_w_t = 0.0, max_qd = 0.0, max_tool = 0.0;
  int min_w_cube = -1;
  for (size_t k = 0; k < plan.q.size(); ++k) {
    int cube = static_cast<int>(scene.size()) - 1;
    for (size_t i = 0; i < scene.size(); ++i) {
      if (static_cast<int>(k) <= plan.end_frame[i]) {
        cube = static_cast<int>(i);
        break;
      }
    }
    const double w = manipulability(plan.q[k]);
    double tool = 0.0, qd = 0.0;
    if (k > 0) {
      const Eigen::Vector3d p1 = forwardKinematics(plan.q[k]).block<3, 1>(0, 3);
      const Eigen::Vector3d p0 = forwardKinematics(plan.q[k - 1]).block<3, 1>(0, 3);
      tool = (p1 - p0).norm() / dt;
      qd = (plan.q[k] - plan.q[k - 1]).cwiseAbs().maxCoeff() / dt;
    }
    if (w < min_w) {
      min_w = w;
      min_w_t = k * dt;
      min_w_cube = cube;
    }
    max_qd = std::max(max_qd, qd);
    max_tool = std::max(max_tool, tool);
    manip << k * dt;
    for (int j = 0; j < 6; ++j) manip << "," << plan.q[k][j];
    manip << "," << w << "," << tool << "," << qd << "," << cube << "\n";
  }

  std::ofstream cyc(dir + "/cycle.csv");
  cyc << "cube,color,start_s,grasp_s,release_s,end_s,duration_s\n";
  std::printf("Per-cube cycle time (home/previous cube -> retreat from the bin):\n");
  double total = 0.0;
  for (size_t i = 0; i < scene.size(); ++i) {
    const double t0 = (i == 0) ? 0.0 : plan.end_frame[i - 1] * dt;
    const double t1 = plan.end_frame[i] * dt;
    cyc << i << "," << scene[i].color << "," << t0 << "," << plan.grasp_frame[i] * dt << ","
        << plan.release_frame[i] * dt << "," << t1 << "," << (t1 - t0) << "\n";
    std::printf("  %-6s %.2f s\n", scene[i].color.c_str(), t1 - t0);
    total += t1 - t0;
  }
  std::printf("  mean   %.2f s per cube; whole cycle incl. return home: %.2f s\n",
              total / scene.size(), plan.q.size() * dt);
  std::printf("Peak joint speed %.3f rad/s (UR5e limit 3.14), peak tool speed %.3f m/s\n", max_qd,
              max_tool);
  std::printf("Min manipulability |det J| = %.4f at t = %.1f s (cube %d); values near 0 mean a singularity\n",
              min_w, min_w_t, min_w_cube);

  // Velocity sweep: transport speed of the hover-to-hover moves (descend/ascend stay slow)
  std::ofstream sw(dir + "/sweep.csv");
  sw << "v_transport,a_transport,total_s,per_cube_s,max_joint_speed,min_manipulability,within_joint_limit\n";
  std::printf("\nTransport speed sweep (accel = 2 x speed):\n");
  std::printf("  v [m/s]  total [s]  per cube [s]  peak joint [rad/s]  min w   within 3.14\n");
  for (double v : {0.15, 0.25, 0.40, 0.60, 0.80, 1.00}) {
    PickPlaceConfig c;
    c.fast.v_lin = v;
    c.fast.a_lin = 2.0 * v;
    const PickPlacePlan p = planPickPlace(scene, c);
    if (!p.ok) {
      std::printf("  %.2f     plan failed: %s\n", v, p.error.c_str());
      continue;
    }
    double qd = 0.0, w = 1e9;
    for (size_t k = 0; k < p.q.size(); ++k) {
      w = std::min(w, manipulability(p.q[k]));
      if (k > 0) qd = std::max(qd, (p.q[k] - p.q[k - 1]).cwiseAbs().maxCoeff() / c.dt);
    }
    const double tot = p.q.size() * c.dt;
    std::printf("  %.2f     %6.2f     %6.2f        %6.3f          %.3f   %s\n", v, tot,
                tot / scene.size(), qd, w, qd <= 3.14 ? "yes" : "NO");
    sw << v << "," << 2.0 * v << "," << tot << "," << tot / scene.size() << "," << qd << "," << w
       << "," << (qd <= 3.14 ? 1 : 0) << "\n";
  }
  std::printf("\nWrote manip.csv, cycle.csv, sweep.csv to %s\n", dir.c_str());
  return 0;
}
