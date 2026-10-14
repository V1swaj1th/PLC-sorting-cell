// Tests the teach-and-playback core: jogging, the YAML program file, the program compiler and
// every error path. No ROS needed. Optional argument: path to write the demo program to.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "sorting_cell_controller/teach.hpp"

using namespace sorting_cell;

static int g_pass = 0, g_fail = 0;
static void expect(const std::string& what, bool ok) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  (ok ? g_pass : g_fail)++;
}

static TeachProgram demoProgram() {
  TeachProgram p;
  p.name = "demo_teach";
  p.set("a_hover", waypointFromPose(toolDownPose({-0.40, -0.20, 0.20}, 0.0)));
  p.set("a_grasp", waypointFromPose(toolDownPose({-0.40, -0.20, 0.02}, 0.0)));
  p.set("b_hover", waypointFromPose(toolDownPose({-0.45, 0.15, 0.20}, 0.5)));
  p.set("b_place", waypointFromPose(toolDownPose({-0.45, 0.15, 0.02}, 0.5)));
  auto mv = [](const char* n, bool fast) {
    Step s;
    s.type = StepType::MOVE;
    s.name = n;
    s.fast = fast;
    return s;
  };
  auto grip = [](bool close) {
    Step s;
    s.type = StepType::GRIPPER;
    s.close = close;
    return s;
  };
  Step home;
  home.type = StepType::HOME;
  p.steps = {mv("a_hover", true), mv("a_grasp", false), grip(true), mv("a_hover", false),
             mv("b_hover", true), mv("b_place", false), grip(false), mv("b_hover", false), home};
  return p;
}

int main(int argc, char** argv) {
  const Vector6d q_home = homeJoints();
  const Eigen::Matrix4d T_home = forwardKinematics(q_home);

  std::printf("[1] Jogging\n");
  {
    Vector6d d;
    d << 0, 0, 0, 0, 0, 0.1;
    const Eigen::Matrix4d T1 = applyJog(applyJog(T_home, d), d);
    d[5] = 0.2;
    const Eigen::Matrix4d T2 = applyJog(T_home, d);
    expect("two yaw jogs of 0.1 rad equal one of 0.2 rad", poseNear(T1, T2, 1e-9, 1e-9));
    expect("yaw jog changes the tool yaw by the jog angle", std::abs(toolYaw(T2) - 0.2) < 1e-9);
    d << 0.01, -0.02, 0.03, 0, 0, 0;
    const Eigen::Matrix4d T3 = applyJog(T_home, d);
    const Eigen::Quaterniond q_before(Eigen::Matrix3d(T_home.block<3, 3>(0, 0)));
    const Eigen::Quaterniond q_after(Eigen::Matrix3d(T3.block<3, 3>(0, 0)));
    expect("translation jog moves the position and keeps the orientation",
           (T3.block<3, 1>(0, 3) - T_home.block<3, 1>(0, 3) - d.head<3>()).norm() < 1e-12 &&
               q_before.angularDistance(q_after) < 1e-12);

    // 60 jogs of 5 mm toward -x, each planned from the previous joints, like the pendant node
    Vector6d q = q_home;
    Eigen::Matrix4d target = T_home;
    int ok_count = 0;
    Vector6d step;
    step << -0.005, 0, 0, 0, 0, 0;
    for (int i = 0; i < 60; ++i) {
      const Eigen::Matrix4d next = applyJog(target, step);
      const JointTrajectory tr = planCartesianMove(q, next);
      if (!tr.ok) break;
      q = tr.q.back();
      target = next;
      ++ok_count;
    }
    expect("60 chained 5 mm jogs all plan (" + std::to_string(ok_count) + "/60)", ok_count == 60);
    expect("arm ends on the accumulated target", poseNear(forwardKinematics(q), target, 1e-5, 1e-4));
    // Jog far beyond the reach: must be rejected cleanly
    Vector6d far;
    far << 3.0, 0, 0, 0, 0, 0;
    expect("a jog beyond the workspace is rejected cleanly",
           !planCartesianMove(q_home, applyJog(T_home, far)).ok);
  }

  std::printf("[2] YAML program file\n");
  const TeachProgram demo = demoProgram();
  {
    const std::string text = programToYaml(demo);
    TeachProgram back;
    std::string err;
    const bool ok = programFromYaml(text, back, err);
    expect("emitted YAML parses back (" + err + ")", ok);
    bool same = ok && back.names() == demo.names() && back.steps.size() == demo.steps.size() &&
                back.name == demo.name;
    for (const std::string& n : demo.names()) {
      const Waypoint* a = demo.find(n);
      const Waypoint* b = back.find(n);
      same = same && b && (a->p - b->p).norm() < 1e-9 && a->q.angularDistance(b->q) < 1e-9;
    }
    for (size_t i = 0; same && i < demo.steps.size(); ++i) {
      const Step &a = demo.steps[i], &b = back.steps[i];
      same = a.type == b.type && a.name == b.name && a.fast == b.fast && a.close == b.close &&
             std::abs(a.seconds - b.seconds) < 1e-12;
    }
    expect("round trip keeps waypoints (1e-9) and steps", same);
    expect("second emit is identical to the first", programToYaml(back) == text);

    const std::string path = argc > 1 ? argv[1] : "teach_test_demo.yaml";
    expect("saveProgram writes the file", saveProgram(demo, path, err));
    TeachProgram loaded;
    expect("loadProgram reads it back", loadProgram(path, loaded, err) && loaded.steps.size() == demo.steps.size());
    TeachProgram dummy;
    expect("loading a missing file fails with a message",
           !loadProgram("/nonexistent/dir/x.yaml", dummy, err) && !err.empty());
    expect("saving into a missing directory fails with a message",
           !saveProgram(demo, "/nonexistent/dir/x.yaml", err) && !err.empty());
    setenv("HOME", "/tmp", 1);
    TeachProgram from_home;
    expect("~ is expanded in program paths",
           saveProgram(demo, "~/teach_test_home.yaml", err) && std::ifstream("/tmp/teach_test_home.yaml").good() &&
               loadProgram("~/teach_test_home.yaml", from_home, err) && from_home.steps.size() == demo.steps.size());
  }

  std::printf("[3] Compiling the demo program from home\n");
  {
    const TeachPlan plan = compileProgram(demo, q_home);
    expect("compiles (" + plan.error + ")", plan.ok);
    if (plan.ok) {
      std::printf("      %zu frames (%.1f s), %zu steps, %zu gripper events\n", plan.q.size(),
                  plan.q.size() * 0.02, plan.step_end_frame.size(), plan.gripper.size());
      bool poses_ok = true, mono = true;
      double worst_p = 0, worst_r = 0, max_jump = 0;
      for (size_t i = 0; i < demo.steps.size(); ++i) {
        if (i > 0 && plan.step_end_frame[i] < plan.step_end_frame[i - 1]) mono = false;
        const Step& s = demo.steps[i];
        if (s.type != StepType::MOVE && s.type != StepType::HOME) continue;
        const Eigen::Matrix4d want = (s.type == StepType::HOME) ? T_home : poseMatrix(*demo.find(s.name));
        const Eigen::Matrix4d got = forwardKinematics(plan.q[plan.step_end_frame[i]]);
        worst_p = std::max(worst_p, (want.block<3, 1>(0, 3) - got.block<3, 1>(0, 3)).norm());
        worst_r = std::max(worst_r, Eigen::Quaterniond(Eigen::Matrix3d(want.block<3, 3>(0, 0)))
                                        .angularDistance(Eigen::Quaterniond(Eigen::Matrix3d(got.block<3, 3>(0, 0)))));
        poses_ok = poses_ok && worst_p < 1e-5 && worst_r < 1e-4;
      }
      for (size_t k = 1; k < plan.q.size(); ++k)
        max_jump = std::max(max_jump, (plan.q[k] - plan.q[k - 1]).cwiseAbs().maxCoeff());
      std::printf("      worst end error %.1e m, %.1e rad; max joint step %.4f rad\n", worst_p, worst_r, max_jump);
      expect("every move ends on its waypoint (1e-5 m, 1e-4 rad)", poses_ok);
      expect("step end frames increase", mono);
      expect("no joint jump above 0.05 rad per frame", max_jump < 0.05);
      expect("two gripper events: close then open",
             plan.gripper.size() == 2 && plan.gripper[0].second && !plan.gripper[1].second &&
                 plan.gripper[0].first < plan.gripper[1].first);
      expect("program returns to the home joints",
             (plan.q.back() - q_home).cwiseAbs().maxCoeff() < 1e-3);
    }
  }

  std::printf("[4] Error handling\n");
  {
    TeachProgram p = demoProgram();
    Step bad;
    bad.type = StepType::MOVE;
    bad.name = "nowhere";
    p.steps.insert(p.steps.begin() + 2, bad);
    TeachPlan plan = compileProgram(p, q_home);
    expect("unknown waypoint is reported with its step: " + plan.error,
           !plan.ok && plan.fail_step == 3 && plan.error.find("nowhere") != std::string::npos);

    TeachProgram far;
    far.set("far", waypointFromPose(toolDownPose({-2.0, 0.0, 0.2}, 0.0)));
    Step m;
    m.type = StepType::MOVE;
    m.name = "far";
    far.steps = {m};
    plan = compileProgram(far, q_home);
    expect("unreachable waypoint fails cleanly: " + plan.error, !plan.ok && plan.fail_step == 1);

    expect("empty program is refused", !compileProgram(TeachProgram(), q_home).ok);

    const char* bad_yaml[][2] = {
        {"not: [valid", "syntax error"},
        {"- a\n- b\n", "not a mapping"},
        {"version: 2\n", "version"},
        {"waypoints:\n  a:\n    position: [1, 2]\n    orientation_xyzw: [0,0,0,1]\n", "position length"},
        {"waypoints:\n  a:\n    position: [1, 2, 3]\n    orientation_xyzw: [0,0,0,0]\n", "zero orientation"},
        {"waypoints:\n  a:\n    position: [x, y, z]\n    orientation_xyzw: [0,0,0,1]\n", "non-numeric"},
        {"steps:\n  - fly: away\n", "unknown step key"},
        {"steps:\n  - dwell: -1\n", "negative dwell"},
        {"steps:\n  - gripper: halfway\n", "bad gripper"},
        {"steps:\n  - move: a\n    speed: warp\n", "bad speed"},
        {"steps: 5\n", "steps not a list"},
    };
    for (const auto& c : bad_yaml) {
      TeachProgram t;
      std::string err;
      const bool ok = programFromYaml(c[0], t, err);
      expect(std::string("rejects ") + c[1] + " (\"" + err.substr(0, 48) + "\")", !ok && !err.empty());
    }
    TeachProgram t;
    std::string err;
    expect("an empty file with only a name loads as an empty program",
           programFromYaml("name: x\n", t, err) && t.steps.empty() && t.names().empty());
    TeachProgram dup = demoProgram();
    const size_t n0 = dup.names().size();
    dup.set("a_hover", waypointFromPose(toolDownPose({-0.5, 0.0, 0.2}, 0.0)));
    expect("re-saving a name replaces it in place", dup.names().size() == n0 && dup.names()[0] == "a_hover");
    expect("a waypoint in use is detected", dup.used("a_hover") && !dup.used("not_used"));
  }

  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
