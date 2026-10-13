// Evidence for the servo layer: runs the drive firmware against the simulated UR5e plant and
// writes CSV logs plus metrics.yaml into a directory, for plot_drive.py.
//
//   drive_report OUT_DIR
//
// 1 step:        0.2 rad shoulder step: P-only, PID, cascade, cascade + gravity feedforward
// 2 antiwindup:  1.0 rad shoulder step with the PID saturating, anti-windup on and off
// 3 tracking:    the default scene's whole sort cycle, 0.5 kg cube attached between grasp and release
// 4 disturbance: 3 kg payload added while holding, and a whole cycle with a 3 kg payload
// 5 estop:       E-stop at the fastest point of a transport move: path stop, standstill, STO
// 6 jam:         the fastest joint jammed there: following error -> fault -> STO
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "servo_core/ur5e_setup.hpp"
#include "sorting_cell_controller/scene.hpp"

using namespace servo_sim;
using servo::DriveInputs;
using servo::DriveOutputs;
using servo::DriveState;

namespace {
const Arr6 kHome = {0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0};
constexpr double kDt = 0.001;

struct Metrics {
  double rise{-1}, overshoot{0}, settle{-1}, ss_error{0};
};
Metrics stepMetrics(const std::vector<double>& t, const std::vector<double>& y, double step) {
  Metrics m;
  double t10 = -1, t90 = -1, peak = 0;
  for (size_t i = 0; i < y.size(); ++i) {
    if (t10 < 0 && y[i] >= 0.1 * step) t10 = t[i];
    if (t90 < 0 && y[i] >= 0.9 * step) t90 = t[i];
    peak = std::max(peak, y[i]);
    if (std::abs(y[i] - step) > 0.02 * step) m.settle = t[i];
  }
  m.rise = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1;
  m.overshoot = std::max(0.0, (peak - step) / step * 100.0);
  double e = 0;
  int n = 0;
  for (size_t i = 0; i < y.size(); ++i)
    if (t[i] >= t.back() - 0.2) {
      e += std::abs(step - y[i]);
      ++n;
    }
  m.ss_error = e / std::max(1, n);
  if (m.settle >= t.back() - 1e-9) m.settle = -1;  // never settled within the window
  return m;
}

servo::DriveConfig commissioning(Tuning t, bool aw = true) {
  servo::DriveConfig c = ur5eDrive(t, aw);
  for (auto& a : c.axis) a.following_error_limit = 10.0f;  // test steps are following errors by definition
  return c;
}

// The sequencer's playback: 50 Hz frames, speed ramps (0.5 s up, 0.3 s down) like the ROS node.
struct Playback {
  const std::vector<sorting_cell::Vector6d>* q{};
  double pos{0}, speed{0};
  void frame(bool allowed) {
    const double target = allowed ? 1.0 : 0.0, old = speed;
    speed = target > speed ? std::min(target, speed + 0.02 / 0.5) : std::max(target, speed - 0.02 / 0.3);
    // trapezoidal integration: the reported velocity (path speed * new speed) matches the position
    pos = std::min(pos + 0.5 * (old + speed), static_cast<double>(q->size() - 1));
  }
  // central difference: the velocity belongs to the same instant as the position
  double frameVel(size_t i, int j) const {
    const size_t a = i > 0 ? i - 1 : 0, b = std::min(i + 1, q->size() - 1);
    return b > a ? ((*q)[b][j] - (*q)[a][j]) / (0.02 * static_cast<double>(b - a)) : 0.0;
  }
  void targets(DriveInputs& in) const {
    const size_t i0 = static_cast<size_t>(pos), i1 = std::min(i0 + 1, q->size() - 1);
    const double f = pos - static_cast<double>(i0);
    for (int j = 0; j < 6; ++j) {
      in.target_pos[j] = static_cast<float>((1 - f) * (*q)[i0][j] + f * (*q)[i1][j]);
      in.target_vel[j] = static_cast<float>(((1 - f) * frameVel(i0, j) + f * frameVel(i1, j)) * speed);
    }
  }
  bool done() const { return pos >= static_cast<double>(q->size() - 1); }
};

struct CycleOptions {
  double payload{0.5};
  int estop_frame{-1};   // E-stop when the playback reaches this plan frame
  int jam_frame{-1};     // jam the joint when the playback reaches this plan frame
  int jam_joint{1};
  double max_time{60};
};
struct CycleResult {
  double fe_max[6]{}, fe_rms[6]{}, cycle_time{0};
  double t_estop{-1}, t_standstill{-1}, t_sto{-1}, stop_dist[6]{};
  double t_fault{-1}, t_jam{-1};
  double v_at_estop{0};
  int fault_axis{-1};
  bool faulted{false};
};

CycleResult runCycle(const sorting_cell::PickPlacePlan& plan, const CycleOptions& o, const std::string& csv) {
  DriveSim sim;
  Arr6 q0;
  for (int j = 0; j < 6; ++j) q0[j] = plan.q[0][j];
  sim.init(q0, ur5eDrive());
  Playback pb;
  pb.q = &plan.q;
  DriveInputs in{};
  in.target_period = 0.02f;
  pb.targets(in);
  std::ofstream f(csv);
  f << "t,state,torque_on,payload";
  for (int j = 0; j < 6; ++j) f << ",ref" << j << ",q" << j << ",v" << j << ",fe" << j << ",tau" << j;
  f << "\n";
  CycleResult r;
  double sq[6]{};
  long n = 0;
  Arr6 q_at_estop{};
  bool estopped = false;
  const long steps = static_cast<long>(o.max_time / kDt);
  for (long k = 0; k < steps; ++k) {
    const double t = k * kDt;
    if (o.estop_frame >= 0 && pb.pos >= o.estop_frame) in.estop = true;
    if (o.jam_frame >= 0 && pb.pos >= o.jam_frame && r.t_jam < 0) {
      sim.plant().setJam(o.jam_joint);
      r.t_jam = t;
    }
    if (k % 20 == 0) {
      pb.frame(sim.drive().outputs().ready && !in.estop);
      pb.targets(in);
      ++in.target_seq;
      bool carrying = false;
      const int fr = static_cast<int>(pb.pos);
      for (size_t i = 0; i < plan.grasp_frame.size(); ++i)
        carrying = carrying || (fr >= plan.grasp_frame[i] && fr < plan.release_frame[i]);
      sim.plant().setPayload(carrying ? o.payload : 0.0);
    }
    const DriveOutputs& d = sim.step(in);
    if (in.estop && !estopped) {
      estopped = true;
      r.t_estop = t;
      q_at_estop = sim.plant().q();
      for (int j = 0; j < 6; ++j) r.v_at_estop = std::max(r.v_at_estop, std::abs(sim.plant().v()[j]));
    }
    if (estopped && r.t_standstill < 0) {
      bool still = true;
      for (int j = 0; j < 6; ++j) still = still && std::abs(sim.plant().v()[j]) < 1e-3;
      if (still) r.t_standstill = t;
    }
    if (estopped && r.t_sto < 0 && !d.torque_on) r.t_sto = t;
    if (d.state == DriveState::FAULT && !r.faulted) {
      r.faulted = true;
      r.t_fault = t;
      r.fault_axis = d.fault_axis;
    }
    if (d.state == DriveState::RUNNING && !estopped) {
      for (int j = 0; j < 6; ++j) {
        r.fe_max[j] = std::max(r.fe_max[j], static_cast<double>(std::abs(d.following_error[j])));
        sq[j] += d.following_error[j] * d.following_error[j];
      }
      ++n;
    }
    if (k % 5 == 0) {
      f << t << "," << static_cast<int>(d.state) << "," << d.torque_on << "," << sim.plant().payload();
      for (int j = 0; j < 6; ++j)
        f << "," << d.ref_pos[j] << "," << sim.plant().q()[j] << "," << sim.plant().v()[j] << ","
          << d.following_error[j] << "," << sim.plant().appliedTorque()[j];
      f << "\n";
    }
    const bool finished = pb.done() && d.ready;
    if (finished && r.cycle_time == 0) r.cycle_time = t;
    if ((finished && t > r.cycle_time + 1.0) || (estopped && t > r.t_estop + 2.0) || (r.faulted && t > r.t_fault + 1.0)) break;
  }
  for (int j = 0; j < 6; ++j) {
    r.fe_rms[j] = std::sqrt(sq[j] / std::max(1L, n));
    if (estopped) r.stop_dist[j] = sim.plant().q()[j] - q_at_estop[j];
  }
  return r;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::printf("usage: drive_report OUT_DIR\n");
    return 2;
  }
  const std::string dir = argv[1];
  std::ofstream M(dir + "/metrics.yaml");
  if (!M) {
    std::printf("cannot write to %s\n", dir.c_str());
    return 1;
  }

  // ---- 1 step responses on the shoulder (joint 2), the joint that carries the arm against gravity
  struct Case {
    const char* name;
    Tuning t;
    bool gff;
  };
  const Case cases[] = {{"p_only", Tuning::P_ONLY, false},
                        {"pid", Tuning::PID, false},
                        {"cascade", Tuning::CASCADE, false},
                        {"cascade_gravity_ff", Tuning::CASCADE, true}};
  const double step = 0.2;
  std::printf("[1] 0.2 rad shoulder step\n");
  M << "step:\n";
  std::ofstream fs(dir + "/step.csv");
  fs << "t,p_only,pid,cascade,cascade_gravity_ff\n";
  std::vector<std::vector<double>> ys;
  std::vector<double> ts;
  for (const Case& c : cases) {
    DriveSim sim;
    sim.init(kHome, commissioning(c.t), c.gff);
    DriveInputs in{};
    for (int j = 0; j < 6; ++j) in.target_pos[j] = static_cast<float>(kHome[j]);
    in.target_seq = 1;
    in.target_period = 0.001f;  // a step within one control period
    for (int k = 0; k < 500; ++k) sim.step(in);
    const double y0 = sim.plant().q()[1];
    in.target_pos[1] = static_cast<float>(y0 + step);
    ++in.target_seq;
    std::vector<double> t, y;
    for (int k = 1; k <= 2500; ++k) {
      sim.step(in);
      t.push_back(k * kDt);
      y.push_back(sim.plant().q()[1] - y0);
    }
    const Metrics m = stepMetrics(t, y, step);
    std::printf("  %-20s rise %6.3f s  overshoot %5.1f %%  settle(2%%) %6.3f s  steady-state error %.2e rad\n", c.name, m.rise,
                m.overshoot, m.settle, m.ss_error);
    M << "  " << c.name << ": {rise_s: " << m.rise << ", overshoot_pct: " << m.overshoot << ", settle_s: " << m.settle
      << ", ss_error_rad: " << m.ss_error << "}\n";
    ys.push_back(y);
    ts = t;
  }
  for (size_t i = 0; i < ts.size(); i += 2) {
    fs << ts[i];
    for (const auto& y : ys) fs << "," << y[i];
    fs << "\n";
  }

  // ---- 2 anti-windup: a step large enough to saturate the torque for a while
  std::printf("[2] anti-windup, 1.0 rad shoulder step, PID saturating at 150 Nm\n");
  M << "antiwindup:\n";
  std::ofstream fa(dir + "/antiwindup.csv");
  fa << "t,with,without,tau_with,tau_without\n";
  std::vector<double> aw_y[2], aw_tau[2], aw_t;
  for (int w = 0; w < 2; ++w) {
    servo::DriveConfig c = commissioning(Tuning::PID, w == 0);
    for (auto& a : c.axis) a.ki *= 3.0f;  // strong integral action makes the effect visible
    DriveSim sim;
    sim.init(kHome, c, false);
    DriveInputs in{};
    for (int j = 0; j < 6; ++j) in.target_pos[j] = static_cast<float>(kHome[j]);
    in.target_seq = 1;
    in.target_period = 0.001f;
    for (int k = 0; k < 2000; ++k) sim.step(in);
    const double y0 = sim.plant().q()[1];
    in.target_pos[1] = static_cast<float>(y0 + 1.0);
    ++in.target_seq;
    aw_t.clear();
    for (int k = 1; k <= 4000; ++k) {
      const DriveOutputs& d = sim.step(in);
      aw_t.push_back(k * kDt);
      aw_y[w].push_back(sim.plant().q()[1] - y0);
      aw_tau[w].push_back(d.torque[1]);
    }
    const Metrics m = stepMetrics(aw_t, aw_y[w], 1.0);
    std::printf("  anti-windup %-3s  overshoot %5.1f %%  settle(2%%) %6.3f s\n", w == 0 ? "on" : "off", m.overshoot, m.settle);
    M << "  " << (w == 0 ? "with" : "without") << ": {overshoot_pct: " << m.overshoot << ", settle_s: " << m.settle << "}\n";
  }
  for (size_t i = 0; i < aw_t.size(); i += 2)
    fa << aw_t[i] << "," << aw_y[0][i] << "," << aw_y[1][i] << "," << aw_tau[0][i] << "," << aw_tau[1][i] << "\n";

  // ---- 3..6 the real sort cycle of the default scene
  sorting_cell::SceneModel model(1);
  const auto plan = sorting_cell::planPickPlace(sorting_cell::cubeTargets(model.scene()),
                                                sorting_cell::planConfig(model.scene()));
  const double planned = static_cast<double>(plan.q.size() - 1) * 0.02;
  auto feLine = [&](const char* name, const CycleResult& r) {
    std::printf("  %-26s max |following error| [mrad]:", name);
    M << "  " << name << ": {cycle_s: " << r.cycle_time << ", fe_max_mrad: [";
    for (int j = 0; j < 6; ++j) {
      std::printf(" %5.2f", r.fe_max[j] * 1e3);
      M << (j ? ", " : "") << r.fe_max[j] * 1e3;
    }
    M << "], fe_rms_mrad: [";
    for (int j = 0; j < 6; ++j) M << (j ? ", " : "") << r.fe_rms[j] * 1e3;
    M << "]}\n";
    std::printf("   cycle %.2f s (planned %.2f s)\n", r.cycle_time, planned);
  };
  std::printf("[3] whole sort cycle of the default scene\n");
  M << "tracking:\n";
  CycleOptions o;
  const CycleResult light = runCycle(plan, o, dir + "/tracking.csv");
  feLine("cube_0.5kg", light);
  std::printf("[4] payload disturbance\n");
  o.payload = 3.0;
  const CycleResult heavy = runCycle(plan, o, dir + "/tracking_3kg.csv");
  feLine("cube_3kg", heavy);
  {
    DriveSim sim;
    sim.init(kHome, ur5eDrive());
    DriveInputs in{};
    for (int j = 0; j < 6; ++j) in.target_pos[j] = static_cast<float>(kHome[j]);
    in.target_seq = 1;
    for (int k = 0; k < 500; ++k) sim.step(in);
    std::ofstream fd(dir + "/disturbance.csv");
    fd << "t,fe1,fe2,fe3,tau1,tau2,tau3\n";
    double peak = 0, recover = -1;
    for (int k = 0; k < 2000; ++k) {
      if (k == 300) sim.plant().setPayload(3.0);
      const DriveOutputs& d = sim.step(in);
      const double e = std::max({std::abs(d.following_error[1]), std::abs(d.following_error[2]), std::abs(d.following_error[3])});
      if (k >= 300) {
        peak = std::max(peak, e);
        if (e > 1e-4) recover = (k - 300) * kDt;
      }
      fd << k * kDt << "," << d.following_error[1] << "," << d.following_error[2] << "," << d.following_error[3] << ","
         << d.torque[1] << "," << d.torque[2] << "," << d.torque[3] << "\n";
    }
    std::printf("  3 kg added while holding: peak following error %.2f mrad, back below 0.1 mrad after %.3f s\n", peak * 1e3,
                recover);
    M << "disturbance_hold: {payload_kg: 3.0, peak_fe_mrad: " << peak * 1e3 << ", recover_s: " << recover << "}\n";
  }

  // the fastest moment of the first transport move (between grasp and release of cube 0)
  int fast = plan.grasp_frame[0];
  double vmax = 0;
  for (int f = plan.grasp_frame[0]; f + 1 < plan.release_frame[0]; ++f) {
    const double v = (plan.q[f + 1] - plan.q[f]).cwiseAbs().maxCoeff() / 0.02;
    if (v > vmax) {
      vmax = v;
      fast = f;
    }
  }
  std::printf("[5] E-stop at the fastest point of the first transport move (planned %.2f rad/s)\n", vmax);
  CycleOptions es;
  es.estop_frame = fast;
  const CycleResult e = runCycle(plan, es, dir + "/estop.csv");
  std::printf("  E-stop at %.2f s, fastest joint %.2f rad/s: standstill after %.3f s, torque off (STO) after %.3f s,"
              " stopping distance [mrad]:", e.t_estop, e.v_at_estop, e.t_standstill - e.t_estop, e.t_sto - e.t_estop);
  for (double d : e.stop_dist) std::printf(" %.1f", d * 1e3);
  std::printf("\n");
  M << "estop: {t_estop_s: " << e.t_estop << ", speed_rad_s: " << e.v_at_estop << ", standstill_after_s: " << e.t_standstill - e.t_estop
    << ", sto_after_s: " << e.t_sto - e.t_estop << "}\n";

  std::printf("[6] joint jam at the same point\n");
  CycleOptions jam;
  jam.jam_frame = fast;
  // jam the joint that moves fastest there
  {
    double best = 0;
    for (int jj = 0; jj < 6; ++jj) {
      const double v = std::abs(plan.q[fast + 1][jj] - plan.q[fast][jj]);
      if (v > best) {
        best = v;
        jam.jam_joint = jj;
      }
    }
  }
  const CycleResult j = runCycle(plan, jam, dir + "/jam.csv");
  std::printf("  joint %d jammed at %.2f s: following-error fault on joint %d after %.3f s, torque off in the same cycle\n",
              jam.jam_joint + 1, j.t_jam, j.fault_axis + 1, j.t_fault - j.t_jam);
  M << "jam: {t_jam_s: " << j.t_jam << ", jammed_joint: " << jam.jam_joint + 1 << ", fault_axis: " << j.fault_axis + 1 << ", fault_after_s: " << j.t_fault - j.t_jam
    << "}\n";
  std::printf("\nwrote %s/{step,antiwindup,tracking,tracking_3kg,disturbance,estop,jam}.csv and metrics.yaml\n", dir.c_str());
  return 0;
}
