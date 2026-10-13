// gtest suite for the servo firmware (PID, axis, drive state machine) running against the
// simulated UR5e joint plant.
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>

#include "servo_core/ur5e_setup.hpp"

// ---- allocation counter: every operator new in this test binary goes through here
static std::atomic<long> g_allocs{0};
void* operator new(std::size_t n) {
  ++g_allocs;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using namespace servo;
using namespace servo_sim;

namespace {
const Arr6 kHome = {0.0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0.0};

DriveInputs holdAt(const Arr6& q) {
  DriveInputs in{};
  for (int j = 0; j < 6; ++j) in.target_pos[j] = static_cast<float>(q[j]);
  in.target_seq = 1;
  in.target_period = 0.001f;  // the tests write a new target every control period
  return in;
}
// marks the targets in `in` as a new set
void sent(DriveInputs& in) { ++in.target_seq; }

void run(DriveSim& sim, DriveInputs& in, int steps) {
  for (int k = 0; k < steps; ++k) sim.step(in);
}

// Step tests command an instant jump, which is a following error by definition: widen the
// supervision limit for them, as when commissioning a drive with test steps.
DriveConfig stepConfig(Tuning t) {
  DriveConfig c = ur5eDrive(t);
  for (int j = 0; j < kAxes; ++j) c.axis[j].following_error_limit = 10.0f;
  return c;
}

// first-order plant y' = (u - y)/tau, for the PID-only tests
struct Lag {
  float y{0.0f};
  float step(float u, float dt, float tau) { return y += (u - y) * dt / tau; }
};
}  // namespace

// ------------------------------------------------------------------ PID
TEST(Pid, ProportionalAndSaturation) {
  PidConfig c;
  c.kp = 2.0f;
  c.out_min = -1.0f;
  c.out_max = 1.0f;
  Pid p(c);
  EXPECT_FLOAT_EQ(p.update(0.25f, 0.0f), 0.5f);
  for (float e = -10.0f; e <= 10.0f; e += 0.37f) {
    const float u = p.update(e, 0.0f, 3.0f * e);
    EXPECT_LE(u, 1.0f);
    EXPECT_GE(u, -1.0f);
  }
}

TEST(Pid, NoDerivativeKickOnSetpointStep) {
  PidConfig c;
  c.kp = 1.0f;
  c.kd = 5.0f;
  c.out_min = -1e6f;
  c.out_max = 1e6f;
  Pid p(c);
  p.update(0.0f, 0.0f);
  const float u = p.update(1.0f, 0.0f);  // setpoint jumps, measurement does not
  EXPECT_FLOAT_EQ(u, 1.0f);              // only the P part reacts
}

TEST(Pid, DerivativeIsFiltered) {
  PidConfig c;
  c.kd = 1.0f;
  c.d_cutoff_hz = 50.0f;
  c.out_min = -1e9f;
  c.out_max = 1e9f;
  Pid p(c);
  p.update(0.0f, 0.0f);
  const float raw = 1.0f / c.dt;  // unfiltered derivative of a 1.0 measurement step
  const float u1 = p.update(0.0f, 1.0f);
  EXPECT_LT(std::abs(u1), 0.3f * raw);  // first-order filter at 50 Hz takes ~24 % in one 1 ms step
  float u = u1;
  for (int k = 0; k < 100; ++k) u = p.update(0.0f, 1.0f);
  EXPECT_LT(std::abs(u), 1e-3f * raw);  // and decays once the measurement is constant
}

TEST(Pid, AntiWindupLimitsOvershootAfterSaturation) {
  auto overshoot = [](bool aw) {
    PidConfig c;
    c.kp = 2.0f;
    c.ki = 8.0f;
    c.out_min = -1.0f;
    c.out_max = 1.0f;
    c.kaw = aw ? -1.0f : 0.0f;
    Pid p(c);
    Lag plant;
    float peak = 0.0f;
    for (int k = 0; k < 8000; ++k) {  // setpoint 0.9 needs u = 0.9: long saturated rise
      const float u = p.update(0.9f, plant.y);
      plant.step(u, c.dt, 0.5f);
      peak = std::max(peak, plant.y);
    }
    return std::make_pair(peak - 0.9f, p.integrator());
  };
  const auto with = overshoot(true), without = overshoot(false);
  EXPECT_LT(with.first, 0.02f);
  EXPECT_GT(without.first, 3.0f * with.first);
  EXPECT_NEAR(with.second, 0.9f, 0.01f);  // the integrator ends where it should
}

TEST(Pid, BumplessReset) {
  PidConfig c;
  c.kp = 1.0f;
  c.ki = 1.0f;
  c.out_min = -100.0f;
  c.out_max = 100.0f;
  Pid p(c);
  p.reset(0.0f, 20.0f);
  EXPECT_NEAR(p.update(0.0f, 0.0f), 20.0f, 1e-4f);  // holds the preloaded output, no jump
}

// ------------------------------------------------------------------ axis + plant
TEST(Axis, VelocityEstimateTracksConstantSpeed) {
  AxisConfig c;
  c.rad_per_count = 1.5e-5f;
  ServoAxis a;
  a.configure(c);
  a.reset(0);
  const double v = 0.8;  // rad/s
  for (int k = 1; k <= 200; ++k) a.observe(static_cast<int32_t>(std::lround(v * k * 0.001 / 1.5e-5)));
  EXPECT_NEAR(a.velocity(), v, 0.01 * v);
}

struct StepResult {
  double overshoot, ss_error, settle_time;
};
static StepResult stepJ2(Tuning t, bool gravity_ff, double step = 0.2) {
  DriveSim sim;
  sim.init(kHome, stepConfig(t), gravity_ff);
  DriveInputs in = holdAt(kHome);
  run(sim, in, 300);  // enable and settle
  const double q0 = sim.plant().q()[1];
  in.target_pos[1] = static_cast<float>(q0 + step);
  sent(in);
  double peak = 0.0, settle = 0.0;
  for (int k = 1; k <= 3000; ++k) {
    sim.step(in);
    const double y = sim.plant().q()[1] - q0;
    peak = std::max(peak, y);
    if (std::abs(y - step) > 0.02 * step) settle = k * 0.001;
  }
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);
  return {(peak - step) / step, std::abs(sim.plant().q()[1] - q0 - step), settle};
}

TEST(Axis, CascadeStepConvergesWithoutOvershoot) {
  const StepResult r = stepJ2(Tuning::CASCADE, true);
  EXPECT_LT(r.ss_error, 3e-5);   // about two encoder counts
  EXPECT_LT(r.overshoot, 0.05);
  EXPECT_LT(r.settle_time, 0.6);
}

TEST(Axis, PonlyLeavesGravityErrorAndIntegralRemovesIt) {
  const StepResult p = stepJ2(Tuning::P_ONLY, false);
  const StepResult pid = stepJ2(Tuning::PID, false);
  EXPECT_GT(p.ss_error, 1e-3);   // P-only holds the shoulder against gravity with an offset
  EXPECT_LT(pid.ss_error, 5e-5); // the integrator removes it
}

TEST(Axis, TorqueLimitRespectedOnLargeStep) {
  DriveSim sim;
  sim.init(kHome, stepConfig(Tuning::CASCADE));
  DriveInputs in = holdAt(kHome);
  run(sim, in, 300);
  in.target_pos[1] += 1.5f;  // large step: the loops saturate
  sent(in);
  double peak_cmd = 0.0, peak_vel = 0.0;
  for (int k = 0; k < 3000; ++k) {
    const DriveOutputs& o = sim.step(in);
    peak_cmd = std::max(peak_cmd, static_cast<double>(std::abs(o.torque[1])));
    peak_vel = std::max(peak_vel, std::abs(sim.plant().v()[1]));
    EXPECT_LE(std::abs(sim.plant().appliedTorque()[1]), 150.0 + 1e-9);
  }
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);
  EXPECT_NEAR(sim.plant().q()[1], kHome[1] + 1.5, 1e-3);  // and it got there
  EXPECT_LE(peak_cmd, 150.0 + 1e-3);
  EXPECT_GT(peak_cmd, 140.0);    // it really was at the limit
  EXPECT_LT(peak_vel, 3.14 * 1.05);
}

// ------------------------------------------------------------------ drive state machine
static DriveSim enabledSim(DriveInputs& in) {
  DriveSim sim;
  sim.init(kHome, ur5eDrive());
  in = holdAt(kHome);
  run(sim, in, 300);
  return sim;
}

TEST(Drive, EnablesHoldsAndReportsReady) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);
  EXPECT_TRUE(sim.drive().outputs().ready);
  for (int j = 0; j < 6; ++j) EXPECT_NEAR(sim.plant().q()[j], kHome[j], 2e-3) << "joint " << j;
}

TEST(Drive, EstopIsControlledStopThenSto) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  // move J1 at 1.5 rad/s, like a trajectory would command it
  float p = in.target_pos[0];
  for (int k = 0; k < 800; ++k) {
    in.target_vel[0] = 1.5f;
    p += 1.5f * 0.001f;
    in.target_pos[0] = p;
    sent(in);
    sim.step(in);
  }
  ASSERT_NEAR(sim.plant().v()[0], 1.5, 0.05);
  in.estop = true;
  double t_sto = -1.0, max_decel = 0.0, v_prev = sim.plant().v()[0];
  for (int k = 1; k <= 2000; ++k) {
    // the motion controller brakes along the path in 0.3 s (as the sequencer does)
    const float s = std::max(0.0f, 1.0f - static_cast<float>(k) * 0.001f / 0.3f);
    in.target_vel[0] = 1.5f * s;
    p += in.target_vel[0] * 0.001f;
    in.target_pos[0] = p;
    sent(in);
    const DriveOutputs& o = sim.step(in);
    max_decel = std::max(max_decel, std::abs(sim.plant().v()[0] - v_prev) / 0.001);
    v_prev = sim.plant().v()[0];
    if (!o.torque_on && t_sto < 0.0) t_sto = k * 0.001;
  }
  EXPECT_GT(t_sto, 0.25);                 // not an instant freeze: it decelerated first
  EXPECT_LT(t_sto, 1.0);                  // and switched the torque off once stopped
  EXPECT_FALSE(sim.drive().outputs().stop_timed_out);
  EXPECT_EQ(sim.drive().state(), DriveState::DISABLED);
  EXPECT_TRUE(sim.drive().outputs().brake_engaged);
  EXPECT_LT(max_decel, 15.0);             // controlled, not a crash stop
  EXPECT_DOUBLE_EQ(sim.plant().v()[0], 0.0);
}

TEST(Drive, StopEnvelopeStopsEvenIfTargetsKeepMoving) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  float p = in.target_pos[0];
  for (int k = 0; k < 800; ++k) {
    in.target_vel[0] = 1.5f;
    p += 1.5f * 0.001f;
    in.target_pos[0] = p;
    sent(in);
    sim.step(in);
  }
  in.estop = true;
  int k = 0;
  for (; k < 3000 && sim.drive().outputs().torque_on; ++k) {
    p += 1.5f * 0.001f;  // a misbehaving controller that does not stop
    in.target_pos[0] = p;
    sent(in);
    sim.step(in);
  }
  EXPECT_LT(k * 0.001, 1.0 + 1e-6);          // STO by the timeout at the latest
  EXPECT_LT(std::abs(sim.plant().v()[0]), 0.1);  // and the envelope had already slowed it down
}

TEST(Drive, LatchedUntilResetThenBumplessRestart) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  in.estop = true;
  run(sim, in, 500);
  ASSERT_EQ(sim.drive().state(), DriveState::DISABLED);
  in.estop = false;
  run(sim, in, 200);
  EXPECT_EQ(sim.drive().state(), DriveState::DISABLED) << "a released E-stop alone must not restart";
  const Arr6 q_before = sim.plant().q();
  in.reset = true;
  double max_jump = 0.0;
  for (int k = 0; k < 600; ++k) {
    sim.step(in);
    for (int j = 0; j < 6; ++j) max_jump = std::max(max_jump, std::abs(sim.plant().q()[j] - q_before[j]));
  }
  in.reset = false;
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);
  EXPECT_TRUE(sim.drive().outputs().ready);
  EXPECT_LT(max_jump, 2e-3);  // gravity feedforward: the brake opens without a sag or jump
}

TEST(Drive, ResetWorksWhicheverSignalArrivesFirst) {
  // PLC reset: reset_cmd and the cleared E-stop arrive on two topics, in either order
  for (int order = 0; order < 2; ++order) {
    DriveInputs in;
    DriveSim sim = enabledSim(in);
    in.estop = true;
    run(sim, in, 500);
    ASSERT_EQ(sim.drive().state(), DriveState::DISABLED);
    if (order == 0) {
      in.reset = true;  // reset first, E-stop still asserted for a few ms
      run(sim, in, 30);
      in.estop = false;
    } else {
      in.estop = false;  // E-stop first
      run(sim, in, 30);
      in.reset = true;
    }
    run(sim, in, 400);
    in.reset = false;
    run(sim, in, 100);
    EXPECT_EQ(sim.drive().state(), DriveState::RUNNING) << "order " << order;
  }
  // a push that ends while the E-stop is still asserted does not count later
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  in.estop = true;
  run(sim, in, 500);
  in.reset = true;
  run(sim, in, 30);
  in.reset = false;
  run(sim, in, 30);
  in.estop = false;
  run(sim, in, 300);
  EXPECT_EQ(sim.drive().state(), DriveState::DISABLED);
}

TEST(Drive, FollowingErrorFaultOnJamAndRecovery) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  sim.plant().setJam(1);
  float p = in.target_pos[1];
  int k = 0;
  for (; k < 2000 && sim.drive().state() != DriveState::FAULT; ++k) {
    in.target_vel[1] = 0.5f;
    p += 0.5f * 0.001f;
    in.target_pos[1] = p;
    sent(in);
    sim.step(in);
  }
  ASSERT_EQ(sim.drive().state(), DriveState::FAULT);
  EXPECT_EQ(sim.drive().outputs().fault, FaultCode::FOLLOWING_ERROR);
  EXPECT_EQ(sim.drive().outputs().fault_axis, 1);
  // 0.03 rad at 0.5 rad/s is 60 ms, plus the 20 ms debounce
  EXPECT_NEAR(k * 0.001, 0.08, 0.01);
  EXPECT_FALSE(sim.drive().outputs().torque_on);
  EXPECT_GT(std::abs(sim.drive().outputs().following_error[1]), 0.03f);  // the trip value is kept for diagnosis
  // reset with the jam still there: the fault comes back
  in.target_vel[1] = 0.0f;
  sent(in);
  in.reset = true;
  run(sim, in, 5);
  in.reset = false;
  run(sim, in, 1500);
  EXPECT_EQ(sim.drive().state(), DriveState::FAULT);
  // clear the jam, hold the current position, reset: back to running
  sim.plant().setJam(-1);
  in.target_pos[1] = static_cast<float>(sim.plant().q()[1]);
  sent(in);
  in.reset = true;
  run(sim, in, 5);
  in.reset = false;
  run(sim, in, 500);
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);
}

TEST(Drive, RejectsPayloadDisturbance) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  run(sim, in, 500);
  sim.plant().setPayload(3.0);  // 3 kg, unknown to the controller's gravity model
  double peak = 0.0;
  for (int k = 0; k < 1500; ++k) {
    const DriveOutputs& o = sim.step(in);
    for (int j = 0; j < 6; ++j) peak = std::max(peak, static_cast<double>(std::abs(o.following_error[j])));
  }
  EXPECT_EQ(sim.drive().state(), DriveState::RUNNING);  // no fault
  EXPECT_LT(peak, 0.01);
  for (int j = 0; j < 6; ++j) EXPECT_LT(std::abs(sim.drive().outputs().following_error[j]), 5e-5) << "joint " << j;
}

TEST(Drive, InterpolatesTargetsSmoothly) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  // 50 Hz targets of a constant-acceleration move: the reference between them must be smooth
  in.target_period = 0.02f;
  const float a = 2.0f;
  float t = 0.0f, prev_v = 0.0f, worst_jump = 0.0f;
  const float p0 = in.target_pos[0];
  for (int k = 0; k < 600; ++k) {
    if (k % 20 == 0) {
      t = static_cast<float>(k) * 0.001f;
      in.target_pos[0] = p0 + 0.5f * a * t * t;
      in.target_vel[0] = a * t;
      sent(in);
    }
    const DriveOutputs& o = sim.step(in);
    if (k > 40) worst_jump = std::max(worst_jump, std::abs(o.ref_vel[0] - prev_v));
    prev_v = o.ref_vel[0];
  }
  // a 2 rad/s^2 ramp changes the velocity by 0.002 rad/s per ms; steps of 0.04 rad/s at the
  // 50 Hz boundaries (no interpolation) would show up here
  EXPECT_LT(worst_jump, 0.005f);
  EXPECT_NEAR(sim.drive().outputs().ref_vel[0], a * (0.6f - 0.02f), 0.05f);  // one period of latency
}

TEST(Firmware, NoHeapAllocationInTheControlLoop) {
  DriveInputs in;
  DriveSim sim = enabledSim(in);
  const long before = g_allocs.load();
  float p = in.target_pos[0];
  for (int k = 0; k < 10000; ++k) {
    in.target_vel[0] = 0.4f;
    p += 0.4f * 0.001f;
    in.target_pos[0] = p;
    sent(in);
    in.estop = (k > 6000 && k < 7000);
    in.reset = (k > 7500 && k < 7510);
    sim.step(in);
  }
  EXPECT_EQ(g_allocs.load() - before, 0) << "operator new was called inside the control loop";
}
