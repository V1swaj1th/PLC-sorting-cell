// Tests PlcLink against a software PLC: a real Modbus TCP server (libmodbus) whose memory is
// scanned every 20 ms by a C++ transcription of the ladder program (the spec in
// plc/ladder_spec.md). No ROS and no OpenPLC needed. Prints PASS/FAIL per check.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include <unistd.h>

#include "sorting_cell_controller/plc_link.hpp"

using namespace sorting_cell;

namespace {
std::mutex g_mu;
std::atomic<bool> g_stop{false};
modbus_mapping_t* g_map = nullptr;
bool g_interrupted = false;  // local BOOL in the ladder program

// One scan of the ladder program, rung by rung. Coil numbers = %QX byte*8 + bit.
void ladderScan(uint8_t* c) {
  auto b = [&](int a) { return c[a] != 0; };
  const bool start_pb = b(16), stop_pb = b(17), estop_ok = b(18), reset_pb = b(19), at_home = b(20),
             cycle_done = b(21), r_fault = b(22), ph_pick = b(23), ph_place = b(24);
  bool run = b(1), latch = b(9);
  const bool estop_active = !estop_ok;                      // R1  NC contact -> coil
  if (estop_active) latch = true;                           // R2  S
  const bool reset_cmd = reset_pb && !estop_active;         // R3
  if (reset_cmd) latch = false;                             // R4  R
  if (run && !at_home) g_interrupted = true;                // R5  S
  if (at_home) g_interrupted = false;                       // R6  R
  const bool can_start = at_home || g_interrupted;          // R7
  run = ((start_pb && can_start) || run) && !stop_pb && !latch && !estop_active && !r_fault &&
        !cycle_done;                                        // R8
  c[1] = run;
  c[2] = estop_active;
  c[3] = reset_cmd;
  c[4] = !run && !estop_active && !latch && !r_fault && !cycle_done;  // R9  st_idle
  c[5] = run && !ph_pick && !ph_place;                      // R10 st_run
  c[6] = run && ph_pick;                                    // R11 st_pick
  c[7] = run && ph_place;                                   // R12 st_place
  c[8] = cycle_done;                                        // R13 st_done
  c[9] = latch;
  c[10] = r_fault;                                          // R14 st_fault
}

void serverThread(int port) {
  modbus_t* ctx = modbus_new_tcp("127.0.0.1", port);
  g_map = modbus_mapping_new(64, 0, 0, 0);
  int s = modbus_tcp_listen(ctx, 1);
  modbus_tcp_accept(ctx, &s);
  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
  while (!g_stop) {
    const int rc = modbus_receive(ctx, query);
    if (rc > 0) {
      std::lock_guard<std::mutex> lk(g_mu);
      modbus_reply(ctx, query, rc, g_map);
    } else if (rc == -1) {
      break;
    }
  }
  close(s);
  modbus_close(ctx);
  modbus_free(ctx);
}

void scanThread() {
  while (!g_stop) {
    if (g_map) {
      std::lock_guard<std::mutex> lk(g_mu);
      ladderScan(g_map->tab_bits);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

int g_pass = 0, g_fail = 0;
PlcLink* g_link = nullptr;
PlcInputs g_in;
PlcOutputs g_out;

void step(int scans = 4) {
  for (int i = 0; i < scans; ++i) {
    if (!g_link->exchange(g_in, g_out)) {
      std::printf("Modbus error: %s\n", g_link->lastError().c_str());
      std::exit(2);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
}
void pulse(bool PlcInputs::*field) {
  g_in.*field = true;
  step();
  g_in.*field = false;
  step();
}
void expect(const char* what, bool cond) {
  std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
  (cond ? g_pass : g_fail)++;
}
void expectState(const char* what, const char* state) {
  const std::string s = plcStateName(g_out);
  const bool ok = (s == state);
  std::printf("  [%s] %s (state %s%s%s)\n", ok ? "PASS" : "FAIL", what, s.c_str(), ok ? "" : ", expected ",
              ok ? "" : state);
  (ok ? g_pass : g_fail)++;
}
}  // namespace

int main() {
  const int port = 15020;
  std::thread srv(serverThread, port);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::thread scan(scanThread);

  PlcLink link("127.0.0.1", port);
  g_link = &link;
  if (!link.connect()) {
    std::printf("connect failed: %s\n", link.lastError().c_str());
    return 2;
  }

  std::printf("1. Power-up: E-stop loop healthy but latch set during the first scans\n");
  g_in.estop_ok = false;
  g_in.at_home = true;
  step();
  expect("estop_active while loop open", g_out.estop_active);
  expectState("state E-STOP", "E-STOP");
  g_in.estop_ok = true;
  step();
  expect("estop_active cleared when loop closes", !g_out.estop_active);
  expect("latch still set (reset required)", g_out.estop_latch);
  expectState("still E-STOP until reset", "E-STOP");

  std::printf("2. Start is refused while the E-stop latch is set\n");
  pulse(&PlcInputs::start_pb);
  expect("run stays off", !g_out.run);

  std::printf("3. Reset clears the latch\n");
  g_in.reset_pb = true;
  step();
  expect("reset_cmd active during the push", g_out.reset_cmd);
  g_in.reset_pb = false;
  step();
  expect("latch cleared", !g_out.estop_latch);
  expectState("state IDLE", "IDLE");

  std::printf("4. Start at home\n");
  pulse(&PlcInputs::start_pb);
  expect("run set and held after the button is released", g_out.run);
  expectState("state RUN", "RUN");
  g_in.at_home = false;
  g_in.arm_busy = true;
  g_in.ph_pick = true;
  step();
  expectState("PICK phase", "PICK");
  g_in.ph_pick = false;
  g_in.ph_place = true;
  step();
  expectState("PLACE phase", "PLACE");

  std::printf("5. E-stop mid-motion\n");
  g_in.estop_ok = false;
  step();
  expect("run dropped", !g_out.run);
  expectState("state E-STOP", "E-STOP");
  g_in.estop_ok = true;
  g_in.arm_busy = false;
  step();
  pulse(&PlcInputs::start_pb);
  expect("start refused before reset", !g_out.run);
  pulse(&PlcInputs::reset_pb);
  expect("latch cleared by reset", !g_out.estop_latch);
  expect("run still off after reset (needs a start)", !g_out.run);
  pulse(&PlcInputs::start_pb);
  expect("start accepted away from home: resume after interruption", g_out.run);

  std::printf("6. Stop button mid-cycle, then resume\n");
  pulse(&PlcInputs::stop_pb);
  expect("run dropped by stop", !g_out.run);
  pulse(&PlcInputs::start_pb);
  expect("resume allowed", g_out.run);

  std::printf("7. Cycle done\n");
  g_in.ph_place = false;
  g_in.at_home = true;
  g_in.cycle_done = true;
  step();
  expect("run dropped", !g_out.run);
  expectState("state DONE", "DONE");
  pulse(&PlcInputs::start_pb);
  expect("start refused while cycle_done is set", !g_out.run);
  g_in.cycle_done = false;
  step();
  expectState("IDLE after the ROS side re-homes", "IDLE");

  std::printf("8. Fault interlock\n");
  g_in.fault = true;
  step();
  expectState("state FAULT", "FAULT");
  pulse(&PlcInputs::start_pb);
  expect("start refused with a fault", !g_out.run);
  g_in.fault = false;
  step();

  std::printf("9. Interlock: start refused when the arm is not at home and no cycle was interrupted\n");
  // Fresh PLC state: clear the interrupted flag by being at home, then move away without run.
  g_in.at_home = false;
  step();
  pulse(&PlcInputs::start_pb);
  expect("run stays off", !g_out.run);
  g_in.at_home = true;
  step();
  pulse(&PlcInputs::start_pb);
  expect("run starts once at home", g_out.run);

  std::printf("10. Link failure is reported\n");
  g_stop = true;
  link.close();
  srv.join();
  scan.join();
  PlcLink dead("127.0.0.1", port);
  expect("connect to a dead port fails cleanly", !dead.connect());

  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
