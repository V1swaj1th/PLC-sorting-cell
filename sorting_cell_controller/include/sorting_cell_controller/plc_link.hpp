#pragma once
// ROS-free Modbus TCP link to the PLC (libmodbus). See plc/register_map.md for the address map.
//
//   coils 0..10   PLC -> ROS  (written by the ladder program, read by this master)
//   coils 16..25  ROS -> PLC  (written by this master, only ever READ by the ladder program)
//
// The two ranges are disjoint, so the PLC scan never overwrites what ROS wrote and vice versa.
#include <modbus/modbus.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace sorting_cell {

struct PlcInputs {  // ROS -> PLC
  bool start_pb{false}, stop_pb{false}, estop_ok{false}, reset_pb{false};
  bool at_home{false}, cycle_done{false}, fault{false};
  bool ph_pick{false}, ph_place{false}, arm_busy{false};
};

struct PlcOutputs {  // PLC -> ROS
  bool run{false}, estop_active{false}, reset_cmd{false};
  bool st_idle{false}, st_run{false}, st_pick{false}, st_place{false}, st_done{false};
  bool estop_latch{false}, st_fault{false};
};

constexpr int kPlcOutBase = 0;    // first PLC-written coil (coil 0 = lamp_out test coil, ignored)
constexpr int kPlcOutCount = 11;  // coils 0..10
constexpr int kPlcInBase = 16;    // first ROS-written coil
constexpr int kPlcInCount = 10;   // coils 16..25

inline std::string plcStateName(const PlcOutputs& o) {
  if (o.estop_active || o.estop_latch) return "E-STOP";
  if (o.st_fault) return "FAULT";
  if (o.st_done) return "DONE";
  if (o.st_pick) return "PICK";
  if (o.st_place) return "PLACE";
  if (o.st_run) return "RUN";
  if (o.st_idle) return "IDLE";
  return "UNKNOWN";
}

class PlcLink {
 public:
  PlcLink(const std::string& host, int port, int slave = 1) : host_(host), port_(port), slave_(slave) {}
  ~PlcLink() { close(); }
  PlcLink(const PlcLink&) = delete;
  PlcLink& operator=(const PlcLink&) = delete;

  bool connected() const { return ctx_ != nullptr; }
  const std::string& lastError() const { return err_; }

  bool connect() {
    close();
    ctx_ = modbus_new_tcp(host_.c_str(), port_);
    if (!ctx_) {
      err_ = "modbus_new_tcp failed";
      return false;
    }
    modbus_set_slave(ctx_, slave_);
    modbus_set_response_timeout(ctx_, 0, 300000);  // 300 ms
    if (modbus_connect(ctx_) != 0) {
      err_ = std::string("connect: ") + modbus_strerror(errno);
      close();
      return false;
    }
    return true;
  }

  void close() {
    if (ctx_) {
      modbus_close(ctx_);
      modbus_free(ctx_);
      ctx_ = nullptr;
    }
  }

  // One cycle: write the ROS-driven coils, then read the PLC-driven coils. Any Modbus error
  // drops the connection (the caller reconnects) and returns false.
  bool exchange(const PlcInputs& in, PlcOutputs& out) {
    if (!ctx_) return false;
    uint8_t w[kPlcInCount] = {in.start_pb, in.stop_pb,  in.estop_ok, in.reset_pb, in.at_home,
                              in.cycle_done, in.fault, in.ph_pick,  in.ph_place, in.arm_busy};
    for (auto& v : w) v = v ? 1 : 0;
    if (modbus_write_bits(ctx_, kPlcInBase, kPlcInCount, w) != kPlcInCount) return fail("write_bits");
    uint8_t r[kPlcOutCount] = {0};
    if (modbus_read_bits(ctx_, kPlcOutBase, kPlcOutCount, r) != kPlcOutCount) return fail("read_bits");
    out.run = r[1];
    out.estop_active = r[2];
    out.reset_cmd = r[3];
    out.st_idle = r[4];
    out.st_run = r[5];
    out.st_pick = r[6];
    out.st_place = r[7];
    out.st_done = r[8];
    out.estop_latch = r[9];
    out.st_fault = r[10];
    return true;
  }

 private:
  bool fail(const char* what) {
    err_ = std::string(what) + ": " + modbus_strerror(errno);
    close();
    return false;
  }

  std::string host_;
  int port_;
  int slave_;
  modbus_t* ctx_{nullptr};
  std::string err_;
};

}  // namespace sorting_cell
