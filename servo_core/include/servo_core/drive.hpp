#pragma once
// Six-axis servo drive as firmware: one fixed-rate step() per control period, fixed-size state,
// no heap. It owns the drive state machine and the safety reactions:
//
//   DISABLED --enable, no E-stop (+ reset if a stop or fault is latched)--> ENABLING
//   ENABLING --brake opening time--> RUNNING        (holds the actual position, then re-syncs
//                                                    to the commanded target at a low speed)
//   RUNNING  --E-stop--> STOPPING                   (stop category 1: controlled deceleration)
//   STOPPING --standstill, or stop timeout--> DISABLED  (STO: torque off, brakes applied)
//   RUNNING/STOPPING --following error--> FAULT         (STO, latched until reset)
//
// In STOPPING the drive keeps following the commanded targets (the motion controller brakes
// along the path) but bounds the reference speed by a falling envelope |v| <= v0*(1 - t/T), so
// the axis stops within T even if the targets do not. That is the supervision part of an SS1 stop.
#include <stdint.h>

#include "servo_core/servo_axis.hpp"

namespace servo {

constexpr int kAxes = 6;

enum class DriveState : uint8_t { DISABLED = 0, ENABLING = 1, RUNNING = 2, STOPPING = 3, FAULT = 4 };
enum class FaultCode : uint8_t { NONE = 0, FOLLOWING_ERROR = 1 };

const char* toString(DriveState s);
const char* toString(FaultCode f);

struct DriveConfig {
  AxisConfig axis[kAxes]{};
  float dt{0.001f};
  float stop_ramp_time{0.5f};       // envelope: |v_ref| <= v0*(1 - t/stop_ramp_time) [s]; must be
                                    // longer than the motion controller's own path stop (0.3 s)
  float standstill_vel{0.02f};      // [rad/s]
  float standstill_time{0.05f};     // [s]
  float stop_timeout{1.0f};         // STO after this even if not at standstill [s]
  float brake_release_time{0.10f};  // [s]
  float resync_vel{0.3f};           // [rad/s] approach speed to the target after enabling
  float resync_tol{0.001f};         // [rad]
  float target_hold_max{0.04f};     // a late target: extrapolate at most this long, then hold [s]
  float fe_debounce{0.02f};         // following error must persist this long [s]
};

// Targets arrive at a lower rate (the motion controller's 50 Hz). Each new set gets a new
// target_seq; the drive interpolates between the last two sets with a cubic Hermite segment
// (continuous position and velocity, one target period of latency), as servo drives do in
// interpolated-position mode.
struct DriveInputs {
  float target_pos[kAxes]{};
  float target_vel[kAxes]{};
  uint32_t target_seq{0};           // increment when new targets are written; 0 = none yet
  float target_period{0.02f};       // time between target sets [s]
  float torque_ff[kAxes]{};         // e.g. gravity compensation from a model [Nm]
  int32_t encoder[kAxes]{};
  bool enable{true};
  bool estop{false};                // true = stop requested (E-stop chain open)
  bool reset{false};                // push button level: a push acknowledges a latched stop or fault,
                                    // also if the E-stop clears only while the button is still held
};

struct DriveOutputs {
  float torque[kAxes]{};
  float position[kAxes]{};
  float velocity[kAxes]{};
  float ref_pos[kAxes]{};
  float ref_vel[kAxes]{};
  float following_error[kAxes]{};
  bool torque_on{false};
  bool brake_engaged{true};
  bool ready{false};                // RUNNING and re-synced: motion may proceed
  bool stop_timed_out{false};       // the last stop reached the timeout before standstill
  DriveState state{DriveState::DISABLED};
  FaultCode fault{FaultCode::NONE};
  int8_t fault_axis{-1};
};

class Drive {
 public:
  void configure(const DriveConfig& c);
  const DriveOutputs& step(const DriveInputs& in);
  const DriveOutputs& outputs() const { return out_; }
  DriveState state() const { return state_; }
  const DriveConfig& config() const { return c_; }

 private:
  void enterDisabled(bool latch);
  void enterEnabling(const DriveInputs& in);
  void torqueOff(const DriveInputs& in);
  void interpolate(const DriveInputs& in, float* p, float* v);

  DriveConfig c_{};
  ServoAxis axis_[kAxes]{};
  DriveOutputs out_{};
  DriveState state_{DriveState::DISABLED};
  // setpoint interpolation: segment from (p0, v0) to (p1, v1) over seg_T_
  float seg_p0_[kAxes]{}, seg_v0_[kAxes]{}, seg_p1_[kAxes]{}, seg_v1_[kAxes]{};
  float seg_t_{0.0f}, seg_T_{0.02f};
  uint32_t last_seq_{0};
  bool have_targets_{false};
  float ref_pos_[kAxes]{};
  float ref_vel_[kAxes]{};
  float stop_v0_[kAxes]{};
  bool resync_[kAxes]{};
  float fe_time_[kAxes]{};
  float timer_{0.0f};
  float still_time_{0.0f};
  bool latched_{false};
  bool prev_reset_{false};
  bool reset_pending_{false};  // a reset push not yet used (E-stop still asserted when it rose)
  bool initialised_{false};
};

}  // namespace servo
