#include "servo_core/drive.hpp"

namespace servo {

namespace {
// Cubic Hermite from (p0, v0) to (p1, v1) over T, evaluated at s = t/T in [0, 1).
inline void hermite(float p0, float v0, float p1, float v1, float T, float s, float& p, float& v) {
  const float s2 = s * s, s3 = s2 * s;
  p = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * T * v0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * T * v1;
  v = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * T * v0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * T * v1) / T;
}
}  // namespace

const char* toString(DriveState s) {
  switch (s) {
    case DriveState::DISABLED: return "DISABLED";
    case DriveState::ENABLING: return "ENABLING";
    case DriveState::RUNNING: return "RUNNING";
    case DriveState::STOPPING: return "STOPPING";
    case DriveState::FAULT: return "FAULT";
  }
  return "?";
}

const char* toString(FaultCode f) {
  switch (f) {
    case FaultCode::NONE: return "NONE";
    case FaultCode::FOLLOWING_ERROR: return "FOLLOWING_ERROR";
  }
  return "?";
}

void Drive::configure(const DriveConfig& c) {
  c_ = c;
  for (int j = 0; j < kAxes; ++j) {
    c_.axis[j].dt = c_.dt;
    axis_[j].configure(c_.axis[j]);
  }
  state_ = DriveState::DISABLED;
  latched_ = false;
  initialised_ = false;
  out_ = DriveOutputs{};
}

void Drive::enterDisabled(bool latch) {
  state_ = DriveState::DISABLED;
  latched_ = latched_ || latch;
}

void Drive::enterEnabling(const DriveInputs& in) {
  state_ = DriveState::ENABLING;
  timer_ = 0.0f;
  latched_ = false;
  out_.fault = FaultCode::NONE;
  out_.fault_axis = -1;
  for (int j = 0; j < kAxes; ++j) {
    axis_[j].reset(in.encoder[j]);
    ref_pos_[j] = axis_[j].position();  // hold where the axis is: no jump on enable
    ref_vel_[j] = 0.0f;
    resync_[j] = true;
    fe_time_[j] = 0.0f;
  }
}

void Drive::torqueOff(const DriveInputs& in) {
  for (int j = 0; j < kAxes; ++j) {
    axis_[j].observe(in.encoder[j]);
    out_.torque[j] = 0.0f;
    out_.position[j] = axis_[j].position();
    out_.velocity[j] = axis_[j].velocity();
    out_.ref_pos[j] = out_.position[j];
    out_.ref_vel[j] = 0.0f;
    if (state_ != DriveState::FAULT) out_.following_error[j] = 0.0f;  // a fault keeps the trip values
  }
  out_.torque_on = false;
  out_.brake_engaged = true;
}

// Cubic Hermite between the last two target sets; past the end, extrapolate with the final
// velocity for at most target_hold_max, then hold.
void Drive::interpolate(const DriveInputs& in, float* p, float* v) {
  if (in.target_seq != last_seq_) {
    last_seq_ = in.target_seq;
    const float T = in.target_period > 1e-4f ? in.target_period : 1e-4f;
    for (int j = 0; j < kAxes; ++j) {
      if (have_targets_) {
        // start the new segment where the reference is now, so a late or early message causes no jump
        float pn = 0.0f, vn = 0.0f;
        const float s = seg_t_ / seg_T_;
        if (s < 1.0f) {
          hermite(seg_p0_[j], seg_v0_[j], seg_p1_[j], seg_v1_[j], seg_T_, s, pn, vn);
        } else {
          const float over = seg_t_ - seg_T_;
          const float h = over < c_.target_hold_max ? over : c_.target_hold_max;
          pn = seg_p1_[j] + seg_v1_[j] * h;
          vn = over < c_.target_hold_max ? seg_v1_[j] : 0.0f;
        }
        seg_p0_[j] = pn;
        seg_v0_[j] = vn;
      } else {
        seg_p0_[j] = in.target_pos[j];
        seg_v0_[j] = in.target_vel[j];
      }
      seg_p1_[j] = in.target_pos[j];
      seg_v1_[j] = in.target_vel[j];
    }
    seg_T_ = T;
    seg_t_ = 0.0f;
    have_targets_ = true;
  }
  for (int j = 0; j < kAxes; ++j) {
    if (!have_targets_) {  // nothing commanded yet: hold the measured position
      p[j] = axis_[j].position();
      v[j] = 0.0f;
      continue;
    }
    const float s = seg_t_ / seg_T_;
    if (s < 1.0f) {
      hermite(seg_p0_[j], seg_v0_[j], seg_p1_[j], seg_v1_[j], seg_T_, s, p[j], v[j]);
    } else {
      const float over = seg_t_ - seg_T_;
      const float h = over < c_.target_hold_max ? over : c_.target_hold_max;
      p[j] = seg_p1_[j] + seg_v1_[j] * h;
      v[j] = over < c_.target_hold_max ? seg_v1_[j] : 0.0f;
    }
  }
  seg_t_ += c_.dt;
}

const DriveOutputs& Drive::step(const DriveInputs& in) {
  if (!initialised_) {
    for (int j = 0; j < kAxes; ++j) axis_[j].reset(in.encoder[j]);
    prev_reset_ = in.reset;
    initialised_ = true;
  }
  if (in.reset && !prev_reset_) reset_pending_ = true;
  if (!in.reset) reset_pending_ = false;  // released before it could be used
  prev_reset_ = in.reset;
  if (in.estop) latched_ = true;  // an E-stop always needs a reset afterwards
  const float dt = c_.dt;

  // ---- transitions that do not need the control law
  switch (state_) {
    case DriveState::DISABLED:
      if (in.enable && !in.estop && (!latched_ || reset_pending_)) {
        reset_pending_ = false;
        enterEnabling(in);
      }
      break;
    case DriveState::FAULT:
      if (in.enable && !in.estop && reset_pending_) {
        reset_pending_ = false;
        enterEnabling(in);
      }
      break;
    case DriveState::ENABLING:
      if (in.estop || !in.enable) enterDisabled(in.estop);  // not moving yet: STO at once
      break;
    case DriveState::RUNNING:
      if (in.estop || !in.enable) {
        state_ = DriveState::STOPPING;
        timer_ = 0.0f;
        still_time_ = 0.0f;
        for (int j = 0; j < kAxes; ++j) {
          const float v = absf(ref_vel_[j]) > absf(axis_[j].velocity()) ? absf(ref_vel_[j]) : absf(axis_[j].velocity());
          stop_v0_[j] = v;
        }
      }
      break;
    case DriveState::STOPPING:
      break;
  }

  if (state_ == DriveState::DISABLED || state_ == DriveState::FAULT) {
    torqueOff(in);
  } else {
    // ---- reference generation
    float tgt_p[kAxes], tgt_v[kAxes];
    interpolate(in, tgt_p, tgt_v);
    timer_ += dt;
    bool all_synced = true;
    for (int j = 0; j < kAxes; ++j) {
      const float tgt = tgt_p[j];
      if (state_ == DriveState::ENABLING) {
        ref_vel_[j] = 0.0f;  // hold position while the brake opens
      } else if (state_ == DriveState::STOPPING) {
        const float env_raw = stop_v0_[j] * (1.0f - timer_ / c_.stop_ramp_time);
        const float env = env_raw > 0.0f ? env_raw : 0.0f;
        ref_vel_[j] = clampf(tgt_v[j], -env, env);
        ref_pos_[j] += ref_vel_[j] * dt;
      } else if (resync_[j]) {
        const float gap = tgt - ref_pos_[j];
        const float step = c_.resync_vel * dt;
        if (absf(gap) <= c_.resync_tol || absf(gap) <= step) {
          ref_pos_[j] = tgt;
          ref_vel_[j] = tgt_v[j];
          resync_[j] = false;
        } else {
          ref_vel_[j] = gap > 0.0f ? c_.resync_vel : -c_.resync_vel;
          ref_pos_[j] += ref_vel_[j] * dt;
        }
      } else {
        ref_pos_[j] = tgt;
        ref_vel_[j] = tgt_v[j];
      }
      all_synced = all_synced && !resync_[j];
    }

    // ---- control law and following-error supervision
    bool fault = false;
    bool still = true;
    for (int j = 0; j < kAxes; ++j) {
      const AxisOutput o = axis_[j].update(ref_pos_[j], ref_vel_[j], in.encoder[j], in.torque_ff[j]);
      out_.torque[j] = o.torque;
      out_.position[j] = o.position;
      out_.velocity[j] = o.velocity;
      out_.ref_pos[j] = ref_pos_[j];
      out_.ref_vel[j] = ref_vel_[j];
      out_.following_error[j] = o.following_error;
      if (absf(o.following_error) > c_.axis[j].following_error_limit) fe_time_[j] += dt;
      else fe_time_[j] = 0.0f;
      if (fe_time_[j] >= c_.fe_debounce && !fault) {
        fault = true;
        out_.fault_axis = static_cast<int8_t>(j);
      }
      still = still && absf(o.velocity) < c_.standstill_vel && ref_vel_[j] == 0.0f;
    }
    out_.torque_on = true;
    out_.brake_engaged = (state_ == DriveState::ENABLING);

    if (fault) {
      state_ = DriveState::FAULT;
      out_.fault = FaultCode::FOLLOWING_ERROR;
      torqueOff(in);  // STO in the same cycle
    } else if (state_ == DriveState::ENABLING && timer_ >= c_.brake_release_time) {
      state_ = DriveState::RUNNING;
      timer_ = 0.0f;
    } else if (state_ == DriveState::STOPPING) {
      still_time_ = still ? still_time_ + dt : 0.0f;
      const bool timed_out = timer_ >= c_.stop_timeout;
      if (still_time_ >= c_.standstill_time || timed_out) {
        out_.stop_timed_out = timed_out && still_time_ < c_.standstill_time;
        enterDisabled(true);
        torqueOff(in);
      }
    }
    out_.ready = (state_ == DriveState::RUNNING) && all_synced;
  }
  if (state_ != DriveState::RUNNING) out_.ready = false;
  out_.state = state_;
  return out_;
}

}  // namespace servo
