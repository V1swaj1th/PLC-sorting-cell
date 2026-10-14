# Audit report

Audit of `plc-sorting-cell` at commit `a3b7982` (branch `main`), done on the `audit` branch.
Status: **Phases 1 to 3 done; behaviour changes P1 to P9 wait for approval; README rewrite (Phase 4)
follows the approved fixes.**

## Summary verdict

The project is genuine where it matters most. Motion comes from the controller's own FK, Jacobian
and IK, verified against the official UR5e description to 1e-10 m. The sequencer plans only from
what the vision node detects in pixels, the sort decision uses the detected colour, and every
plotted number is computed by code in the repository. The E-stop really passes through the PLC.

What is not sound yet is the safety story around failures, and some claims went beyond the code:

- **The committed ladder program is out of date.** It has no drive alarm, so the README's PLC
  result could not be reproduced from the repository.
- **The E-stop is a software input.** It was described as a loop the PC cannot override.
- **No watchdogs anywhere.** Neither the PLC nor the ROS nodes notice a dead bridge or a dead node.
  If the bridge dies mid-motion, the arm keeps moving.
- **`at_home` and the gripper are inferred.** `at_home` (used by a PLC interlock) and the grasp come
  from the playback position, not from the measured arm.
- **Three of the "tests" could never fail, and the Python tests crashed `colcon test`.**

All of these are fixable in a few hours, and none of them requires removing a feature.

## How this was verified

| What | Where it ran | Status |
|---|---|---|
| All ROS-free code: kinematics, IK, planner, scene rules, teach, Modbus link, servo firmware and plant, vision, dashboard core | Sandbox, Ubuntu 24.04, GCC 13, Eigen 3.4, yaml-cpp, libmodbus, OpenCV 4.x; built from the clean clone with `-Wall -Wextra -Wpedantic` (no warnings) | Run |
| Multi-node behaviour (real `scene_server`, `sorting_sequencer`, `joint_drive`, `plc_bridge` source, Modbus to a software PLC running the ladder) | Sandbox, in-process stand-in for `rclcpp` with virtual time | Run. Evidence only, the harness is not in the repo (see P7) |
| `colcon build`, `colcon test`, launch files, RViz, real `rclcpp` | Not possible here: the ROS package servers are blocked from the sandbox | **Untested here.** Covered by the CI workflow added on this branch and by your container |
| Ladder on real OpenPLC | Your container, `plc/plc_check.py` | 24/28 before the ladder update (the drive-alarm section failed). You updated the ladder afterwards, but the 28/28 output was not shared and the updated project is **not** in the repo |

The software PLC used here is a hand transcription of the ladder spec. It checks the bridge and the
nodes, **not** the ladder itself. Only `plc_check.py` against OpenPLC checks the ladder.

## Phase 1: shortcut table

| # | Location | What it fakes or assumes | Verdict |
|---|---|---|---|
| 1 | `plc/sorting_cell/pous/programs/main.ld` | Committed ladder is the M5 version: no `drive_alarm`, `drive_fault`, R15/R16. README claimed 28/28 for a ladder that is not in the repo | **Must fix**: export the current project from OpenPLC Editor and commit it (needs you) |
| 2 | README "nothing the PC does can override an open E-stop loop" | `estop_ok` (coil 18) is written by `plc_bridge` from the ROS topic `panel/estop`, so the E-stop button is a software input | **Relabelled** (commit `197870e`); a hardware E-stop is impossible in this setup |
| 3 | `sorting_sequencer.cpp:103-110`, `joint_drive.cpp:72-77` | Last `plc/run`, `plc/estop` and `drive/ready` values are trusted forever. Test: bridge killed mid-transit, arm kept moving | **Must fix** (P1) |
| 4 | Ladder (no watchdog) | Modbus coils keep their last value, so a dead bridge leaves `estop_ok=1` and `run=1`. The "normally-closed loop" only fails safe on a PLC cold start. Test: PLC `run` stayed 1 after the bridge, the sequencer or the drive died | **Must fix** (P2), or state it as a limitation |
| 5 | `sorting_sequencer.cpp:336` | `at_home` is computed from the playback state (`WAITING`, `DONE`, frame < 0.5), not from the joints, and the PLC start interlock trusts it | Should fix (P3) |
| 6 | `sorting_sequencer.cpp:317-331` | The grasp is the plan frame index: the cube attaches even if the actual tool is off target (the 30 mrad following-error limit allows about 25 mm at the tool). Playback keeps advancing during the 0.3 s stop ramp, so an E-stop during a dwell still closes or opens the gripper (reproduced) | Should fix (P4) |
| 7 | `vision_node.py:44` | The camera renders the scene file once per revision; after a cycle the cubes "reappear" at their start positions, and nothing senses that a cube reached its bin | Acceptable simulation simplification, **documented** |
| 8 | `cube_detector.py` `pixel_to_table` / `table_to_pixel` | Renderer and detector share one camera model, so the position test proves self-consistency, not calibration | Acceptable, **relabelled**; an independent hand-computed-point check is added |
| 9 | `scene_server` / `checkPlan` | Scenes are pre-checked with the true positions (not the detections) | Acceptable: this is layout validation; execution still uses detections |
| 10 | `ik_test`, `traj_test`, `sort_plan_test` | Printed numbers and always returned 0 | **Fixed** (`405b4f5`) |
| 11 | README results | Stale vision numbers; anti-windup row hid that Ki was tripled; E-stop row overstated the speed; planned cycle time not labelled as planned | **Fixed** (`197870e`) |
| 12 | `drive_report.cpp` | Servo results are against the project's own plant model (software-in-the-loop); the controller was tuned on the same model | Acceptable, must stay labelled as SIL |
| 13 | Teach mode | Gripper is a marker colour; not PLC-gated, no E-stop; writes `joint_states` directly (no drive) | Acceptable, already documented |

Checked and found **not** faked:
- The sort decision uses the colour vision detected (`planPickPlace` uses `CubeTarget.color`, which comes from `vision/detections`).
- No hardcoded poses bypass the IK: every frame of the plan is an IK solution, and the home pose is the only fixed joint vector.
- Plots read CSVs that the report programs compute; no plotted values are hardcoded.
- The bridge publishes `plc/estop` from the PLC's latch, not from its own input.
- No `try`/`catch` hides a failure: every `catch` reports the error and refuses the input.

## Phase 2: engineering correctness

| Item | Result | Evidence (command, output) |
|---|---|---|
| DH parameters vs official UR5e | **PASS** | `d1 0.1625, a2 -0.425, a3 -0.3922, d4 0.1333, d5 0.0997, d6 0.0996` match `ur_description/config/ur5e/default_kinematics.yaml` |
| FK vs URDF joint chain, 20,000 random joint vectors | **PASS**: 2.2e-10 m, 6.2e-10 rad | `kinematics_report …/default_kinematics.yaml`, line A; gtest `ForwardKinematicsMatchesUrdfChain` (catches a 1 mm change in d6) |
| FK vs TF on a live run | PASS on your earlier run (5.8e-11 m static); not re-run here | `fk_checker` needs ROS |
| Jacobian vs central finite differences (position and rotation rows), 5000 joint vectors | **PASS**: 2.9e-10 | `kinematics_report`, line B; gtest |
| IK, tool-down table poses (the cell's use), seed home | **PASS**: 2000/2000 | `kinematics_report`, line C |
| IK, random joint vectors | **Reported honestly**: 91.3 % (seed within 0.1 rad), 86.1 % (0.5 rad), 76.2 % (seed home). Failed solves from a near seed end within 1 mm | line C |
| IK near singularities | **Bounded but slow**: wrist (q5≈0) 26.9 %, elbow (q3≈0) 11.4 % reach 1e-5 m in 500 iterations; residual < 1 mm; DLS step stays < 0.2 rad where the undamped step is 9.8 rad | line C, `ik_test` [4] |
| Joint-limit handling | **PASS**: no IK result outside the limits (2000 targets at the elbow limit, all campaigns) | line C |
| Units and frames | **PASS**: metres and radians throughout; FK frame is `base` (UR convention, base_link rotated π about z), verified by the URDF check; cube yaw folded mod 90° | code review + line A |
| Velocity and acceleration limits | **PARTIAL**: Cartesian limits enforced (tool speed 0.400 m/s at a 0.40 limit); **joint speed and acceleration not enforced**. Random scenes: peak joint speed max 2.97 rad/s (limit 3.14; the drive's setpoint limit is 2.9), joint acceleration up to 8.4 rad/s² | `kinematics_report` line D; sweep at 1.0 m/s gives 3.39 rad/s |
| Trapezoid profile incl. short moves (no cruise) | **PASS**: v ≤ v_max, a ≤ a_max, exact end points, triangular peak = √a | gtest `TrapezoidRespectsLimits…`, line E |
| Commanded velocity continuity | **PASS**: profile starts and ends at zero velocity per segment; drive interpolates with cubic Hermite | code review, `traj_test` |
| Pixel-to-table transform | **PASS** against hand-computed points | `vision_robustness.py` line 1 |
| Colour robustness | **PARTIAL**: no misses with brightness ×0.6 to ×0.8, colour cast, noise σ=12, 9×9 blur. Brightness ×1.3 loses orange (80 of 600 cubes); hue shift ±6 gives 2/14 errors, ±8 gives 8/14 | `vision_robustness.py` lines 2 and 5 |
| Overlapping or out-of-range cubes | **FAIL (guarded by scene rules)**: two same-colour cubes 35 mm apart merge into one detection between them; a cube cut by the image border is reported up to 16 mm off. The scene rules keep cubes 90 mm apart and inside the view, so this cannot happen through the dashboard | lines 3 and 4 |
| Reachability check of random positions | **PASS**: `pickReachable` uses the real planner; 300/300 random scenes plan | line D, `scene_test` |
| Plant model sanity | **PASS with stated assumptions**: torque limits 150/150/150/28/28/28 Nm and 3.14 rad/s match `joint_limits.yaml`; link masses from UR; rotor inertia, friction, brake values assumed; no joint coupling or Coriolis | code review |
| PID implementation | **PASS**: fixed `dt`, derivative on measurement with first-order filter, back-calculation anti-windup (kaw = ki/kp), output clamp, bumpless reset | `pid.cpp` review, gtests |
| Step metrics computed from data | **PASS**: 10-90 % rise, overshoot from peak, 2 % settling = last exit from the band, steady-state error = mean over the last 0.2 s | `drive_report.cpp:32` |
| Stability under the plotted gains, payload disturbance | **PASS (in simulation)**: cascade 0 % overshoot, settles 0.29 s; 3 kg step: peak 3.2 mrad, back below 0.1 mrad in 0.70 s; whole cycle with 3 kg: max following error 6.1 mrad | `drive_report` (numbers identical to the README) |
| Worst-case random scenes through the drive | **PASS**: the 5 fastest of 1000 random scenes (up to 2.97 rad/s) run with no fault, max following error 5.5 mrad | sandbox run (`audit_drive`, P7) |
| Timing | **PASS with note**: the drive runs a fixed 1 kHz on simulated time and extrapolates late targets; the sequencer's playback is tick-based, so a slow timer slows the motion instead of skipping | code review; untested on real ROS |
| Stale data between nodes | **FAIL**: see Phase 1 items 3 and 4 | integration test [B] |
| PLC: E-stop latch, reset only with the loop closed, start/stop self-hold, start interlock, drive alarm latch | **PASS** against the ladder spec (software PLC); on OpenPLC 24/28 before your ladder update, after-update result not shared | `plc_check.py`, `plc_link_test` 39/39 |
| PLC: regions written by the PLC and by ROS are disjoint | **PASS**: coils 0-11 vs 16-26; `plc_check.py` check 0 proves persistence on OpenPLC | code + your run |
| PLC: behaviour when ROS dies | **FAIL**: no watchdog, `run` stays on (Phase 1 item 4) | integration test [B] |
| E-stop during approach, grasp dwell, transit, release dwell | **PASS** for stop, torque off, refusal of start before reset, safe resume and correct final placement. **FAIL** for the gripper: it changes state during the stop ramp in the two dwell phases | integration test [A], 4 × 7 checks, 26/28 |
| Reset after E-stop, repeated cycles | **PASS**: 3 consecutive cycles; resume after each phase | [A], [C] |
| No cubes / duplicate container colours / unreachable cube | **PASS**: refused with a reason (scene needs 1 to 8 objects, unique container colours, plannable) | `scene_test` 68/68 |
| All cubes unmatched (gray only) | **PASS**: all four go to the gray container, three cycles in a row | [C] |

## Phase 3: repository quality

| Finding | Status |
|---|---|
| `__pycache__/*.pyc`, OpenPLC Editor `build/` output and an empty `sorting_cell_bridge` template package were committed | Moved to `_to_review/` (with `COLCON_IGNORE`), `.gitignore` extended (`678e637`). Delete after you check |
| `package.xml`: `TODO` descriptions and licences, `root@todo.todo`; `yaml-cpp`, `libmodbus-dev`, `pkg-config` and the launch-time packages not declared, so `rosdep` could not set up a clean machine | Fixed (`80856d2`); all keys verified to exist in rosdep |
| No LICENSE file although the packages declare MIT | Added (`80856d2`) |
| Python checks called `sys.exit` at import, which makes pytest abort with INTERNALERROR, so `colcon test` never ran them | Fixed (`7d5e5c1`): renamed to `check_*.py` with a pytest wrapper; a wrapper failing on a failing check was confirmed |
| Template ament lint tests in one package only, never passing | Moved to `_to_review`; flake8 for the whole repo in CI (`.flake8`, `629bbf0`) |
| C++ tests not registered with CTest; three of them could not fail | Fixed (`405b4f5`): gtest suite plus six check programs under `colcon test` |
| No CI | Added (`1eaa766`): `ros:jazzy`, rosdep, colcon build/test; flake8 job. **Has not run yet**; it runs on the first push |
| `~` not expanded in teach program paths (you hit this) | Fixed (`47b3c22`) with a test |
| Parameters buried in code (speeds, hover height, stop ramp, following-error limits, gains) | Open (P8) |
| `README` register map, `plc/register_map.md`, `plc_link.hpp` | Consistent with each other; the committed ladder is not (item 1) |
| `docs/architecture.svg` | Nodes, topics and coil ranges match the code and the launch file. "fail-safe: no PLC = E-stop" holds only while the bridge itself runs (item 3) |
| Dead code | `fk_demo` (11 lines) and `cartesian_move_demo` are milestone demos; fine to keep, could move to an `examples` folder |
| Magic numbers | Mostly named constants with comments. Exceptions: the sequencer's `kStopTime`/`kStartTime` duplicated in `drive_report.cpp`; the drive tunings in `ur5e_setup.hpp` |

## Commits on the `audit` branch so far

| Commit | Change |
|---|---|
| `678e637` | Generated files and unused package moved to `_to_review/` |
| `80856d2` | Package metadata, declared dependencies, LICENSE |
| `7d5e5c1` | Python checks run under `colcon test` |
| `405b4f5` | Real pass/fail tests, new gtest suite, CTest registration |
| `629bbf0` | Python style fixes, `.flake8` |
| `1eaa766` | GitHub Actions CI |
| `47b3c22` | `~` in teach program paths |
| `316abea` | `kinematics_report`, `validation/vision_robustness.py` |
| `197870e` | README overclaims corrected |
| (this file) | `AUDIT_REPORT.md` |

## Proposed changes awaiting approval

| # | Change | Why | Size |
|---|---|---|---|
| P1 | Stale-data watchdog in the sequencer and the drive: with `use_plc`, no PLC message for 0.5 s means E-stop; in drive mode, no `drive/ready` or `joint_states` for 0.2 s means "drive not ready" | Item 3 | Small (C++ only) |
| P2 | PLC communication watchdog: the bridge toggles a heartbeat coil (27) every cycle; a TON in the ladder drops `run` and latches a `comm_fault` if it stops for 0.5 s; reset needed. Register map v4, `plc_check.py` +4 checks | Item 4; the obvious interview question | Medium; **you edit the ladder** (2 variables, 3 rungs) |
| P3 | `at_home` from the measured joints (all within 0.01 rad of home, arm at rest) | Item 5 | Small |
| P4 | Gripper changes only while motion is permitted; in drive mode, grasp and release fault the sequencer if the actual tool is more than 10 mm from the cube or slot | Item 6 | Small |
| P5 | Enforce the joint speed limit in the planner: if a Cartesian segment needs more than 2.8 rad/s on any joint, re-time that segment slower | Phase 2 velocity item | Small |
| P6 | Vision: ignore contours touching the image border, and report a blob larger than 1.6 cube areas as an error instead of one wrong centroid | Phase 2 vision items | Small |
| P7 | Integration tests in the repo with real ROS (`launch_testing`): a full cycle, E-stop in each phase against a small software PLC over Modbus, process death (P1/P2), gray-only scene. Runs in CI | Makes the Phase 2 multi-node evidence reproducible | Medium |
| P8 | Move tunables into `sorting_cell_bringup/config/cell.yaml` (speeds, hover height, stop ramp, cube mass, following-error limits) | Phase 3 standard | Medium |
| P9 | Full README rewrite (Phase 4) with a `scripts/regenerate_results.sh` that rebuilds every figure and table | Phase 4 | Medium |
| P10 | **You:** export the current OpenPLC project into `plc/sorting_cell` and commit it; confirm `plc_check.py` 28/28 | Item 1 | 10 min |
| P11 | **You:** check `_to_review/` and approve deleting it | Rule: no deletions without approval | 2 min |

## Weakest parts an interviewer will probe

1. **"What happens if the PC crashes?"** Today: the PLC keeps `run` on, and with only the bridge
   dead the arm keeps moving. P1 and P2 fix this.
2. **"Your E-stop is a ROS topic?"** It is, in simulation. Be ready to explain the real version:
   a safety relay or safety PLC with dual-channel E-stop, STO wired to the drives, and the PLC and
   PC only told about it.
3. **"How do you know the grasp worked?"** You don't; the grasp is an attach by frame index
   (P4 adds the position check).
4. **"Your IK fails 24 % of the time from home."** It is a local solver; for the cell's tool-down
   poses it is 100 %, and a scene is planned completely before it is accepted. Know when you would
   use an analytic UR IK instead (eight closed-form solutions, branch choice).
5. **"Is the PID tuned on a real motor?"** No; it is tuned on the project's own plant model, with
   assumed friction and rotor inertia. That is software-in-the-loop, and the step response is a
   property of the model.

## Phase 5: videos

Keep the total under 4 minutes. Embed GIFs 1, 2 and 3 in the README (8 to 15 s each, 960 px wide,
under 5 MB). Link the full videos (YouTube unlisted or a GitHub release asset).

| # | File | Length | On screen | Steps | Caption | Proves |
|---|---|---|---|---|---|---|
| 1 | `01_full_cycle.mp4` (+ GIF) | 45-60 s | RViz (left) and `rqt_image_view /camera/image` (right); terminal with `/plc/state` | `run_cell.sh use_plc:=true`, panel reset, start; let all 4 cubes sort | "Vision → own IK planner → servo drive; the PLC gates the run" | Autonomous sort from detections, PLC gating |
| 2 | `02_estop_recovery.mp4` (+ GIF) | 30-40 s | RViz, `ros2 topic echo /plc/state`, dashboard drive panel | Press E-stop during transit with a cube in the gripper; show the arm stop and `E-STOP`; try start (refused); release, reset, start; cycle completes | "E-stop latched by the PLC, start refused until reset, resume from the same point" | E-stop path, reset-and-resume |
| 3 | `03_colour_rule.mp4` (+ GIF) | 25-35 s | Dashboard and RViz | Change the green container to purple in the dashboard; start; the green cube goes to gray, a purple cube to purple | "Own-colour container, otherwise the gray one" | Colour matching and gray bin |
| 4 | `04_placement_reach.mp4` | 30 s | Dashboard with the reachability overlay | Drag a cube into a red (unreachable) cell: refused with the reason; Random objects twice; start | "Every placement is planned with the own IK before it is accepted" | Click/drag/random placement, IK reachability |
| 5 | `05_teach_playback.mp4` | 40 s | RViz and the `teach_keyboard` terminal | Jog to two waypoints with `m`, close and open the gripper, write the program, `P` to play | "Jog, save waypoints, replay through the same planner" | Teach and playback |
| 6 | `06_servo_drive.mp4` | 30 s | Dashboard drive panel and the generated `docs/drive/step_response.png`, `disturbance.png` | Run `drive_report` + `plot_drive.py` in a terminal (sped up), show the plots; jam J2 from the dashboard: the following error bar rises, `DRIVE_ALARM` | "Cascaded P/PI on a simulated plant: step, payload, jam trip (software-in-the-loop)" | Motor control claims, labelled as SIL |
| 7 | `07_clean_build.mp4` | 20-30 s (sped up) | Terminal, then the GitHub Actions page | `git clone`, `rosdep install`, `colcon build`, `colcon test`; show the green CI run | "Builds and tests from a clean checkout" | Reproducibility |

## Resume bullets (draft, true for the code as it stands)

- Built a simulated robotic sorting cell in ROS 2 (C++ and Python): a UR5e picks coloured cubes found by an OpenCV camera node and sorts them by colour, supervised by a PLC.
- Wrote the robot kinematics from scratch in C++/Eigen: DH forward kinematics (matches the official UR5e model to 1e-10 m), geometric Jacobian, damped-least-squares IK and a Cartesian planner with trapezoidal velocity profiles.
- Programmed the cell sequence in ladder logic (OpenPLC): start/stop self-hold, latched E-stop with reset, start interlocks and state machine. Connected it to ROS 2 with a C++ Modbus TCP bridge (libmodbus).
- Implemented servo control as software-in-the-loop: a firmware-style C++ library (cascaded P/PI with feedforward, anti-windup, following-error trip, controlled stop) driving a simulated UR5e joint model. Characterised step response, payload rejection and fault behaviour.
- Added a browser dashboard for scene editing with IK reachability checks, a teach-and-playback mode, and a test suite (gtest, pytest) running in GitHub Actions.

Once P1 and P2 are in, the PLC bullet can add: "with communication watchdogs on both sides".

## What remains open

- P1 to P11 above.
- The CI workflow has not run yet: it runs on the first push of this branch.
- `fk_checker` live numbers (FK vs TF while moving) were not re-measured.
