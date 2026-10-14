# Operating guide

Detailed usage of the sorting cell: running it with and without the PLC, the scene dashboard,
scene files and edit commands, and teach mode. The overview, method and results are in the
[README](../README.md).

## Running the cell

### Requirements

- ROS 2 Jazzy with `ros-jazzy-ur-description`, RViz, `libmodbus-dev`, `python3-opencv`,
  `libyaml-cpp-dev` (teach mode), `python3-matplotlib` (plots), `python3-yaml`, `pymodbus` (PLC check script only).
- OpenPLC Runtime v4 ([autonomy-logic/openplc-runtime](https://github.com/autonomy-logic/openplc-runtime))
  and OpenPLC Editor v4 to compile and upload the ladder program. The Modbus slave plugin must be
  enabled on port 5020; [`plc/plc_setup.sh`](../plc/plc_setup.sh) does this after each upload.

### Build

```
cd ~/sorting_cell_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### Stand-alone (no PLC)

When ROS runs in a Docker container, start everything from the **host** with one command. It
launches the cell inside the container and opens the dashboard in the host's browser as soon as
it answers (it finds the right address whether the container uses host or bridge networking):

```
./scripts/run_cell.sh                     # extra launch arguments are passed on, e.g. use_plc:=true
```

`CONTAINER=name` and `PORT=8766` change the container and the dashboard port. Inside the container
(or on a native ROS install) the launch alone is:

```
ros2 launch sorting_cell_bringup sorting.launch.py
```

then open **http://localhost:8765**, arrange the scene and press **Start sort**
(see [Scene dashboard](#scene-dashboard)). `auto_start:=true` starts by itself after 5 s instead,
`dashboard:=false` leaves the dashboard out.

### With the PLC

1. Open the project in [`plc/sorting_cell`](../plc/sorting_cell) with the OpenPLC Editor, build and
   upload, then run `plc/plc_setup.sh`. The ladder spec and the register map are in
   [`plc/ladder_spec.md`](../plc/ladder_spec.md) and [`plc/register_map.md`](../plc/register_map.md).
2. Check the ladder on its own: `python3 plc/plc_check.py` (expects `28 passed, 0 failed`).
3. Launch the cell:
   ```
   ros2 launch sorting_cell_bringup sorting.launch.py use_plc:=true
   ```
4. Operate it from another shell:
   ```
   ros2 topic pub --once /panel/reset std_msgs/msg/Empty "{}"           # clear the power-up E-stop
   ros2 topic pub --once /panel/start std_msgs/msg/Empty "{}"
   ros2 topic pub --once /panel/stop  std_msgs/msg/Empty "{}"
   ros2 topic pub --once /panel/estop std_msgs/msg/Bool "{data: true}"  # press the E-stop
   ros2 topic pub --once /panel/estop std_msgs/msg/Bool "{data: false}" # release it (reset needed)
   ros2 topic echo /plc/state                # IDLE, RUN, PICK, PLACE, DONE, E-STOP, FAULT, DRIVE_ALARM
   ```
   After a finished cycle, press reset to re-home and scan again.
5. Servo fault demo: jam a joint mid-move (dashboard "Jam joint", or
   `ros2 topic pub --once /drive/inject std_msgs/msg/String "{data: 'jam 2'}"`). The drive trips on
   following error and switches the torque off, the PLC latches DRIVE_ALARM and drops run. Clear
   the jam (`'clear'`), press reset, then start: the cycle resumes.

### Tests and validation

`colcon test` runs every automated test (gtests, the check programs below and the Python checks);
the same runs in CI on each push (`.github/workflows/ci.yml`).

```
colcon test && colcon test-result --verbose
ros2 run sorting_cell_controller kinematics_report $(ros2 pkg prefix ur_description)/share/ur_description/config/ur5e/default_kinematics.yaml
python3 validation/vision_robustness.py
ros2 run sorting_cell_controller ik_test             # Jacobian, IK, singularity handling
ros2 run sorting_cell_controller traj_test           # velocity profile and Cartesian planner
ros2 run sorting_cell_controller sort_plan_test      # pick-and-place planning, 300 random scenes
ros2 run sorting_cell_controller plc_link_test       # bridge vs a software PLC (no PLC needed)
ros2 run sorting_cell_controller teach_test           # program file, waypoints, jogs and playback planning
ros2 run sorting_cell_controller fk_checker          # own FK vs TF (needs a running robot_state_publisher)
ros2 run sorting_cell_controller validation_report docs/validation
python3 validation/plot_validation.py docs/validation
ros2 run sorting_cell_controller scene_test           # scene rules, matching, gray container, edit commands, random placement
PYTHONPATH=sorting_cell_vision python3 sorting_cell_vision/test/check_detector.py
PYTHONPATH=sorting_cell_vision python3 sorting_cell_vision/test/check_scene_codec.py
PYTHONPATH=sorting_cell_teach python3 sorting_cell_teach/test/check_keymap.py
PYTHONPATH=sorting_cell_dashboard python3 sorting_cell_dashboard/test/check_core.py
colcon test --packages-select servo_core && colcon test-result --verbose    # servo gtests (from the workspace)
ros2 run sorting_cell_controller drive_report docs/drive              # servo evidence, then:
python3 validation/plot_drive.py docs/drive
```


## Scene dashboard

![Scene dashboard](dashboard.png)

`sorting.launch.py` also starts a small web dashboard (`sorting_cell_dashboard`, Python standard
library HTTP server, no extra packages). Open **http://localhost:8765**. It shows the table from
above with the robot base at the bottom, and every change goes through `scene_server`, so the same
rules and the same IK check apply as for a scene file.

- **Drag** a cube or a container to move it. While dragging, the outline turns green or red as the
  position is checked; on release the move is applied or refused with the reason.
- **Click placement:** select an item (on the table or in a list), then click a spot on the table.
- **Random placement:** Objects, Containers or Everything, each a new arrangement that passes every
  rule and plans with the own IK. Reset reloads the scene file the cell started with.
- **Colours:** a dropdown per object and per container; the arrow next to each object shows where
  it will go (its colour's container, or the gray one).
- **Add / remove** objects and containers; a new item is put at a free, reachable place.
- **IK reachability overlay:** red cells are places the arm cannot reach with the tool pointing down
  (home, hover, grasp height planned with the own IK, 25 mm grid). The whole table lies inside the
  reach; the overlay shows the limit around it and the dead zone next to the base.
- **Servo drive panel:** drive state, following error per joint against its limit, and fault
  injection (jam a joint, clear it, reset the drive) for the demo.
- **Start sort** sends `cell/start`. While the arm moves, editing is locked. With `use_plc:=true`
  the PLC start button decides and the dashboard's Start is disabled.
- Arrow keys nudge the selected item by 10 mm (Shift: 2 mm), Delete removes it. Save and Load
  write and read scene YAML files (paths are inside the ROS machine or container).

`scripts/run_cell.sh` picks the address for you. Without it, if the browser cannot reach the page
(another machine, or a container without host networking), launch with `dashboard_host:=0.0.0.0`
and open `http://<that machine's or container's IP>:8765`. The server only answers requests addressed to `localhost` or an
IP address, and only accepts JSON posts, so other web sites cannot drive it from your browser.

## Scene configuration, colour matching and the gray container

The scene is a YAML file: which cubes lie where, which coloured containers exist and where, and
where the gray container is. The cell reads it at start-up (`scene:=` launch argument), checks it
with the same geometry rules and the same IK/planner that run the arm, and refuses a scene that
cannot be sorted, with the reason.

```yaml
version: 1
name: mismatch
objects:                       # the cubes the camera sees
  - {color: red,    x: -0.40, y: -0.20, yaw: 0.0}
  - {color: yellow, x: -0.62, y:  0.25, yaw: 0.3}
  - {color: purple, x: -0.50, y:  0.00, yaw: 0.8}
containers:                    # one container per colour (140 mm square)
  - {color: red,   x: -0.30, y: -0.30}
  - {color: green, x: -0.30, y: -0.10}
gray_container: {x: -0.72, y: 0.0}   # takes every object with no container of its colour
```

**Matching rule.** An object goes to the container of its own colour. If there is none, it goes to
the gray container. Colours: `red`, `orange`, `yellow`, `green`, `cyan`, `blue`, `purple`
(objects and containers use the same list; gray is reserved for the catch-all container).
Several objects of one colour are fine, they take separate places inside the container (a 3 x 3
grid, 45 mm pitch, so each container holds nine).

| Rule | Limit |
|---|---|
| Objects | 1 to 8; x -0.77..-0.23, y -0.37..0.37 (inside the camera view); at least 0.09 m apart |
| Containers | at most one per colour, up to 7; x -0.73..-0.27, y -0.38..0.38 (fully on the table); no overlaps |
| Object next to a container | the cube must lie outside every container footprint |
| Reachability | the complete sort is planned with the own IK before a scene is accepted; the tool must reach every pick and every drop place |

Check a file before launching it:

```
ros2 run sorting_cell_controller scene_check my_scene.yaml
```

It prints each object with its destination container and the planned cycle time, or the first
problem found. Two examples are installed with the bringup package: `scene_default.yaml` (every
colour has a container) and `scene_mismatch.yaml` (yellow, purple and orange go to gray):

```
ros2 launch sorting_cell_bringup sorting.launch.py scene:=/path/to/my_scene.yaml
ros2 topic pub --once /cell/start std_msgs/msg/Empty "{}"                # begin (or repeat) the sort
```

### Changing the scene while the cell is running

`scene_server` owns the scene and publishes it (`scene/config`); the vision node renders it, the
detector finds the cubes in the image, and the sequencer plans from those detections. Every
message carries a revision number, so cubes of one scene are never planned against containers of
another. Edits are sent as text on `scene/command` and answered on `scene/reply`:

| Command | Effect |
|---|---|
| `move_object I X Y`, `move_container I X Y`, `move_gray X Y` | move one item (indices count from 0) |
| `yaw_object I RADIANS` | rotate an object |
| `color_object I COLOR`, `color_container I COLOR` | recolour (container colours must stay unique) |
| `add_object COLOR X Y [YAW]`, `add_container COLOR X Y`, `remove_object I`, `remove_container I` | change the number of items |
| `random objects`, `random containers`, `random all` | new random positions that pass every rule and the IK check |
| `check_object I X Y`, `check_container I X Y`, `check_gray X Y` | test a position without changing anything |
| `load PATH`, `save PATH`, `reset` | scene files (`~` works) |

```
ros2 topic pub --once /scene/command std_msgs/msg/String "{data: 'color_container 3 purple'}"
ros2 topic echo /scene/reply
```

Every edit is also logged by `scene_server` in the launch terminal, for example
`command 'color_container 3 purple': OK, container 3 is now purple (scene revision 2)` or
`command 'color_container 3 purple': REFUSED, there is no container 3 (valid: 0 to 2)`.

A refused command leaves the scene unchanged and says why. Edits are refused while the arm is
moving; after the cycle the changed scene is planned from scratch. With a PLC, a finished cycle
still waits for the PLC reset, then plans whatever scene is current.

## Teach and playback

A stand-alone mode for programming the arm by hand: move the tool in Cartesian steps, save named
waypoints, record a program of moves, gripper actions and dwells, and play it back or store it as
a YAML file. It uses the same FK, Jacobian, IK and trapezoidal planner as the sorting cell.

```
ros2 launch sorting_cell_bringup teach.launch.py         # shell 1: pendant + RViz
ros2 run sorting_cell_teach teach_keyboard               # shell 2: key-driven client (needs a real terminal)
```

Do not run it together with `sorting.launch.py` (both publish `joint_states`). Teach mode is not
gated by the PLC and has no E-stop.

### Keys

| Key | Action |
|---|---|
| `w` `s` | jog -x / +x in the base frame (`w` is toward the table) |
| `a` `d` | jog -y / +y |
| `r` `f` | jog up / down |
| `q` `e` | rotate the tool about the vertical axis, + / - |
| `+` `-` | step size 2, 5, 10, 20, 50, 100 mm (yaw step is 5 rad per metre of step, 0.05 rad at 10 mm) |
| `m` | save the current pose as a named waypoint and append a `move` step |
| `o` `c` | append a gripper open / close step |
| `t` `h` | append a dwell (asks for seconds) / a home step |
| `g` `H` | go to a waypoint / go home (not recorded) |
| `u` `N` | remove the last step / clear everything |
| `P` `space` `v` `x` | play, pause, resume, stop |
| `W` `L` | write the program to a YAML file / load one |
| `l` `p` `i` | list waypoints / list program steps / show the current pose |
| `?` `Q` | help / quit |

The same commands can be sent as text on `teach/command` (`jog dx dy dz rx ry rz`, `save NAME`,
`gripper open|close`, `dwell SECONDS`, `addhome`, `undo`, `goto NAME`, `home`, `delete NAME`,
`new`, `list`, `steps`, `status`, `write PATH`, `load PATH`, `play`, `pause`, `resume`, `stop`).
Replies appear on `teach/status`:

```
ros2 topic pub --once /teach/command std_msgs/msg/String "{data: 'jog -0.05 0 0 0 0 0'}"
ros2 topic echo /teach/status
```

### Program file

```yaml
version: 1
name: demo_teach
waypoints:
  a_hover:
    position: [-0.4, -0.2, 0.2]                 # metres, robot base frame
    orientation_xyzw: [0.7071, 0.7071, 0, 0]    # tool orientation as a quaternion
  a_grasp:
    position: [-0.4, -0.2, 0.02]
    orientation_xyzw: [0.7071, 0.7071, 0, 0]
steps:
  - move: a_hover
    speed: fast          # optional; the default is the slow transport speed
  - move: a_grasp
  - gripper: close
  - dwell: 0.5           # seconds
  - move: a_hover
  - home: true
```

Step types are `move` (a waypoint name, optional `speed: fast`), `gripper` (`open` or `close`),
`dwell` (seconds) and `home`. A file with an unknown waypoint, an unknown step type or a bad number
is rejected with a message and leaves the current program untouched. A program is compiled before
it plays: every move is solved with the IK from the end of the previous move, so an unreachable
waypoint is reported with its step number before the arm moves. An example is in
[`programs/demo_teach.yaml`](../sorting_cell_controller/programs/demo_teach.yaml); load it with `L`.

