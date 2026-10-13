"""Key bindings for the teach pendant. Pure Python, no ROS, so it can be tested on its own.

Jog directions are in the robot base frame. The table is on the -x side of the base, so
"forward" (w) is -x. Rotation is about the vertical axis (tool yaw).
"""
from dataclasses import dataclass
from typing import Optional

LINEAR_STEPS = [0.002, 0.005, 0.01, 0.02, 0.05, 0.10]   # metres per key press
ANGULAR_STEP_RATIO = 5.0                                # rad per metre of linear step: 0.01 m -> 0.05 rad
DEFAULT_STEP_INDEX = 2

HELP = """\
Teach pendant keys
  jog (base frame)  w/s: -x/+x (w = toward the table)   a/d: -y/+y   r/f: up/down   q/e: yaw +/-
  step size         + / -  (currently shown in the status line)
  waypoints         m: save waypoint + append move   g: go to a waypoint   H: go home   x: stop motion
  program steps     o: gripper open   c: gripper close   t: dwell   h: home step   u: undo last step
  program           P: play   space: pause   v: resume   N: new (clear)   W: write file   L: load file
  info              l: waypoints   p: program steps   i: pose / status   ?: this help   Q: quit
"""


@dataclass
class Action:
    command: Optional[str] = None      # text to publish on teach/command
    prompt: Optional[str] = None       # ask the user for a value first
    template: Optional[str] = None     # command with {} for the prompted value
    step_index: Optional[int] = None   # new step-size index
    show_help: bool = False
    quit: bool = False
    note: Optional[str] = None         # local message for the user


def _jog(step_index, axis, sign):
    s = LINEAR_STEPS[step_index]
    d = [0.0] * 6
    if axis == 5:
        d[5] = sign * s * ANGULAR_STEP_RATIO
    else:
        d[axis] = sign * s
    return 'jog ' + ' '.join(f'{v:.6f}' for v in d)


def key_to_action(key: str, step_index: int = DEFAULT_STEP_INDEX) -> Optional[Action]:
    """Map one key press to an Action; None for keys that are not bound."""
    step_index = max(0, min(len(LINEAR_STEPS) - 1, step_index))
    jog = {'w': (0, -1), 's': (0, +1), 'a': (1, -1), 'd': (1, +1),
           'r': (2, +1), 'f': (2, -1), 'q': (5, +1), 'e': (5, -1)}
    if key in jog:
        return Action(command=_jog(step_index, *jog[key]))
    if key in ('+', '='):
        if step_index >= len(LINEAR_STEPS) - 1:
            return Action(step_index=step_index, note='step size is already at the maximum')
        return Action(step_index=step_index + 1)
    if key in ('-', '_'):
        if step_index <= 0:
            return Action(step_index=step_index, note='step size is already at the minimum')
        return Action(step_index=step_index - 1)
    simple = {'H': 'home', 'x': 'stop', 'o': 'gripper open', 'c': 'gripper close',
              'h': 'addhome', 'u': 'undo', 'P': 'play', ' ': 'pause', 'v': 'resume',
              'N': 'new', 'l': 'list', 'p': 'steps', 'i': 'status'}
    if key in simple:
        return Action(command=simple[key])
    prompts = {'m': ('Waypoint name: ', 'save {}'), 'g': ('Go to waypoint: ', 'goto {}'),
               't': ('Dwell seconds: ', 'dwell {}'), 'W': ('Write program to file: ', 'write {}'),
               'L': ('Load program from file: ', 'load {}')}
    if key in prompts:
        return Action(prompt=prompts[key][0], template=prompts[key][1])
    if key == '?':
        return Action(show_help=True)
    if key == 'Q':
        return Action(quit=True)
    return None


def build_command(template: str, value: str) -> Optional[str]:
    """Fill a prompt template. Returns None when the value is empty or has a line break."""
    value = value.strip()
    if not value or '\n' in value or '\r' in value:
        return None
    return template.format(value)
