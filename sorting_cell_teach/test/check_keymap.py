"""Run: PYTHONPATH=sorting_cell_teach python3 sorting_cell_teach/test/check_keymap.py"""
import sys

from sorting_cell_teach.keymap import LINEAR_STEPS, build_command, key_to_action

passed = failed = 0


def check(what, ok):
    global passed, failed
    print(f'  [{"PASS" if ok else "FAIL"}] {what}')
    if ok:
        passed += 1
    else:
        failed += 1


def vec(cmd):
    t = cmd.split()
    return t[0], [float(v) for v in t[1:]]


i = 2
s = LINEAR_STEPS[i]
for key, axis, sign in [('w', 0, -1), ('s', 0, 1), ('a', 1, -1), ('d', 1, 1),
                        ('r', 2, 1), ('f', 2, -1), ('q', 5, 1), ('e', 5, -1)]:
    name, v = vec(key_to_action(key, i).command)
    nonzero = [k for k, x in enumerate(v) if abs(x) > 0]
    check(f'{key} moves axis {axis} by sign {sign}',
          name == 'jog' and len(v) == 6 and nonzero == [axis] and v[axis] * sign > 0)
_, v = vec(key_to_action('w', i).command)
check('linear step equals the selected size', abs(abs(v[0]) - s) < 1e-9)
_, v = vec(key_to_action('q', i).command)
check('yaw step is 5 rad per metre of linear step', abs(v[5] - s * 5.0) < 1e-9)
check('step size up', key_to_action('+', 2).step_index == 3)
check('step size down', key_to_action('-', 2).step_index == 1)
top = key_to_action('+', len(LINEAR_STEPS) - 1)
check('step size stops at the maximum', top.step_index == len(LINEAR_STEPS) - 1 and top.note)
check('step size stops at the minimum', key_to_action('-', 0).step_index == 0 and key_to_action('-', 0).note)
check('out-of-range step index is clamped', abs(vec(key_to_action('w', 99).command)[1][0]) == LINEAR_STEPS[-1])
for key, cmd in [('H', 'home'), ('x', 'stop'), ('o', 'gripper open'), ('c', 'gripper close'),
                 ('h', 'addhome'), ('u', 'undo'), ('P', 'play'), (' ', 'pause'), ('v', 'resume'),
                 ('N', 'new'), ('l', 'list'), ('p', 'steps'), ('i', 'status')]:
    check(f'{key!r} sends "{cmd}"', key_to_action(key, i).command == cmd)
for key, tpl in [('m', 'save {}'), ('g', 'goto {}'), ('t', 'dwell {}'), ('W', 'write {}'), ('L', 'load {}')]:
    a = key_to_action(key, i)
    check(f'{key!r} prompts and fills "{tpl}"', a.prompt and a.template == tpl and a.command is None)
check('? shows help', key_to_action('?', i).show_help)
check('Q quits', key_to_action('Q', i).quit)
check('unbound key gives None', key_to_action('z', i) is None and key_to_action('\x1b', i) is None)
check('prompt value is filled in', build_command('save {}', ' pick_a \n') == 'save pick_a')
check('empty prompt value is cancelled', build_command('save {}', '   ') is None)
check('value with a line break is rejected', build_command('save {}', 'a\nb') is None)

print(f'\n{passed} passed, {failed} failed')
sys.exit(0 if failed == 0 else 1)
