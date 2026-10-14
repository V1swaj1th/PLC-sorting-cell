"""Run: PYTHONPATH=sorting_cell_vision python3 sorting_cell_vision/test/check_scene_codec.py <scene_config.yaml>
The YAML is written by the C++ scene_test (so this checks the real C++ output)."""
import json
import sys

from sorting_cell_vision.cube_detector import detect_cubes, render_scene
from sorting_cell_vision.scene_codec import detections_message, parse_scene

passed = failed = 0


def check(what, ok):
    global passed, failed
    print(f'  [{"PASS" if ok else "FAIL"}] {what}')
    passed, failed = (passed + 1, failed) if ok else (passed, failed + 1)


text = open(sys.argv[1]).read() if len(sys.argv) > 1 else \
    'version: 1\nrevision: 3\nobjects:\n  - {color: red, x: -0.4, y: -0.2, yaw: 0}\n'
rev, objs = parse_scene(text)
check(f'scene parses: revision {rev}, {len(objs)} objects', isinstance(rev, int) and len(objs) >= 1)
dets = detect_cubes(render_scene(objs, noise=4.0, seed=rev))
check('the detector finds every object of the scene', sorted(d['color'] for d in dets) == sorted(o[0] for o in objs))
msg = json.loads(detections_message(rev, dets))
check('detections message carries the revision and all cubes', msg['rev'] == rev and len(msg['cubes']) == len(objs))
check('cube entries have color, x, y, yaw', all(set(c) == {'color', 'x', 'y', 'yaw'} for c in msg['cubes']))
for bad, why in [('not: [valid', 'broken YAML'), ('objects: 5\nrevision: 1', 'objects not a list'),
                 ('objects: []', 'no revision'), ('revision: 1\nobjects: [{color: red, x: 1}]', 'missing y'),
                 ('revision: 1\nobjects: [{color: red, x: .nan, y: 0}]', 'NaN')]:
    try:
        parse_scene(bad)
        check(f'rejects {why}', False)
    except ValueError:
        check(f'rejects {why}', True)
print(f'\n{passed} passed, {failed} failed')
sys.exit(0 if failed == 0 else 1)
