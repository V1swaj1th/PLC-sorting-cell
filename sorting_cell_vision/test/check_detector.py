"""Run: PYTHONPATH=sorting_cell_vision python3 sorting_cell_vision/test/check_detector.py"""
import math
import sys

import numpy as np

from sorting_cell_vision.cube_detector import BGR, detect_cubes, render_scene

rng = np.random.default_rng(1)
colors = list(BGR)
worst = 0.0
worst_yaw = 0.0
yaw_errs = []
misses = 0
trials = 400
for t in range(trials):
    n = int(rng.integers(3, 9))                      # 3 to 8 cubes, colours may repeat
    cubes = []
    while len(cubes) < n:
        x, y = rng.uniform(-0.77, -0.23), rng.uniform(-0.37, 0.37)
        if all(math.hypot(x - c[1], y - c[2]) >= 0.09 for c in cubes):
            cubes.append((colors[int(rng.integers(len(colors)))], x, y, rng.uniform(0, math.pi / 2)))
    det = detect_cubes(render_scene(cubes, noise=6.0, seed=t))
    ok = len(det) == n and sorted(d["color"] for d in det) == sorted(c[0] for c in cubes)
    if ok:
        for c in cubes:
            near = min((d for d in det if d["color"] == c[0]), key=lambda d: math.hypot(d["x"] - c[1], d["y"] - c[2]))
            worst = max(worst, math.hypot(near["x"] - c[1], near["y"] - c[2]))
            dy = abs((near["yaw"] - c[3] + math.pi / 4) % (math.pi / 2) - math.pi / 4)
            worst_yaw = max(worst_yaw, dy)
            yaw_errs.append(dy)
    else:
        misses += 1
        if misses <= 3:
            print("  miss in trial", t, "expected", sorted(c[0] for c in cubes), "got", sorted(d["color"] for d in det))

# each colour alone, on its own
single = 0
for name in colors:
    d = detect_cubes(render_scene([(name, -0.5, 0.0, 0.3)], noise=6.0, seed=7))
    single += (len(d) == 1 and d[0]["color"] == name)

print(f"{trials} random scenes (3-8 cubes, 7 colours, noise): {misses} misses, worst position error "
      f"{worst * 1000:.2f} mm, yaw error mean {math.degrees(np.mean(yaw_errs)):.2f} deg / worst {math.degrees(worst_yaw):.2f} deg; single cubes: {single}/{len(colors)}")
ok = misses == 0 and worst < 0.002 and worst_yaw < math.radians(5) and single == len(colors)
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
