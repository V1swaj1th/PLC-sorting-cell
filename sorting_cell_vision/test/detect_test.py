import math
import numpy as np
from sorting_cell_vision.cube_detector import *

rng = np.random.default_rng(1)
colors = list(BGR)
worst = 0.0
misses = 0
trials = 300
for t in range(trials):
    n = int(rng.integers(3, 5))
    cols = list(rng.permutation(colors)[:n])
    cubes = []
    while len(cubes) < n:
        x, y = rng.uniform(-0.65, -0.35), rng.uniform(-0.30, 0.30)
        if all(math.hypot(x - c[1], y - c[2]) > 0.09 for c in cubes):
            cubes.append((cols[len(cubes)], x, y, rng.uniform(0, math.pi / 2)))
    det = detect_cubes(render_scene(cubes, noise=6.0, seed=t))
    got = {d["color"]: d for d in det}
    if len(det) != n or set(got) != set(c[0] for c in cubes):
        misses += 1
        continue
    for col, x, y, yaw in cubes:
        worst = max(worst, math.hypot(got[col]["x"] - x, got[col]["y"] - y))
print(f"trials={trials} scene failures={misses} worst position error={worst*1000:.2f} mm")
# yaw check (single cube)
for yaw in (0.0, 0.3, 0.7, 1.2):
    d = detect_cubes(render_scene([("red", -0.5, 0.1, yaw)]))[0]
    err = abs((d["yaw"] - yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4)
    print(f"yaw true {yaw:.2f} detected {d['yaw']:.2f} err {math.degrees(err):.1f} deg")
