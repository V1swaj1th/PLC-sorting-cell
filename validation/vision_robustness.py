"""Robustness of the HSV cube detector beyond the conditions it was tuned for.

Usage: python3 validation/vision_robustness.py

The synthetic camera and the detector share one camera model, so the pixel-to-table check here
compares against hand-computed points instead of the inverse function.
"""
import math
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'sorting_cell_vision'))
from sorting_cell_vision.cube_detector import (BGR, IMG_W, PX_PER_M, detect_cubes,  # noqa: E402
                                               pixel_to_table, render_scene)

rng = np.random.default_rng(3)
COLOURS = list(BGR)


def random_scene(n):
    out = []
    while len(out) < n:
        x, y = rng.uniform(-0.77, -0.23), rng.uniform(-0.37, 0.37)
        if all(math.hypot(x - a, y - b) >= 0.09 for _, a, b, _ in out):
            out.append((COLOURS[rng.integers(7)], x, y, rng.uniform(0, math.pi / 2)))
    return out


def score(objs, img):
    found = detect_cubes(img)
    missed = wrong = 0
    worst = 0.0
    for colour, x, y, _ in objs:
        best = min(found, key=lambda d: math.hypot(d['x'] - x, d['y'] - y), default=None)
        if best is None or math.hypot(best['x'] - x, best['y'] - y) > 0.01:
            missed += 1
            continue
        worst = max(worst, math.hypot(best['x'] - x, best['y'] - y))
        wrong += best['color'] != colour
    spurious = len(found) - (len(objs) - missed)
    return missed, wrong, spurious, worst


# 1. transform against hand-computed points: image centre is (-0.50, 0), 800 px per metre,
#    image u grows with base y, image v grows with -x
points = [((-0.5, 0.0), (320, 240)), ((-0.5, 0.1), (400, 240)), ((-0.6, 0.0), (320, 320)),
          ((-0.35, -0.2), (160, 120))]
err = max(abs(pixel_to_table(u, v)[0] - x) + abs(pixel_to_table(u, v)[1] - y) for (x, y), (u, v) in points)
print('1 pixel_to_table against hand-computed points: max error %.1e m' % err)

# 2. illumination and image quality the detector was not tuned for
conditions = [('brightness x0.6', lambda im: im * 0.6), ('brightness x0.8', lambda im: im * 0.8),
              ('brightness x1.3', lambda im: im * 1.3),
              ('warm colour cast', lambda im: im + np.array([-15, 0, 25])),
              ('noise sigma 12', lambda im: im + rng.normal(0, 12, im.shape)),
              ('blur 9x9', lambda im: cv2.GaussianBlur(im, (9, 9), 0).astype(float))]
for label, change in conditions:
    totals, worst, n = [0, 0, 0], 0.0, 0
    for _ in range(100):
        objs = random_scene(6)
        n += len(objs)
        img = render_scene(objs, noise=4, seed=int(rng.integers(1_000_000))).astype(float)
        img = np.clip(change(img), 0, 255).astype(np.uint8)
        m, w, s, wo = score(objs, img)
        totals = [totals[0] + m, totals[1] + w, totals[2] + s]
        worst = max(worst, wo)
    print('2 %-18s %d cubes: missed %d, wrong colour %d, spurious %d, worst position error %.2f mm'
          % (label, n, totals[0], totals[1], totals[2], worst * 1e3))
for colour in COLOURS:
    img = np.clip(render_scene([(colour, -0.5, 0, 0)], noise=4, seed=1).astype(float) * 1.3, 0, 255)
    if not detect_cubes(img.astype(np.uint8)):
        print('  brightness x1.3: %s is lost' % colour)

# 3. touching or overlapping cubes (the scene rules keep cube centres at least 90 mm apart)
for gap, same in [(0.045, True), (0.035, True), (0.045, False), (0.030, False)]:
    objs = [('red', -0.5, 0.0, 0.0), ('red' if same else 'orange', -0.5, gap, 0.0)]
    found = detect_cubes(render_scene(objs, noise=4, seed=1))
    print('3 centres %.0f mm apart, %s colour: %d detections %s'
          % (gap * 1e3, 'same' if same else 'different', len(found),
             [(d['color'], round(d['y'], 3)) for d in found]))

# 4. a cube cut by the image border (the scene rules keep cubes inside the view)
for y in (0.38, 0.40, 0.41):
    found = detect_cubes(render_scene([('blue', -0.5, y, 0.0)], noise=4, seed=2))
    print('4 cube at y = %.2f m (image border at %.2f m): detected at %s'
          % (y, (IMG_W / 2) / PX_PER_M, [round(d['y'], 4) for d in found]))

# 5. hue margin between neighbouring colour ranges
for shift in (2, 4, 6, 8):
    bad = 0
    for colour in COLOURS:
        for s in (-shift, shift):
            hsv = cv2.cvtColor(render_scene([(colour, -0.5, 0.0, 0.0)], noise=2, seed=3), cv2.COLOR_BGR2HSV)
            hsv = hsv.astype(int)
            hsv[..., 0] = (hsv[..., 0] + s) % 180
            found = detect_cubes(cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR))
            bad += not (len(found) == 1 and found[0]['color'] == colour)
    print('5 hue shifted by +-%d (OpenCV hue units, 0-179): %d of 14 cases misclassified or lost' % (shift, bad))
