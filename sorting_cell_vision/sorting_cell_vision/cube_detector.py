"""ROS-free synthetic camera + HSV cube detector (no ROS imports, so it is unit-testable)."""
import math
import cv2
import numpy as np

# Camera model: top-down, fixed. Image u -> base y, image v -> base -x.
IMG_W, IMG_H = 640, 480
PX_PER_M = 800.0
CENTER_X, CENTER_Y = -0.50, 0.0     # base-frame point at the image centre
CUBE_SIZE = 0.04                    # [m]
CUBE_Z = CUBE_SIZE / 2.0            # cube centre height above the z=0 table
MIN_AREA_PX = 200

BGR = {"red": (40, 40, 220), "green": (60, 190, 60),
       "blue": (220, 90, 40), "yellow": (40, 220, 230)}

# OpenCV HSV: H 0-179
HSV_RANGES = {
    "red": [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (179, 255, 255))],
    "yellow": [((20, 120, 70), (35, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],
    "blue": [((100, 120, 50), (130, 255, 255))],
}


def pixel_to_table(u, v):
    """Pixel centroid -> (x, y) in the robot base frame [m]."""
    return (CENTER_X - (v - IMG_H / 2.0) / PX_PER_M,
            (u - IMG_W / 2.0) / PX_PER_M)


def table_to_pixel(x, y):
    return ((y * PX_PER_M) + IMG_W / 2.0,
            (CENTER_X - x) * PX_PER_M + IMG_H / 2.0)


def render_scene(cubes, noise=0.0, seed=0):
    """cubes: list of (color, x, y, yaw_rad). Returns a BGR image."""
    img = np.full((IMG_H, IMG_W, 3), (95, 95, 95), np.uint8)
    half = CUBE_SIZE * PX_PER_M / 2.0
    for color, x, y, yaw in cubes:
        u, v = table_to_pixel(x, y)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half)):
            pts.append((u + c * dx - s * dy, v + s * dx + c * dy))
        cv2.fillPoly(img, [np.round(np.array(pts) * 16).astype(np.int32)], BGR[color],
                     lineType=cv2.LINE_AA, shift=4)
    if noise > 0:
        rng = np.random.default_rng(seed)
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


def detect_cubes(img):
    """Returns a list of dicts: color, x, y (base frame), u, v, yaw (rad, mod 90deg), area."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    found = []
    for color, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            m = cv2.moments(c)
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            (_, _), (_, _), ang = cv2.minAreaRect(c)
            x, y = pixel_to_table(u, v)
            # (du, dv) -> (dx, dy) = (-dv, du) is a pure 90 deg rotation, so image angle
            # equals base yaw up to a 90 deg offset; a cube is symmetric, so use mod 90 deg.
            yaw = math.radians(ang) % (math.pi / 2.0)
            found.append({"color": color, "x": x, "y": y, "u": u, "v": v,
                          "yaw": yaw, "area": area})
    found.sort(key=lambda d: (d["x"], d["y"]))
    return found
