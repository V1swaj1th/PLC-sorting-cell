#!/usr/bin/env python3
"""Plots the M6 validation CSVs written by `ros2 run sorting_cell_controller validation_report DIR`.

    python3 plot_validation.py DIR        # reads DIR/{manip,cycle,sweep}.csv, writes DIR/*.png
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"
W_THRESHOLD = 0.02  # IK damping starts below this manipulability (IkOptions::w_threshold)

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10,
    "axes.axisbelow": True, "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
})


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, key, cast=float):
    return [cast(r[key]) for r in rows]


def manipulability(d, rows):
    t, w, cube = col(rows, "t"), col(rows, "manipulability"), col(rows, "cube", int)
    fig, ax = plt.subplots(figsize=(9, 3.8))
    top = max(w) * 1.18
    for c in sorted(set(cube)):
        idx = [i for i, v in enumerate(cube) if v == c]
        if c % 2 == 1:
            ax.axvspan(t[idx[0]], t[idx[-1]], color=GRID, alpha=0.45, lw=0)
        ax.text((t[idx[0]] + t[idx[-1]]) / 2, top * 0.985, "cube %d" % (c + 1), ha="center",
                va="top", color=MUTED, fontsize=9)
    ax.plot(t, w, color=BLUE, lw=1.8)
    ax.axhline(W_THRESHOLD, color=ORANGE, lw=1.4, ls=(0, (5, 3)))
    ax.text(t[-1], W_THRESHOLD - 0.0035, "IK damping threshold (0.02)", ha="right", va="top",
            color=ORANGE, fontsize=9)
    k = min(range(len(w)), key=w.__getitem__)
    ax.plot([t[k]], [w[k]], "o", color=BLUE, ms=7, mec=SURFACE, mew=2)
    ax.annotate("min %.3f at %.1f s" % (w[k], t[k]), (t[k], w[k]), xytext=(8, 14),
                textcoords="offset points", color=INK, fontsize=9)
    ax.set_ylim(0, top)
    ax.set_xlim(0, t[-1])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("manipulability |det J|")
    ax.set_title("Manipulability dips to %.3f at the bins, just above the damping threshold" % min(w))
    fig.tight_layout()
    fig.savefig(d / "manipulability.png", dpi=160)
    plt.close(fig)


def cycle(d, rows):
    names = [r["color"] for r in rows]
    dur = col(rows, "duration_s")
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    bars = ax.bar(names, dur, color=BLUE, width=0.5)
    for b, v in zip(bars, dur):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.12, "%.1f s" % v, ha="center", color=INK, fontsize=9)
    ax.set_ylim(0, max(dur) * 1.2)
    ax.grid(axis="x", visible=False)
    ax.set_ylabel("cycle time [s]")
    ax.set_title("Cycle time per cube (mean %.1f s)" % (sum(dur) / len(dur)))
    fig.tight_layout()
    fig.savefig(d / "cycle_time.png", dpi=160)
    plt.close(fig)


def sweep(d, rows):
    v = col(rows, "v_transport")
    tot, qd = col(rows, "total_s"), col(rows, "max_joint_speed")
    ok = col(rows, "within_joint_limit", int)
    fig, (a, b) = plt.subplots(2, 1, figsize=(7.5, 5.4), sharex=True)
    a.plot(v, tot, color=BLUE, lw=1.8, marker="o", ms=6, mec=SURFACE, mew=1.5)
    a.set_ylabel("whole cycle [s]")
    a.set_title("Faster transport shortens the cycle with diminishing returns")
    b.plot(v, qd, color=BLUE, lw=1.8)
    for x, y, g in zip(v, qd, ok):
        b.plot([x], [y], "o", ms=6, mec=SURFACE, mew=1.5, color=BLUE if g else ORANGE)
    b.axhline(3.14, color=ORANGE, lw=1.4, ls=(0, (5, 3)))
    b.text(v[0], 3.14 + 0.1, "UR5e joint speed limit 3.14 rad/s", color=ORANGE, fontsize=9, va="bottom")
    b.set_ylim(0, 3.9)
    b.set_ylabel("peak joint speed [rad/s]")
    b.set_xlabel("transport speed limit [m/s]  (acceleration = 2 x speed)")
    fig.tight_layout()
    fig.savefig(d / "speed_sweep.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    d = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    manipulability(d, read(d / "manip.csv"))
    cycle(d, read(d / "cycle.csv"))
    sweep(d, read(d / "sweep.csv"))
    print("wrote manipulability.png, cycle_time.png, speed_sweep.png to", d)
