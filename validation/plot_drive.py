#!/usr/bin/env python3
"""Plots the servo evidence written by `ros2 run sorting_cell_controller drive_report DIR`.

    python3 plot_drive.py DIR        # reads DIR/*.csv and DIR/metrics.yaml, writes DIR/*.png
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

SURFACE, INK, MUTED, GRID, BAND = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df", "#efeee9"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]  # fixed order
JOINTS = ["J1 base", "J2 shoulder", "J3 elbow", "J4 wrist 1", "J5 wrist 2", "J6 wrist 3"]

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "lines.linewidth": 2,
    "axes.axisbelow": True, "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False,
})


def read(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def step(d, m):
    s = read(d / "step.csv")
    names = [("p_only", "P only"), ("pid", "PID"), ("cascade", "Cascade P/PI"),
             ("cascade_gravity_ff", "Cascade + gravity FF")]
    fig, ax = plt.subplots(figsize=(9, 4.4))
    ax.axhspan(0.98, 1.02, color=BAND, lw=0, label="±2 % band")
    for i, (k, lab) in enumerate(names):
        mm = m["step"][k]
        settle = f'{mm["settle_s"]:.2f} s' if mm["settle_s"] >= 0 else "not within 2.5 s"
        ax.plot(s["t"], [v / 0.2 for v in s[k]], color=SERIES[i], zorder=3 + i,
                label=f'{lab}: overshoot {mm["overshoot_pct"]:.1f} %, settles {settle}, '
                      f'error {mm["ss_error_rad"] * 1e3:.3f} mrad')
    ax.set_xlim(0, 1.5)
    ax.set_ylim(0, 1.25)
    ax.set_xlabel("time after the step [s]")
    ax.set_ylabel("shoulder position / step")
    ax.set_title("0.2 rad shoulder step: P only vs PID vs cascade")
    ax.legend(loc="lower right", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(d / "step_response.png", dpi=150)
    plt.close(fig)


def antiwindup(d, m):
    a = read(d / "antiwindup.csv")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    for i, (k, lab) in enumerate([("with", "anti-windup on"), ("without", "anti-windup off")]):
        ax1.plot(a["t"], a[k], color=SERIES[i], label=f'{lab}: overshoot {m["antiwindup"][k]["overshoot_pct"]:.0f} %')
        ax2.plot(a["t"], a["tau_" + k], color=SERIES[i])
    ax1.axhline(1.0, color=MUTED, lw=1, ls="--")
    ax1.set_ylabel("shoulder move [rad]")
    ax1.set_title("1.0 rad step with the PID saturated at 150 Nm")
    ax1.legend(loc="lower right")
    ax2.set_ylabel("torque command [Nm]")
    ax2.set_xlabel("time after the step [s]")
    ax2.set_xlim(0, 3)
    fig.tight_layout()
    fig.savefig(d / "antiwindup.png", dpi=150)
    plt.close(fig)


def tracking(d, m):
    light, heavy = read(d / "tracking.csv"), read(d / "tracking_3kg.csv")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, data, title in [(ax1, light, "0.5 kg cube"), (ax2, heavy, "3 kg payload")]:
        t = data["t"]
        carry = [p > 0 for p in data["payload"]]
        start = None
        for i, c in enumerate(carry + [False]):
            if c and start is None:
                start = t[i]
            if not c and start is not None:
                ax.axvspan(start, t[i - 1], color=BAND, lw=0)
                start = None
        for j in range(6):
            ax.plot(t, [v * 1e3 for v in data[f"fe{j}"]], color=SERIES[j], lw=1.4, label=JOINTS[j])
        ax.set_ylabel("following error [mrad]")
        ax.set_title(f"Whole sort cycle, {title} (shaded: payload carried)")
    lim = 30
    ax1.legend(ncol=6, loc="upper center", bbox_to_anchor=(0.5, -0.02), fontsize=8.5)
    ax2.set_xlabel(f"time [s]   (fault limit {lim} mrad on J1-J3, 50 mrad on J4-J6: far outside these axes)")
    fig.tight_layout()
    fig.savefig(d / "tracking.png", dpi=150)
    plt.close(fig)


def disturbance(d, m):
    s = read(d / "disturbance.csv")
    t = [v - 0.3 for v in s["t"]]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    for i, (k, lab) in enumerate([("1", "J2 shoulder"), ("2", "J3 elbow"), ("3", "J4 wrist 1")]):
        ax1.plot(t, [v * 1e3 for v in s["fe" + k]], color=SERIES[i + 1], label=lab)
        ax2.plot(t, s["tau" + k], color=SERIES[i + 1])
    for ax in (ax1, ax2):
        ax.axvline(0, color=MUTED, lw=1, ls="--")
    dm = m["disturbance_hold"]
    ax1.set_title(f"3 kg payload added while holding: peak {dm['peak_fe_mrad']:.1f} mrad, "
                  f"back under 0.1 mrad in {dm['recover_s']:.2f} s")
    ax1.set_ylabel("following error [mrad]")
    ax1.legend(loc="lower right")
    ax2.set_ylabel("torque [Nm]")
    ax2.set_xlabel("time after the payload is added [s]")
    ax2.set_xlim(-0.2, 1.5)
    fig.tight_layout()
    fig.savefig(d / "disturbance.png", dpi=150)
    plt.close(fig)


def estop(d, m):
    s = read(d / "estop.csv")
    em = m["estop"]
    t0 = em["t_estop_s"]
    t = [v - t0 for v in s["t"]]
    keep = [i for i, v in enumerate(t) if -0.6 <= v <= 1.0]
    tt = [t[i] for i in keep]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.4), sharex=True)
    moving = sorted(range(6), key=lambda j: -max(abs(s[f"v{j}"][i]) for i in keep))[:3]
    for n, j in enumerate(sorted(moving)):
        ax1.plot(tt, [s[f"v{j}"][i] for i in keep], color=SERIES[j], label=JOINTS[j])
        ax2.plot(tt, [s[f"tau{j}"][i] for i in keep], color=SERIES[j])
    sto = em["sto_after_s"]
    for ax in (ax1, ax2):
        ax.axvline(0, color=MUTED, lw=1, ls="--")
        ax.axvline(sto, color=MUTED, lw=1, ls=":")
    ax1.annotate("E-stop", (0, ax1.get_ylim()[1]), xytext=(4, -12), textcoords="offset points", color=MUTED, fontsize=9)
    ax1.annotate("STO + brakes", (sto, ax1.get_ylim()[1]), xytext=(4, -12), textcoords="offset points", color=MUTED, fontsize=9)
    ax1.set_title(f"E-stop at full speed: path stop, standstill after {em['standstill_after_s']:.2f} s, "
                  f"torque off after {sto:.2f} s")
    ax1.set_ylabel("joint velocity [rad/s]")
    ax1.legend(loc="upper right")
    ax2.set_ylabel("torque [Nm]")
    ax2.set_xlabel("time after the E-stop [s]")
    fig.tight_layout()
    fig.savefig(d / "estop.png", dpi=150)
    plt.close(fig)


def jam(d, m):
    s = read(d / "jam.csv")
    jm = m["jam"]
    j = jm["jammed_joint"] - 1
    t0 = jm["t_jam_s"]
    t = [v - t0 for v in s["t"]]
    keep = [i for i, v in enumerate(t) if -0.2 <= v <= jm["fault_after_s"] + 0.001]  # torque is off after the trip
    tt = [t[i] for i in keep]
    lim = 30 if j < 3 else 50
    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.plot(tt, [abs(s[f"fe{j}"][i]) * 1e3 for i in keep], color=SERIES[j], label=f"|following error| {JOINTS[j]}")
    ax.axhline(lim, color=MUTED, lw=1, ls="--")
    ax.annotate(f"fault limit {lim} mrad", (tt[0], lim), xytext=(4, 4), textcoords="offset points", color=MUTED, fontsize=9)
    ax.axvline(0, color=MUTED, lw=1, ls=":")
    ax.axvline(jm["fault_after_s"], color=MUTED, lw=1, ls=":")
    ax.annotate("jam", (0, 0), xytext=(4, 4), textcoords="offset points", color=MUTED, fontsize=9)
    ax.annotate("fault: STO", (jm["fault_after_s"], 0), xytext=(4, 4), textcoords="offset points", color=MUTED, fontsize=9)
    ax.set_title(f"{JOINTS[j]} jammed mid-move: fault after {jm['fault_after_s'] * 1e3:.0f} ms (incl. 20 ms debounce)")
    ax.set_ylabel("mrad")
    ax.set_xlim(-0.2, 0.3)
    ax.set_xlabel("time after the jam [s]   (after the trip the torque is off and the brakes hold)")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(d / "jam.png", dpi=150)
    plt.close(fig)


def main():
    d = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    m = yaml.safe_load(open(d / "metrics.yaml"))
    for f in (step, antiwindup, tracking, disturbance, estop, jam):
        f(d, m)
    print(f"wrote {d}/{{step_response,antiwindup,tracking,disturbance,estop,jam}}.png")


if __name__ == "__main__":
    main()
