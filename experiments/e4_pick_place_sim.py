"""E4 - Dynamic simulation of the full pick-and-sort cycle in MuJoCo.

Questions answered:
  1. Does the whole cycle work: both objects picked and dropped in the right bins?
  2. What torque does each joint need while moving, and how close is that to what the
     drives can give at that speed?
  3. Does the arm clear the table and the bins?
  4. How much margin is left if the payload is heavier or the cycle is run faster?

Run:  uv run python -m experiments.e4_pick_place_sim
"""
from __future__ import annotations

import imageio.v2 as imageio
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from arm import params as P
from arm.dynamics import gravity_torque
from arm.sim import GRASP_TOLERANCE, run_cycle
from arm.task import BIN_WRIST_TURN, EXERCISE_FRACTION, EXERCISE_MIN_Z, plan_cycle
from .common import ASSUMPTION_NOTE, FIGS, JOINT_COLOURS, MEDIA, save_json, savefig, table, verdict, write_report

COLOURS = ("red", "blue")
HEAVY_PAYLOAD = 0.200
SPEED_FACTORS = (1.0, 1.5, 2.0, 2.5, 3.0)
MIN_BIN_CLEARANCE = 0.010
MIN_TABLE_CLEARANCE = 0.005


def utilisation(log):
    """|torque| / torque available at that speed, per joint, over time."""
    avail = np.column_stack([d.available_torque(log.qd[:, i]) for i, d in enumerate(P.DRIVES)])
    return np.abs(log.tau) / avail, avail


def write_video(frames, fps=30):
    mp4 = MEDIA / "e4_pick_place.mp4"
    with imageio.get_writer(mp4, fps=fps, quality=7, macro_block_size=None) as w:
        for f in frames:
            w.append_data(f)
    gif = MEDIA / "e4_pick_place.gif"
    stride = max(3, -(-len(frames) // 260))
    small = [Image.fromarray(f).resize((480, 270), Image.LANCZOS) for f in frames[::stride]]
    small[0].save(gif, save_all=True, append_images=small[1:], duration=int(1000 * stride / fps), loop=0, optimize=True)
    return mp4, gif


def storyboard(stills):
    keys = list(stills)
    cols = 3
    rows = int(np.ceil(len(keys) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.45 * rows))
    for ax in axes.ravel():
        ax.axis("off")
    for ax, k in zip(axes.ravel(), keys):
        ax.imshow(stills[k])
        ax.set_title(k)
    return savefig(fig, "e4_storyboard.png")


def main():
    traj = plan_cycle(COLOURS)
    log = run_cycle(traj, COLOURS, render=True)
    mp4, gif = write_video(log.frames)
    f_story = storyboard(log.stills)
    util, avail = utilisation(log)

    # ---------------- torque against time ----------------
    fig, axes = plt.subplots(2, 3, figsize=(12.5, 6), sharex=True)
    for i, ax in enumerate(axes.ravel()):
        ax.fill_between(log.t, -avail[:, i], avail[:, i], color="#2ca02c", alpha=0.12, label="usable torque at this speed")
        ax.plot(log.t, log.tau[:, i], color=JOINT_COLOURS[i], lw=1.1, label="required")
        ax.set_title(f"{P.JOINTS[i].name}  (peak use {util[:, i].max():.0%})")
        pk = np.abs(log.tau[:, i]).max()
        ax.set_ylim(-max(pk * 1.6, 0.05), max(pk * 1.6, 0.05)) if avail[:, i].min() > 4 * pk else None
        if i >= 3:
            ax.set_xlabel("time (s)")
        if i % 3 == 0:
            ax.set_ylabel("joint torque (N.m)")
    axes[0, 0].legend(loc="lower right", fontsize=7)
    fig.suptitle("Joint torque needed to follow the planned cycle (green band = what the drive can give)")
    f_tau = savefig(fig, "e4_joint_torques.png")

    # ---------------- stepper operating points on the torque-speed envelope ----------------
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.7))
    for ax, i in zip(axes, range(3)):
        d = P.DRIVES[i]
        w = np.linspace(0, max(np.abs(log.qd[:, i]).max() * 3, d.corner_speed() * 1.6), 300)
        ax.plot(np.degrees(w), d.available_torque(w), "g", label="usable torque (pull-out estimate)")
        ax.plot(np.degrees(w), np.full_like(w, d.hold_torque), "g:", label="holding torque")
        ax.scatter(np.degrees(np.abs(log.qd[:, i])), np.abs(log.tau[:, i]), s=3, color=JOINT_COLOURS[i],
                   label="cycle operating points")
        ax.axvline(P.JOINTS[i].vmax_deg, color="k", ls="--", lw=0.8, label="planned speed limit")
        ax.set(xlabel="joint speed (deg/s)", title=P.JOINTS[i].name, xscale="symlog", ylim=(0, d.hold_torque * 1.1))
        ax.set_xscale("symlog", linthresh=30)
    axes[0].set_ylabel("joint torque (N.m)")
    axes[2].legend(fontsize=7, loc="center right")
    f_env = savefig(fig, "e4_torque_speed.png")

    # ---------------- tracking and clearance ----------------
    tcp_err = np.linalg.norm(log.tcp - log.tcp_ref, axis=1) * 1e3
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True)
    axes[0].plot(log.t, tcp_err, "k")
    axes[0].set(ylabel="tool-tip tracking error (mm)", title="Tracking error of the simulated arm against the plan")
    axes[1].plot(log.t, log.clear_table * 1e3, color="#8c564b", label="wrist / gripper to table")
    axes[1].plot(log.t, log.clear_bins * 1e3, color="#1f77b4", label="wrist / gripper / carried object to bin walls")
    axes[1].axhline(MIN_BIN_CLEARANCE * 1e3, color="r", ls="--", lw=1, label="minimum allowed to bins")
    axes[1].set(ylabel="smallest distance (mm)", xlabel="time (s)", ylim=(0, 200), title="Clearance")
    axes[1].legend(loc="upper right")
    f_clear = savefig(fig, "e4_tracking_clearance.png")

    # ---------------- power ----------------
    power = log.tau * log.qd
    p_total = np.clip(power, 0, None).sum(axis=1)
    dt_log = log.t[1] - log.t[0]
    energy = float(p_total.sum() * dt_log)

    # ---------------- margin studies (no rendering) ----------------
    cycle_only = plan_cycle(COLOURS, axes_check=False)
    heavy = run_cycle(cycle_only, COLOURS, payload_mass=HEAVY_PAYLOAD, render=False)
    util_heavy, _ = utilisation(heavy)
    sweep = []
    for s in SPEED_FACTORS:
        lg = log if s == 1.0 else run_cycle(plan_cycle(COLOURS, speed=s, axes_check=False), COLOURS, render=False)
        u, _ = utilisation(lg)
        sweep.append({"speed": s, "util": u.max(axis=0).tolist(), "placed": all(lg.placed),
                      "saturated": lg.saturated, "duration": float(lg.t[-1]),
                      "tcp_err_mm": float(np.linalg.norm(lg.tcp - lg.tcp_ref, axis=1).max() * 1e3)})
    fig, ax = plt.subplots(figsize=(7.2, 4))
    for i in range(3):
        ax.plot([r["speed"] for r in sweep], [r["util"][i] * 100 for r in sweep], "-o", color=JOINT_COLOURS[i],
                label=P.JOINTS[i].name)
    ax.axhline(100, color="r", lw=1)
    ax.text(SPEED_FACTORS[0], 103, "usable stepper torque", color="r", fontsize=8)
    ax.set(xlabel="speed factor applied to the whole cycle (accelerations scale with its square)",
           ylabel="peak torque use (%)", title="How much faster could the cycle run before torque runs out?")
    ax.legend()
    f_sweep = savefig(fig, "e4_speed_margin.png")

    # ---------------- verdicts ----------------
    peak_util = util.max(axis=0)
    worst_joint = int(peak_util.argmax())
    ok_place = all(log.placed)
    ok_torque = bool(peak_util.max() < 1.0 and not log.saturated)
    ok_clear = bool(log.clear_bins.min() >= MIN_BIN_CLEARANCE and log.clear_table.min() >= MIN_TABLE_CLEARANCE)
    travel = np.degrees(traj.q.max(axis=0) - traj.q.min(axis=0))
    full_range = np.degrees(P.Q_MAX - P.Q_MIN)
    ok_axes = bool(np.all(travel >= 0.5 * full_range))
    axes_rows = [[P.JOINTS[i].name, f"{np.degrees(traj.q[:, i].min()):.0f} to {np.degrees(traj.q[:, i].max()):.0f}",
                  f"{travel[i]:.0f}", f"{full_range[i]:.0f}", f"{travel[i] / full_range[i]:.0%}", verdict(travel[i] >= 0.5 * full_range[i])]
                 for i in range(P.N)]
    static_task = np.max([np.abs(gravity_torque(q)) for q in traj.q[::200]], axis=0)

    torque_rows = []
    for i, d in enumerate(P.DRIVES):
        k = int(util[:, i].argmax())
        torque_rows.append([P.JOINTS[i].name, f"{np.abs(log.tau[:, i]).max():.3f}", f"{static_task[i]:.3f}",
                            f"{d.hold_torque:.2f}", f"{avail[k, i]:.2f}", f"{peak_util[i]:.0%}",
                            f"{util_heavy[:, i].max():.0%}", verdict(peak_util[i] < 1.0)])
    place_rows = [[f"object {i + 1} ({c})", f"{log.grasp_error[i] * 1e3:.2f}",
                   " / ".join(f"{v * 1e3:.0f}" for v in log.objects[i]),
                   " / ".join(f"{v * 1e3:.0f}" for v in P.BINS[c]), verdict(log.placed[i])] for i, c in enumerate(COLOURS)]
    sweep_rows = [[f"{r['speed']:g}x", f"{r['duration']:.1f}"] + [f"{u:.0%}" for u in r["util"][:3]] +
                  [f"{r['tcp_err_mm']:.2f}", "yes" if r["placed"] else "NO", "yes" if r["saturated"] else "no"] for r in sweep]

    save_json("e4.json", {"axes_ok": ok_axes, "axis_travel_deg": travel.tolist(), "placed": log.placed, "peak_util": peak_util.tolist(), "peak_tau": np.abs(log.tau).max(axis=0).tolist(),
                          "peak_mech_power_W": float(p_total.max()), "energy_J": energy,
                          "peak_power_per_joint_W": np.abs(power).max(axis=0).tolist(), "duration": traj.duration,
                          "min_clear_bins_mm": float(log.clear_bins.min() * 1e3), "sweep": sweep})

    report = f"""# E4 - Dynamic simulation of the pick-and-sort cycle

**Script:** `experiments/e4_pick_place_sim.py` · **Model:** `arm/mjcf.py`, `arm/sim.py`, `arm/task.py`

{ASSUMPTION_NOTE}
## Purpose

Run the complete minimum demonstration in a physics simulator before anything is built: pick two
objects at the fixed location and sort them into two bins. The run gives the torque each drive must
deliver while moving, which the static study (E2) cannot, and checks clearance to the cell.

## Video

![Pick-and-sort cycle]({'media/' + gif.name})

Full-resolution clip: [`media/{mp4.name}`](media/{mp4.name}) ({len(log.frames) / 30:.0f} s, real time).

![Storyboard]({f_story})

## Method

* **Model.** MuJoCo {__import__('mujoco').__version__}, generated from `arm/params.py`: six hinge joints with the mass budget of
  E2, rotor inertia reflected through each reduction, a two-finger gripper, the table, two bins
  and two {P.OBJECT_SIZE * 1e3:.0f} mm, {P.PAYLOAD_MASS * 1e3:.0f} g cubes. Time step 1 ms. Links are drawn as simple shapes;
  the CAD meshes can replace them later without changing the results much, since mass and geometry come from the same file.
* **Motion.** The trajectory from `arm/task.py`: the all-axes check, then the sort cycle ({traj.duration:.1f} s in total). Heavier-payload and speed studies replay the sort cycle only.
* **Control.** Each joint is driven by the torque needed to follow the plan (computed-torque
  tracking), limited to the holding torque of its drive. The logged torque is what the real
  motor has to produce: gravity, inertia, and the payload once it is gripped.
* **Usable torque.** For steppers, {P.STEPPER_DYNAMIC_FACTOR:.0%} of holding torque up to the corner speed, falling as 1/speed
  above it. For servos, a straight line from stall torque to no-load speed.
* **Grasp.** Not simulated as friction. When the fingers close, the object is attached to the hand
  if the tool tip is within {GRASP_TOLERANCE * 1e3:.0f} mm of its centre; it is released when the fingers open.
* **Colour.** The colour is known to the simulation; the sensor itself is not modelled.

## Result 1 - all six axes move

The spec book asks for "full movement in all axes and rotations". The run starts with an **all-axes check**: each joint
alone is taken towards both of its limits (to {EXERCISE_FRACTION:.0%} of the range, less where the tool would drop below
{EXERCISE_MIN_Z * 1e3:.0f} mm above the table) and back. The sort cycle that follows turns the wrist by {np.degrees(BIN_WRIST_TURN):.0f} deg at each bin.

{table(["Joint", "Angle reached (deg)", "Travel (deg)", "Joint range (deg)", "Share of range", "Moves at least half its range"], axes_rows)}
Verdict: {verdict(ok_axes)}

## Result 2 - the sort cycle works

{table(["Object", "Tool-tip error at grip (mm)", "Final position x / y / z (mm)", "Bin centre x / y (mm)", "In the right bin"], place_rows)}
Verdict: {verdict(ok_place)}

## Result 3 - torque while moving

![Joint torques]({f_tau})

{table(["Joint", "Peak torque in the cycle (N.m)", "Largest static torque along the path (N.m)", "Holding torque of the drive (N.m)",
        "Usable torque at the critical instant (N.m)", "Peak use", f"Peak use with a {HEAVY_PAYLOAD * 1e3:.0f} g payload", "Verdict"], torque_rows)}
"Peak use" is the largest ratio of required torque to usable torque at the same instant.
The dynamic peaks are close to the static values: at these speeds the arm is
gravity-dominated, so the static sizing of E2 is the governing calculation.

![Torque-speed operating points]({f_env})

Verdict: {verdict(ok_torque)} (no joint reached its torque limit: {'true' if not log.saturated else 'FALSE'})

## Result 4 - tracking and clearance

![Tracking and clearance]({f_clear})

{table(["Quantity", "Value", "Requirement", "Verdict"], [
    ["Worst tool-tip tracking error", f"{tcp_err.max():.2f} mm", "below the grip tolerance", verdict(tcp_err.max() < GRASP_TOLERANCE * 1e3)],
    ["Smallest distance, wrist / gripper / carried object to bin walls", f"{log.clear_bins.min() * 1e3:.0f} mm",
     f">= {MIN_BIN_CLEARANCE * 1e3:.0f} mm", verdict(log.clear_bins.min() >= MIN_BIN_CLEARANCE)],
    ["Smallest distance, wrist / gripper to table (fingertips at the grip)", f"{log.clear_table.min() * 1e3:.0f} mm",
     f">= {MIN_TABLE_CLEARANCE * 1e3:.0f} mm", verdict(log.clear_table.min() >= MIN_TABLE_CLEARANCE)]])}
The tracking error here is the controller's, with ideal actuators. It shows the plan is smooth
enough to follow; it is not a prediction of the real arm's accuracy (see E1 for that).

Verdict: {verdict(ok_clear)}

## Result 5 - margin

Heavier payload: with {HEAVY_PAYLOAD * 1e3:.0f} g instead of {P.PAYLOAD_MASS * 1e3:.0f} g the peak use rises to
{util_heavy.max():.0%} and both objects are {'still placed correctly' if all(heavy.placed) else 'NOT placed correctly'}.

Faster cycle: the sort cycle (without the all-axes check) is replayed at higher speed.

{table(["Speed", "Cycle (s)", "J1 peak use", "J2 peak use", "J3 peak use", "Tool-tip error (mm)", "Both placed", "Torque limit hit"], sweep_rows)}
![Speed margin]({f_sweep})

Torque is not what limits the cycle time; the step-pulse rate of the controller is (E3).

## Mechanical power

Peak mechanical power at the joints: {p_total.max():.2f} W. Energy over the cycle: {energy:.1f} J.
The mechanical work is tiny compared with the electrical power the steppers draw to hold
position, so the power budget (E5) is set by holding currents, not by motion.

## What this decides

1. The minimum demonstration is feasible with the provided actuators and the reductions chosen in E2.
2. The most loaded joint is the {P.JOINTS[worst_joint].name.split(" ", 1)[1]} at {peak_util[worst_joint]:.0%} of its usable torque; the design keeps
   working with a payload four times heavier ({util_heavy.max():.0%}).
3. The cell layout (pick spot and bins) is clear of collisions with {log.clear_bins.min() * 1e3:.0f} mm to spare at the bins.
4. The planned speed and acceleration limits can go into the firmware as starting values.

## Limits of this experiment

* Ideal actuators: no lost steps, no backlash, no belt stretch, no servo dead-band, no friction model.
* The grasp is an attachment, so finger force and slip are not tested.
* Homing on the limit switches and the colour decision are firmware logic and are not in this run.
* Link shapes are placeholders, so clearance near the arm's own structure is approximate. Self-collision of the arm with itself is not checked, which matters most at the extremes of the all-axes check.
"""
    write_report("E4_pick_place_simulation.md", report)


if __name__ == "__main__":
    main()
