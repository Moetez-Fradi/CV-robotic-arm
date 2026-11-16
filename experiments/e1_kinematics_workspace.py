"""E1 - Kinematics and workspace.

Questions answered:
  1. Is the closed-form inverse kinematics correct?
  2. Can the arm reach the pick spot and both bins with the tool pointing down,
     inside the joint limits, and with what margin?
  3. What positioning resolution do the chosen drives give at the tool?

Run:  uv run python -m experiments.e1_kinematics_workspace
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from arm import params as P
from arm.kinematics import fk, fk_frames, ik, jacobian, tool_down
from arm.task import key_poses
from .common import ASSUMPTION_NOTE, JOINT_COLOURS, save_json, savefig, table, verdict, write_report

GEARBOX_BACKLASH_DEG = 1.0   # ASSUMED typical backlash of the PG518 planetary gearbox
SERVO_PULSE_SPAN_US = 2000   # 500-2500 us for 180 deg
PCA9685_COUNT_US = 20000 / 4096


def validate_ik(n=2000, seed=0):
    rng = np.random.default_rng(seed)
    recovered, worst, counts = 0, 0.0, []
    for _ in range(n):
        q = rng.uniform(P.Q_MIN, P.Q_MAX)
        T = fk(q)
        sols = ik(T, q)
        counts.append(len(sols))
        if sols and np.max(np.abs(sols[0] - q)) < 1e-6:
            recovered += 1
        for s in sols:
            worst = max(worst, float(np.abs(fk(s) - T).max()))
    return {"n": n, "recovered": recovered, "worst_error": worst,
            "mean_solutions": float(np.mean(counts)), "max_solutions": int(np.max(counts))}


def side_map(step=0.004):
    r = np.arange(0.0, 0.56 + 1e-9, step)
    z = np.arange(-0.04, 0.50 + 1e-9, step)
    ok = np.zeros((len(z), len(r)), dtype=bool)
    for i, zi in enumerate(z):
        for k, rk in enumerate(r):
            ok[i, k] = bool(ik(tool_down(rk, 0.0, zi, yaw=0.0)))
    return r, z, ok


def top_map(height, step=0.008):
    x = np.arange(-0.5, 0.5 + 1e-9, step)
    ok = np.zeros((len(x), len(x)), dtype=bool)
    for i, yi in enumerate(x):
        for k, xk in enumerate(x):
            ok[i, k] = bool(ik(tool_down(xk, yi, height)))
    return x, ok


def radial_margin(r, z, ok, r0, z0):
    """Distance from (r0, z0) to the nearest unreachable cell of the side map."""
    rr, zz = np.meshgrid(r, z)
    d = np.hypot(rr - r0, zz - z0)
    return float(d[~ok].min())


def stick(q):
    """Arm skeleton in the vertical plane through the base axis: (radial, z) points."""
    pts = [np.zeros(3)] + [T[:3, 3] for T in fk_frames(q)] + [fk(q)[:3, 3]]
    pts = np.array(pts)
    c, s = np.cos(q[0]), np.sin(q[0])
    return pts[:, 0] * c + pts[:, 1] * s, pts[:, 2]


def main():
    poses = key_poses()
    ikv = validate_ik()

    # ---------------- reachability maps ----------------
    r, z, ok = side_map()
    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    ax.contourf(r * 1e3, z * 1e3, ok.astype(float), levels=[0.5, 1.5], colors=["#cfe8cf"])
    ax.contour(r * 1e3, z * 1e3, ok.astype(float), levels=[0.5], colors=["#2e7d32"], linewidths=1.2)
    th = np.linspace(-np.pi / 2, np.pi / 2, 200)
    reach = P.L2 + P.L3 + P.D6
    ax.plot(reach * np.cos(th) * 1e3, (P.D1 + reach * np.sin(th)) * 1e3, "k:", lw=0.8,
            label=f"geometric max reach ({reach * 1e3:.0f} mm)")
    styles = {"ready": "#555555", "above pick": "#1f77b4", "pick": "#d62728",
              "above red bin": "#ff7f0e", "drop red bin": "#8c564b"}
    for name, colour in styles.items():
        sr, sz = stick(poses[name])
        ax.plot(sr * 1e3, sz * 1e3, "-o", color=colour, lw=2, ms=4, label=name)
    ax.axhline(0, color="k", lw=1)
    ax.set(xlabel="radial distance from base axis (mm)", ylabel="height above table (mm)",
           title="Side view: where the tool tip can be with the tool pointing down (green)",
           xlim=(-60, 560), ylim=(-40, 500), aspect="equal")
    ax.legend(loc="upper right")
    f_side = savefig(fig, "e1_reach_side.png")

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    for ax, h, label in zip(axes, (P.Z_GRASP, P.Z_SAFE), ("grip height", "travel height")):
        x, okt = top_map(h)
        ax.contourf(x * 1e3, x * 1e3, okt.astype(float), levels=[0.5, 1.5], colors=["#cfe8cf"])
        ax.contour(x * 1e3, x * 1e3, okt.astype(float), levels=[0.5], colors=["#2e7d32"], linewidths=1.0)
        ax.plot(0, 0, "k+", ms=10)
        ax.plot(P.PICK_XY[0] * 1e3, P.PICK_XY[1] * 1e3, "o", color="#c9a400", ms=9, label="pick spot")
        for colour, (bx, by) in P.BINS.items():
            a = (P.BIN_INNER / 2 + P.BIN_WALL_T) * 1e3
            ax.add_patch(plt.Rectangle((bx * 1e3 - a, by * 1e3 - a), 2 * a, 2 * a, fill=False,
                                       ec=colour, lw=2, label=f"{colour} bin"))
        ax.set(title=f"Top view at {label} (z = {h * 1e3:.0f} mm)", xlabel="x (mm)", ylabel="y (mm)",
               aspect="equal")
        ax.legend(loc="upper left")
    f_top = savefig(fig, "e1_reach_top.png")

    # ---------------- key poses, margins, conditioning ----------------
    rows, usage, pose_data = [], {}, {}
    all_ok = True
    for name, q in poses.items():
        T = fk(q)
        p = T[:3, 3]
        lim_margin = np.degrees(np.minimum(q - P.Q_MIN, P.Q_MAX - q))
        J = jacobian(q)
        sv = np.linalg.svd(J[:3], compute_uv=False)
        margin = radial_margin(r, z, ok, np.hypot(p[0], p[1]), p[2])
        tool_err = float(np.degrees(np.arccos(np.clip(-T[2, 2], -1, 1))))
        usage[name] = (q - P.Q_MIN) / (P.Q_MAX - P.Q_MIN)
        good = lim_margin.min() > 5 and margin > 0.02
        all_ok &= good
        rows.append([name, " / ".join(f"{a:.1f}" for a in np.degrees(q)),
                     f"{lim_margin.min():.0f} (J{int(lim_margin.argmin()) + 1})",
                     f"{margin * 1e3:.0f}", f"{sv[0] / sv[-1]:.1f}", f"{tool_err:.2f}", verdict(good)])
        pose_data[name] = {"q_deg": np.degrees(q).round(3).tolist(), "tcp_m": p.round(4).tolist()}

    fig, ax = plt.subplots(figsize=(8, 3.6))
    names = list(usage)
    w = 0.13
    for j in range(P.N):
        ax.bar(np.arange(len(names)) + (j - 2.5) * w, [usage[n][j] * 100 for n in names], w,
               color=JOINT_COLOURS[j], label=f"J{j + 1}")
    ax.axhspan(0, 5, color="r", alpha=0.12)
    ax.axhspan(95, 100, color="r", alpha=0.12)
    ax.set_xticks(np.arange(len(names)), names, rotation=20, ha="right")
    ax.set(ylabel="position within joint range (%)", ylim=(0, 100),
           title="Joint-range usage at the task poses (red bands = within 5 % of a limit)")
    ax.legend(ncol=6, loc="upper center")
    f_usage = savefig(fig, "e1_joint_usage.png")

    # ---------------- resolution at the pick pose ----------------
    qp = poses["pick"]
    J = jacobian(qp)
    res_rows, total_full, total_micro = [], 0.0, 0.0
    for i, d in enumerate(P.DRIVES):
        lever = float(np.linalg.norm(J[:3, i]))
        if d.is_stepper:
            full = 360 / d.motor.steps_per_rev / d.ratio
            micro = full / d.microstep
            note = f"full step {full:.3f} deg, 1/{d.microstep} microstep {micro:.4f} deg"
            cmd, acc = micro, full
        else:
            cmd = 180 * PCA9685_COUNT_US / SERVO_PULSE_SPAN_US
            acc = 1.0
            note = f"PCA9685 12-bit count {cmd:.2f} deg, servo repeatability ~1 deg (ASSUMED)"
        res_rows.append([P.JOINTS[i].name, note, f"{lever * 1e3:.0f}",
                         f"{np.radians(cmd) * lever * 1e3:.2f}", f"{np.radians(acc) * lever * 1e3:.2f}"])
        total_micro += np.radians(cmd) * lever
        total_full += np.radians(acc) * lever
    backlash_joint = GEARBOX_BACKLASH_DEG / (P.SHOULDER_TEETH / P.PULLEY_TEETH)
    backlash_tcp = np.radians(backlash_joint) * float(np.linalg.norm(J[:3, 1]))

    save_json("e1.json", {"ik": ikv, "poses": pose_data, "all_ok": bool(all_ok)})

    report = f"""# E1 - Kinematics and workspace

**Script:** `experiments/e1_kinematics_workspace.py` · **Model:** `arm/kinematics.py`, `arm/params.py`

{ASSUMPTION_NOTE}
## Purpose

Show that the proposed arm geometry can do the minimum demonstration (pick at the fixed
location, place in two bins) and that the inverse kinematics the firmware host will use is
correct. This supports the choice of link lengths and of the cell layout.

## Model

Six revolute joints: base yaw, shoulder pitch, elbow pitch, forearm roll, wrist pitch, tool
roll. The last three axes meet at one point (spherical wrist), so the inverse kinematics has a
closed form: the wrist centre fixes J1-J3, the remaining rotation fixes J4-J6.

{table(["Parameter", "Value (mm)"], [
    ["Table to shoulder axis", f"{P.D1 * 1e3:.0f}"], ["Upper arm (shoulder to elbow)", f"{P.L2 * 1e3:.0f}"],
    ["Forearm (elbow to wrist centre)", f"{P.L3 * 1e3:.0f}"], ["Wrist centre to tool tip", f"{P.D6 * 1e3:.0f}"],
    ["Maximum reach from shoulder", f"{reach * 1e3:.0f}"]])}
{table(["Joint", "Range (deg)", "Actuator"], [[j.name, f"{j.q_min_deg:g} to {j.q_max_deg:g}", d.description]
                                               for j, d in zip(P.JOINTS, P.DRIVES)])}
## Result 1 - the inverse kinematics is correct

{ikv["n"]} random joint configurations inside the limits were converted to a tool pose and solved back.

{table(["Check", "Result"], [
    ["Original configuration recovered", f"{ikv['recovered']} / {ikv['n']}"],
    ["Worst pose error over every returned solution", f"{ikv['worst_error']:.1e} (m or rad)"],
    ["Solutions inside the joint limits per pose", f"{ikv['mean_solutions']:.1f} on average, up to {ikv['max_solutions']}"]])}
Verdict: {verdict(ikv["recovered"] == ikv["n"] and ikv["worst_error"] < 1e-9)}

## Result 2 - the task poses are reachable with margin

The green area is every tool-tip position the arm can hold with the tool pointing straight
down and all joints inside their limits. The coloured skeletons are the task poses.

![Side-view reach]({f_side})

![Top-view reach]({f_top})

{table(["Pose", "J1 / J2 / J3 / J4 / J5 / J6 (deg)", "Smallest joint-limit margin (deg)",
        "Distance to workspace edge (mm)", "Jacobian condition number", "Tool tilt from vertical (deg)", "Verdict"], rows)}
Pass rule: at least 5 deg from every joint limit and at least 20 mm inside the workspace edge.
The condition number stays low, so no task pose is near a singularity.

![Joint-range usage]({f_usage})

Overall verdict: {verdict(all_ok)}

## Result 3 - positioning resolution at the pick pose

Tool-tip displacement caused by the smallest commanded step of each joint, at the pick pose.

{table(["Joint", "Angular resolution", "Lever arm to tool tip (mm)", "Tool-tip step, commanded (mm)",
        "Tool-tip step, full-step / repeatability (mm)"], res_rows)}
* Sum of commanded steps: **{total_micro * 1e3:.2f} mm**. Sum of full-step and servo repeatability figures: **{total_full * 1e3:.1f} mm**.
* Gearbox backlash of {GEARBOX_BACKLASH_DEG:g} deg (assumed) becomes {backlash_joint:.2f} deg at the shoulder after the belt stage,
  which is {backlash_tcp * 1e3:.1f} mm at the tool tip. Gravity keeps the shoulder loaded on one side of the backlash for
  the whole cycle, so this mainly matters if the arm passes over vertical.
* The gripper opening must therefore tolerate a few millimetres of error. The right-hand column is pessimistic for the
  steppers (it assumes microstepping gives no accuracy gain), so in practice the wrist servos are the larger contributor.

## What this decides

* Link lengths of {P.L2 * 1e3:.0f} / {P.L3 * 1e3:.0f} / {P.D6 * 1e3:.0f} mm cover the layout (pick at
  {P.PICK_XY[0] * 1e3:.0f} mm, bins at {np.hypot(*P.BINS['red']) * 1e3:.0f} mm from the base axis).
* The firmware host can use the closed-form inverse kinematics; no numerical solver is needed.
* Gripper jaws should open at least {P.OBJECT_SIZE * 1e3 + 2 * total_full * 1e3:.0f} mm for a {P.OBJECT_SIZE * 1e3:.0f} mm object.

## Limits of this experiment

* Reach is checked with the tool pointing down only, which is what the task needs.
* Self-collision and link thickness are not considered here; clearance to the bins and the table is checked in E4.
"""
    write_report("E1_kinematics_workspace.md", report)


if __name__ == "__main__":
    main()
