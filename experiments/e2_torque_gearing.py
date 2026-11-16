"""E2 - Static torque, load and reduction-ratio selection.

Questions answered:
  1. What is the worst gravity torque at each joint?
  2. Which joint gets the single geared stepper, and what belt/gear ratios are needed?
  3. How much mass growth can the chosen drives absorb?

Run:  uv run python -m experiments.e2_torque_gearing
"""
from __future__ import annotations

import itertools

import numpy as np
import matplotlib.pyplot as plt

from arm import params as P
from arm.dynamics import gravity_torque, moving_mass, point_masses
from arm.kinematics import fk_frames
from arm.task import key_poses
from .common import ASSUMPTION_NOTE, JOINT_COLOURS, save_json, savefig, table, verdict, write_report

SF_MIN = P.MIN_STATIC_SF
SF_STALL = 1 / P.STEPPER_DYNAMIC_FACTOR      # below this the load exceeds the usable pull-out torque
BELT_CENTRES = {"shoulder": 0.070, "elbow": 0.230}   # ASSUMED motor-to-joint centre distances (m)
GT2_PITCH = 0.002


def worst_case_sets():
    """Gravity torque per unit structure mass and per kg payload over a pose grid."""
    q2 = np.radians(np.arange(P.JOINTS[1].q_min_deg, P.JOINTS[1].q_max_deg + 1, 10))
    q3 = np.radians(np.arange(P.JOINTS[2].q_min_deg, P.JOINTS[2].q_max_deg + 1, 10))
    q4 = np.radians([0, 90])
    q5 = np.radians(np.arange(-90, 91, 30))
    A, B = [], []
    for a, b, c, d in itertools.product(q2, q3, q4, q5):
        q = [0, a, b, c, d, 0]
        A.append(gravity_torque(q, payload=0.0))
        B.append(gravity_torque(q, payload=1.0, mass_scale=0.0))
    return np.array(A), np.array(B)


def worst(A, B, scale=1.0, payload=P.PAYLOAD_MASS):
    return np.abs(scale * A + payload * B).max(axis=0)


def heatmaps():
    q2 = np.arange(P.JOINTS[1].q_min_deg, P.JOINTS[1].q_max_deg + 1, 5)
    q3 = np.arange(P.JOINTS[2].q_min_deg, P.JOINTS[2].q_max_deg + 1, 5)
    tau = np.zeros((len(q3), len(q2), 2))
    for i, b in enumerate(q3):
        for k, a in enumerate(q2):
            tau[i, k] = np.abs(gravity_torque(np.radians([0, a, b, 0, 0, 0])))[1:3]
    return q2, q3, tau


def base_inertia(q):
    frames = fk_frames(q)
    I = 0.0
    for link, m, pos in point_masses():
        c = (frames[link - 1] @ np.array([*pos, 1.0]))[:3]
        I += m * (c[0] ** 2 + c[1] ** 2)
    return I


def belt_length(teeth_small, teeth_big, centre):
    d, D = teeth_small * GT2_PITCH / np.pi, teeth_big * GT2_PITCH / np.pi
    return 2 * centre + np.pi * (D + d) / 2 + (D - d) ** 2 / (4 * centre)


def main():
    A, B = worst_case_sets()
    tau_max = worst(A, B)
    tau_empty = worst(A, B, payload=0.0)
    poses = key_poses()
    tau_task = np.max([np.abs(gravity_torque(q)) for q in poses.values()], axis=0)

    # base axis carries no gravity load on a level table: size it on inertia
    q_stretch = np.radians([0, 90, 0, 0, 0, 0])
    I_base = base_inertia(q_stretch) + P.DRIVES[0].reflected_inertia
    tau_base = I_base * P.A_MAX[0]
    required = tau_max.copy()
    required[0] = tau_base

    # ---------------- heatmaps ----------------
    q2, q3, tau = heatmaps()
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.3))
    for ax, k, name in zip(axes, (0, 1), ("shoulder (J2)", "elbow (J3)")):
        im = ax.pcolormesh(q2, q3, tau[:, :, k], cmap="viridis", shading="nearest")
        cs = ax.contour(q2, q3, tau[:, :, k], colors="w", linewidths=0.6, levels=6)
        ax.clabel(cs, fmt="%.1f", fontsize=7)
        for pname, q in poses.items():
            ax.plot(np.degrees(q[1]), np.degrees(q[2]), "o", mfc="none", mec="r", ms=7,
                    label="task poses" if pname == "ready" else None)
        ax.set(xlabel="shoulder angle J2 (deg, 0 = vertical)", ylabel="elbow angle J3 (deg, 0 = straight)",
               title=f"Holding torque at the {name}, N.m\nwrist straight, {P.PAYLOAD_MASS * 1e3:.0f} g payload")
        ax.grid(False)
        ax.legend(loc="lower left")
        fig.colorbar(im, ax=ax)
    f_heat = savefig(fig, "e2_gravity_heatmaps.png")

    # ---------------- drive options ----------------
    def joint_torque(motor, ratio, eff):
        return motor.hold_torque * ratio * eff

    options = {}
    for jname, idx in (("shoulder", 1), ("elbow", 2)):
        rows = []
        cands = [("plain 17HS4401, direct", P.NEMA17, 1.0, 1.0),
                 ("geared 17HS4401S + PG518, direct", P.NEMA17_S, P.GEARBOX, P.GEARBOX_EFF)]
        for teeth in (48, 64, 80, 96, 120):
            r = teeth / P.PULLEY_TEETH
            cands.append((f"plain + GT2 16:{teeth}", P.NEMA17, r, P.BELT_EFF))
            cands.append((f"geared + GT2 16:{teeth}", P.NEMA17_S, P.GEARBOX * r, P.GEARBOX_EFF * P.BELT_EFF))
        for label, motor, ratio, eff in cands:
            t = joint_torque(motor, ratio, eff)
            sf = t / tau_max[idx]
            rows.append([label, f"{ratio:.2f}", f"{t:.2f}", f"{sf:.2f}", verdict(sf >= SF_MIN)])
        options[jname] = rows

    teeth = np.arange(16, 129, 4)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    curves = [("shoulder, geared motor + belt", P.NEMA17_S, P.GEARBOX, P.GEARBOX_EFF * P.BELT_EFF, 1, "-"),
              ("shoulder, plain motor + belt", P.NEMA17, 1.0, P.BELT_EFF, 1, "--"),
              ("elbow, plain motor + belt", P.NEMA17, 1.0, P.BELT_EFF, 2, "-"),
              ("elbow, geared motor + belt", P.NEMA17_S, P.GEARBOX, P.GEARBOX_EFF * P.BELT_EFF, 2, "--")]
    for label, motor, base, eff, idx, ls in curves:
        sf = joint_torque(motor, base * teeth / P.PULLEY_TEETH, eff) / tau_max[idx]
        ax.plot(teeth, sf, ls, color=JOINT_COLOURS[idx], label=label)
    ax.axhline(SF_MIN, color="k", lw=1)
    ax.text(17, SF_MIN + 0.12, f"design rule: safety factor {SF_MIN:g}", fontsize=8)
    for idx, t in ((1, P.SHOULDER_TEETH), (2, P.ELBOW_TEETH)):
        ax.plot(t, P.DRIVES[idx].hold_torque / tau_max[idx], "o", color=JOINT_COLOURS[idx], ms=9, mec="k")
    ax.set(xlabel="teeth on the printed GT2 pulley (motor pulley: 16 teeth)", ylim=(0, 8),
           ylabel="holding torque / worst gravity torque",
           title="Choosing the belt ratio (filled dots = selected design)")
    ax.legend(loc="upper left")
    f_ratio = savefig(fig, "e2_ratio_selection.png")

    # ---------------- required vs available ----------------
    avail = np.array([d.hold_torque for d in P.DRIVES])
    sf = np.where(required > 1e-6, avail / np.maximum(required, 1e-6), np.inf)

    def sf_txt(v):
        return f"{v:.1f}" if np.isfinite(v) else "no load"
    fig, ax = plt.subplots(figsize=(8, 3.8))
    x = np.arange(P.N)
    ax.bar(x - 0.2, np.maximum(required, 1e-3), 0.4, color="#d62728", label="worst-case load")
    ax.bar(x + 0.2, avail, 0.4, color="#2ca02c", label="holding / stall torque at the joint")
    for i in range(P.N):
        ax.text(i + 0.2, avail[i] * 1.08, f"SF {sf_txt(sf[i])}", ha="center", fontsize=8)
    ax.set_yscale("log")
    ax.set_xticks(x, [j.name for j in P.JOINTS], rotation=15)
    ax.set(ylabel="torque (N.m, log scale)", title="Worst-case load against available torque, selected drives")
    ax.legend()
    f_bar = savefig(fig, "e2_required_vs_available.png")

    # ---------------- sensitivity ----------------
    scales = np.linspace(0.7, 1.6, 19)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.9), sharey=True)
    sens = {}
    for ax, idx, name in zip(axes, (1, 2), ("shoulder", "elbow")):
        for payload, ls in ((0.05, "-"), (0.10, "--"), (0.20, ":")):
            s = [avail[idx] / worst(A, B, sc, payload)[idx] for sc in scales]
            ax.plot((scales - 1) * 100, s, ls, color=JOINT_COLOURS[idx], label=f"payload {payload * 1e3:.0f} g")
            sens[(name, payload)] = s
        ax.axhline(SF_MIN, color="k", lw=1)
        ax.axhline(SF_STALL, color="r", lw=1)
        ax.text(-29, SF_MIN + 0.05, "design rule", fontsize=8)
        ax.text(-29, SF_STALL + 0.05, "usable stepper torque exceeded", fontsize=8, color="r")
        ax.set(xlabel="moving-mass error versus the budget (%)", title=f"{name} safety factor")
        ax.legend(loc="upper right")
    axes[0].set_ylabel("holding torque / worst gravity torque")
    f_sens = savefig(fig, "e2_sensitivity.png")

    def max_growth(idx, payload=P.PAYLOAD_MASS, target=SF_MIN):
        fine = np.linspace(0.5, 3.0, 501)
        ok = [sc for sc in fine if avail[idx] / worst(A, B, sc, payload)[idx] >= target]
        return (max(ok) - 1) * 100 if ok else float("nan")

    growth = {name: (max_growth(idx), max_growth(idx, target=SF_STALL)) for name, idx in (("shoulder", 1), ("elbow", 2))}

    belts = {"shoulder": belt_length(P.PULLEY_TEETH, P.SHOULDER_TEETH, BELT_CENTRES["shoulder"]),
             "elbow": belt_length(P.PULLEY_TEETH, P.ELBOW_TEETH, BELT_CENTRES["elbow"])}
    gearbox_out = tau_max[1] / (P.SHOULDER_TEETH / P.PULLEY_TEETH) / P.BELT_EFF

    carried = sum(m for link, m, _ in point_masses() if link >= 2)
    save_json("e2.json", {"tau_worst": required.tolist(), "tau_task": tau_task.tolist(),
                          "available": avail.tolist(), "sf": [float(min(v, 999)) for v in sf]})

    sel_rows = []
    for i, (j, d) in enumerate(zip(P.JOINTS, P.DRIVES)):
        basis = "inertia at max acceleration" if i == 0 else "gravity, worst pose"
        sel_rows.append([j.name, d.description, f"{d.ratio:.2f}", f"{required[i]:.3f}", basis,
                         f"{tau_task[i]:.3f}", f"{avail[i]:.2f}", sf_txt(sf[i]), verdict(sf[i] >= SF_MIN)])

    mass_rows = [[m.name, f"J{m.link}", f"{m.mass * 1e3:.0f}", f"{m.pos[2] * 1e3:.0f}"] for m in P.MASSES]
    mass_rows.append(["payload", "J6", f"{P.PAYLOAD_MASS * 1e3:.0f}", f"{P.PAYLOAD_POS[2] * 1e3:.0f}"])

    report = f"""# E2 - Torque, load and reduction ratios

**Script:** `experiments/e2_torque_gearing.py` · **Model:** `arm/dynamics.py`, `arm/params.py`

{ASSUMPTION_NOTE}
## Purpose

Decide how the three steppers are geared. The kit has one stepper with a 5.18:1 planetary
gearbox, two plain steppers (0.49 N.m), 2 m of GT2 belt and two 16-tooth pulleys. This
experiment finds the worst load on every joint and picks the reductions that hold it with margin.

## Method

* Every part is a point mass on its link (table below). The holding torque at joint *j* is
  the sum of `m g (axis_j x (r_mass - r_joint)) . z` over all masses the joint carries.
* The same masses are built into the MuJoCo model, and the two agree to better than
  1e-6 N.m on random poses (checked in `experiments/run_all.py`).
* Worst case: the whole shoulder/elbow range in 10 deg steps, wrist at seven pitch angles and
  two roll angles ({len(A)} poses), with the {P.PAYLOAD_MASS * 1e3:.0f} g design payload.
* Design rule: holding torque at the joint at least **{SF_MIN:g} x** the worst static load. The
  margin covers the drop from holding to usable running torque (taken as
  {P.STEPPER_DYNAMIC_FACTOR:.0%} of holding), acceleration loads and friction.

### Mass budget

Total moving mass including the turret and the payload: **{moving_mass() * 1e3:.0f} g**.

{table(["Item", "Moves with", "Mass (g)", "Distance along the link from its joint (mm)"], mass_rows)}
## Result 1 - worst-case loads

{table(["Joint", "Worst load with payload (N.m)", "Worst load, empty (N.m)", "Worst load at the task poses (N.m)"],
       [[j.name, f"{tau_max[i]:.3f}", f"{tau_empty[i]:.3f}", f"{tau_task[i]:.3f}"] for i, j in enumerate(P.JOINTS)])}
The shoulder peak is the arm stretched out horizontally. The base carries no gravity load on a
level table, so it is sized on inertia instead: {I_base:.3f} kg.m^2 about the base axis with the arm
stretched, times the planned {np.degrees(P.A_MAX[0]):.0f} deg/s^2, gives {tau_base:.2f} N.m.

![Gravity torque maps]({f_heat})

## Result 2 - which motor goes where, and the ratios

A plain stepper on the shoulder would need more than {SF_MIN * tau_max[1] / (P.NEMA17.hold_torque * P.BELT_EFF):.0f}:1
of belt reduction, which is a {SF_MIN * tau_max[1] / (P.NEMA17.hold_torque * P.BELT_EFF) * P.PULLEY_TEETH * GT2_PITCH / np.pi * 1e3:.0f} mm
pulley. **The geared stepper therefore goes on the shoulder**, with a belt stage after it. The gearbox
alone is not enough either.

![Ratio selection]({f_ratio})

### Shoulder options (worst load {tau_max[1]:.2f} N.m)

{table(["Option", "Total ratio", "Holding torque at joint (N.m)", "Safety factor", f"SF >= {SF_MIN:g}"], options["shoulder"])}
### Elbow options (worst load {tau_max[2]:.2f} N.m)

{table(["Option", "Total ratio", "Holding torque at joint (N.m)", "Safety factor", f"SF >= {SF_MIN:g}"], options["elbow"])}
### Selected drives

{table(["Joint", "Drive", "Ratio", "Design load (N.m)", "Basis", "Load at task poses (N.m)",
        "Available (N.m)", "Safety factor", "Verdict"], sel_rows)}
![Required against available]({f_bar})

Checks on the selection:

* Printed pulley pitch diameters: shoulder {P.SHOULDER_TEETH} teeth = {P.SHOULDER_TEETH * GT2_PITCH / np.pi * 1e3:.0f} mm,
  elbow {P.ELBOW_TEETH} teeth = {P.ELBOW_TEETH * GT2_PITCH / np.pi * 1e3:.0f} mm.
* Belt needed: about {belts["shoulder"] * 1e3:.0f} mm (shoulder) + {belts["elbow"] * 1e3:.0f} mm (elbow) =
  {sum(belts.values()) * 1e3:.0f} mm of the 2000 mm provided, for assumed centre distances of
  {BELT_CENTRES["shoulder"] * 1e3:.0f} and {BELT_CENTRES["elbow"] * 1e3:.0f} mm. The belt is open-ended, so each end is
  clamped to the printed pulley; this works because neither joint turns a full revolution.
* Gearbox output torque at the worst shoulder load: {gearbox_out:.2f} N.m. Check this against the
  PG518 rated output torque in its datasheet.
* Both 16-tooth pulleys are used by the shoulder and the elbow. The base therefore needs a printed
  gear stage (modelled as {P.BASE_GEAR:g}:1), or a third pulley requested from the organisers.
* Wrist: the two 20 kg.cm servos go on forearm roll and wrist pitch, the two 13 kg.cm servos on
  tool roll and the gripper. All have a large margin.

## Result 3 - sensitivity to mass

Printed-part masses are the least certain inputs, so the safety factor is recomputed with all
moving masses scaled together and with heavier payloads.

![Sensitivity]({f_sens})

{table(["Joint", f"Mass growth before SF drops below {SF_MIN:g}", "Mass growth before usable stepper torque is exceeded"],
       [[n, f"{g[0]:+.0f} %", f"{g[1]:+.0f} %"] for n, g in growth.items()])}
## What this decides

1. Geared stepper on the **shoulder** with a 16:{P.SHOULDER_TEETH} belt stage (total {P.DRIVES[1].ratio:.1f}:1).
2. Plain stepper on the **elbow** with a 16:{P.ELBOW_TEETH} belt stage, motor mounted behind the shoulder
   axis so it acts as a small counterweight.
3. Plain stepper on the **base** with a printed {P.BASE_GEAR:g}:1 stage, chosen for resolution as much as for torque.
4. Mass targets for the CAD: keep the mass carried by the shoulder near {carried * 1e3:.0f} g (payload included).
   Each extra gram at the wrist adds about {P.G * (P.L2 + P.L3):.1f} mN.m at the shoulder.

## Limits of this experiment

* Static only. Acceleration loads and the fall of stepper torque with speed are checked in E4.
* Friction, cable drag and belt pre-tension are not modelled; the safety factor is there to cover them.
* Holding torque of the geared stepper ({P.NEMA17_S.hold_torque} N.m) and the efficiencies
  ({P.GEARBOX_EFF:.0%} gearbox, {P.BELT_EFF:.0%} belt) are assumed and must be confirmed from datasheets.
"""
    write_report("E2_torque_gearing.md", report)


if __name__ == "__main__":
    main()
