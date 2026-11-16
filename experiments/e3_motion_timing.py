"""E3 - Motion profile, step rates and cycle time.

Questions answered:
  1. How long does one sort cycle take with the planned speed limits?
  2. What step rate must the Arduino UNO generate, and which microstep setting
     and step-generation method does that require?
  3. Do the steppers stay in the speed range where their torque is still available?

Run:  uv run python -m experiments.e3_motion_timing
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from arm import params as P
from arm.task import plan_cycle
from .common import ASSUMPTION_NOTE, JOINT_COLOURS, save_json, savefig, table, verdict, write_report

# Step-generation budgets on a 16 MHz ATmega328P (ASSUMED, to be measured on the bench):
BUDGETS = {
    "polled stepping in loop() (AccelStepper-style)": 4_000,     # library documentation figure, shared by all axes
    "timer-interrupt stepping (grbl-style)": 20_000,             # conservative; grbl documents ~30 kHz
}
MICROSTEPS = (8, 16, 32, 64)   # what the TMC2209 offers through its MS1/MS2 pins
STEPPERS = [i for i, d in enumerate(P.DRIVES) if d.is_stepper]


def step_rates(traj, microstep=None):
    cols = []
    for i in STEPPERS:
        d = P.DRIVES[i]
        k = d.steps_per_rad * ((microstep / d.microstep) if microstep else 1.0)
        cols.append(np.abs(traj.qd[:, i]) * k)
    return np.array(cols).T


def shade(ax, traj):
    for k, (name, t0, t1) in enumerate(traj.segments):
        if k % 2 == 0:
            ax.axvspan(t0, t1, color="k", alpha=0.045, lw=0)


def main():
    traj = plan_cycle(axes_check=False)     # timing is about the sort cycle; the all-axes check is a one-off
    chk = plan_cycle(axes_check=True)
    chk_peak = float(step_rates(chk).sum(axis=1).max())
    T_nom = traj.duration
    n_obj = sum(1 for e in traj.events if e[1] == "attach")
    t_return = sum(t1 - t0 for name, t0, t1 in traj.segments if name in ("return to ready", "settle"))
    per_object = (T_nom - t_return) / n_obj

    # ---------------- profiles ----------------
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    for i in range(P.N):
        axes[0].plot(traj.t, np.degrees(traj.q[:, i]), color=JOINT_COLOURS[i], label=f"J{i + 1}")
        axes[1].plot(traj.t, np.degrees(traj.qd[:, i]), color=JOINT_COLOURS[i])
    axes[2].fill_between(traj.t, traj.grip, color="#888", step="mid", label="gripper closed")
    for t, kind, obj in traj.events:
        if kind in ("attach", "detach", "read colour"):
            axes[2].axvline(t, color={"attach": "g", "detach": "r", "read colour": "b"}[kind], lw=1)
            axes[2].text(t, 1.04, kind, rotation=30, fontsize=7, ha="left", va="bottom")
    for ax in axes:
        shade(ax, traj)
    axes[0].set(ylabel="joint angle (deg)", title="Planned sort cycle: two objects, one per bin")
    axes[0].legend(ncol=6, loc="upper right")
    axes[1].set(ylabel="joint speed (deg/s)")
    axes[2].set(ylabel="gripper", xlabel="time (s)", ylim=(0, 1.45), yticks=[0, 1])
    f_prof = savefig(fig, "e3_cycle_profiles.png")

    # ---------------- step rates ----------------
    rates = step_rates(traj)
    total = rates.sum(axis=1)
    peak_total = float(total.max())
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for k, i in enumerate(STEPPERS):
        ax.plot(traj.t, rates[:, k] / 1e3, color=JOINT_COLOURS[i], label=f"J{i + 1} ({P.DRIVES[i].ratio:.1f}:1)")
    ax.plot(traj.t, total / 1e3, "k", lw=1.6, label="all steppers together")
    for (name, b), ls in zip(BUDGETS.items(), ("--", ":")):
        ax.axhline(b / 1e3, color="r", ls=ls, lw=1.2, label=f"{name}: {b / 1e3:g} kHz")
    shade(ax, traj)
    ax.set(xlabel="time (s)", ylabel="step rate (kHz)", ylim=(0, max(peak_total / 1e3 * 1.25, 6)),
           title=f"Step pulses needed at 1/{P.DRIVES[1].microstep} microstepping, nominal speeds")
    ax.set_ylim(0, 21.5)
    ax.legend(loc="upper right", ncol=2)
    f_rate = savefig(fig, "e3_step_rates.png")

    # ---------------- cycle time against budget and microstep ----------------
    grid, peaks = {}, {}
    for m in MICROSTEPS:
        peaks[m] = float(step_rates(traj, m).sum(axis=1).max())
        for name, b in BUDGETS.items():
            slow = max(1.0, peaks[m] / b)
            grid[(m, name)] = (slow, T_nom * slow, per_object * slow)

    # confirm the time-stretch rule by re-planning one slowed-down case
    name0 = next(iter(BUDGETS))
    slow0 = grid[(P.DRIVES[1].microstep, name0)][0]
    check = plan_cycle(speed=1 / slow0, axes_check=False)
    check_peak = float(step_rates(check).sum(axis=1).max())
    check_ok = check_peak <= BUDGETS[name0] * 1.03 and abs(check.duration - T_nom * slow0) / (T_nom * slow0) < 0.03

    b_axis = np.logspace(np.log10(1500), np.log10(60000), 200)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    for m, c in zip(MICROSTEPS, ("#1f77b4", "#2ca02c", "#ff7f0e", "#d62728")):
        ax.plot(b_axis / 1e3, per_object * np.maximum(1, peaks[m] / b_axis), color=c, label=f"1/{m} microstep")
    for (name, b), ls in zip(BUDGETS.items(), ("--", ":")):
        ax.axvline(b / 1e3, color="k", ls=ls, lw=1)
        ax.text(b / 1e3 * 1.04, 55, name.split(" (")[0], rotation=90, fontsize=7, va="top")
    ax.set(xscale="log", xlabel="total step rate the controller can generate (kHz)",
           ylabel="time per object (s)", ylim=(0, 60),
           title="Cycle time against step-generation capability")
    ax.legend()
    f_budget = savefig(fig, "e3_cycle_time_vs_budget.png")

    # ---------------- motor speed against torque corner ----------------
    speed_rows, speed_ok = [], True
    for i in STEPPERS:
        d = P.DRIVES[i]
        peak_joint = float(np.abs(traj.qd[:, i]).max())
        rpm = peak_joint * d.ratio * 60 / (2 * np.pi)
        corner_rpm = d.corner_speed() * d.ratio * 60 / (2 * np.pi)
        ok = rpm < corner_rpm
        speed_ok &= ok
        speed_rows.append([P.JOINTS[i].name, f"{np.degrees(peak_joint):.0f}", f"{rpm:.0f}", f"{corner_rpm:.0f}",
                           f"{rpm / corner_rpm:.0%}", verdict(ok)])

    alone_rows = []
    for i in STEPPERS:
        d = P.DRIVES[i]
        row = [P.JOINTS[i].name, f"{P.JOINTS[i].vmax_deg:g}"]
        for m in MICROSTEPS:
            spr = d.motor.steps_per_rev * m * d.ratio
            row.append(" / ".join(f"{b / spr * 360:.0f}" for b in BUDGETS.values()))
        alone_rows.append(row)

    seg_rows = [[name, f"{t0:.2f}", f"{t1 - t0:.2f}"] for name, t0, t1 in traj.segments]
    grid_rows = []
    for m in MICROSTEPS:
        row = [f"1/{m}", f"{peaks[m] / 1e3:.1f}"]
        for name in BUDGETS:
            slow, T, per = grid[(m, name)]
            row += [f"{per:.1f}", f"{60 / per:.1f}"]
        grid_rows.append(row)

    m_sel = P.DRIVES[1].microstep
    polled, timer = (grid[(m_sel, n)] for n in BUDGETS)
    save_json("e3.json", {"cycle_time": T_nom, "per_object": per_object, "peak_step_rate": peak_total,
                          "per_object_polled": polled[2], "per_object_timer": timer[2]})

    b_polled = BUDGETS[name0]
    if polled[0] <= 1.0:
        firmware_decision = (
            f"**Polled stepping is enough on paper, with little headroom.** The nominal cycle needs "
            f"{peak_total / 1e3:.1f} kHz, which is {peak_total / b_polled:.0%} of the assumed polled budget. I2C calls to the "
            f"servo driver and the colour sensor share the same loop and will eat into it. Plan: start with polled "
            f"stepping because it is the simplest, and move step generation into a hardware timer interrupt if the "
            f"bench test shows late or missed pulses.")
    else:
        firmware_decision = (
            f"**Step pulses from a hardware timer interrupt, not from `loop()`.** Polled stepping would stretch the "
            f"cycle by {polled[0]:.1f}x ({polled[2]:.1f} s per object against {timer[2]:.1f} s). The demonstration still "
            f"works with polled stepping, so it is the fallback.")

    report = f"""# E3 - Motion profile, step rates and cycle time

**Script:** `experiments/e3_motion_timing.py` · **Model:** `arm/task.py`, `arm/params.py`

{ASSUMPTION_NOTE}
## Purpose

Fix the speed limits, the microstep setting and the way the Arduino UNO generates step pulses.
Three steppers behind {P.DRIVES[0].ratio:g}:1, {P.DRIVES[1].ratio:.1f}:1 and {P.DRIVES[2].ratio:g}:1 reductions
need many pulses per degree, and the UNO is a slow controller, so this is a real constraint on cycle time.

## Method

* This experiment times the sort cycle only (the all-axes check of E4 is left out). The sort cycle is planned by `arm/task.py`: joint-space moves between stations and straight
  vertical lines for the approach and the retreat, all with trapezoidal speed profiles
  (one third accelerating, one third cruising, one third braking).
* Step rate of an axis = joint speed x 200 x microstep x reduction / 360 deg.
* Two step-generation methods are compared, with these total budgets (assumed, to be measured on
  the first day of the build phase):
{chr(10).join(f"  * {n}: {b:,} steps/s" for n, b in BUDGETS.items())}
* If the nominal cycle needs more than the budget, every move is stretched in time by the same factor.
  This rule was confirmed by re-planning one slowed case: {verdict(check_ok)}.

Planned limits:

{table(["Joint", "Speed limit (deg/s)", "Acceleration limit (deg/s^2)", "Steps per degree at 1/%d" % m_sel],
       [[j.name, f"{j.vmax_deg:g}", f"{j.amax_deg:g}",
         f"{d.steps_per_rad * np.pi / 180:.1f}" if d.is_stepper else "servo (PWM)"] for j, d in zip(P.JOINTS, P.DRIVES)])}
## Result 1 - the nominal cycle

Two objects sorted in **{T_nom:.1f} s**: **{per_object:.1f} s per object**, or {60 / per_object:.1f} objects per minute, if the
controller can deliver the pulses.

![Cycle profiles]({f_prof})

{table(["Segment", "Start (s)", "Duration (s)"], seg_rows)}
## Result 2 - step rate and what it means for the firmware

![Step rates]({f_rate})

Peak demand at nominal speed and 1/{m_sel} microstepping: **{peak_total / 1e3:.1f} kHz** with the steppers moving together.
The one-off all-axes check (each joint alone, see E4) peaks at {chk_peak / 1e3:.1f} kHz.

{table(["Microstep", "Peak step rate (kHz)"] + [h for n in BUDGETS for h in (f"Time per object, {n.split(' (')[0]} (s)", "Objects per minute")],
       grid_rows)}
![Cycle time against budget]({f_budget})

Highest speed each axis could reach if it moved alone (deg/s, polled / timer-interrupt):

{table(["Joint", "Planned limit (deg/s)"] + [f"1/{m}" for m in MICROSTEPS], alone_rows)}
## Result 3 - the motors stay in their flat-torque range

Stepper torque is roughly constant up to a corner speed set by supply voltage and winding
inductance, `V / (2 L I steps_per_rev)`, then falls. With {P.SUPPLY_VOLTAGE:g} V the corner is far above what the cycle uses.

{table(["Joint", "Peak joint speed (deg/s)", "Peak motor speed (rpm)", "Corner speed (rpm)", "Used", "Verdict"], speed_rows)}
## What this decides

1. **1/{m_sel} microstepping on every stepper.** It is the coarsest setting the TMC2209 offers through its
   MS pins, and the driver interpolates to 256 microsteps internally, so finer settings add pulse load
   without a visible gain. Resolution at 1/{m_sel} is already below the servo errors (see E1).
2. {firmware_decision}
3. **Speed limits** of {P.JOINTS[0].vmax_deg:g} / {P.JOINTS[1].vmax_deg:g} / {P.JOINTS[2].vmax_deg:g} deg/s on base / shoulder / elbow
   are kept for the dynamic simulation (E4).

## Limits of this experiment

* The two step-rate budgets are assumptions about the UNO, not measurements. I2C traffic to the
  PCA9685 and the colour sensor will take time from a polled loop; a bench test with the real
  firmware is the first task of the build phase.
* Whether the motors can actually deliver the planned accelerations is a torque question, answered in E4.
* The cycle has no waiting time for the colour reading or for an operator placing the next object.
"""
    write_report("E3_motion_timing.md", report)


if __name__ == "__main__":
    main()
