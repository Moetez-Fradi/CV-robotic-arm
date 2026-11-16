"""E5 - Power budget.

Questions answered:
  1. Is the 24 V 12.5 A supply enough, and with what margin?
  2. Are the two buck converters adequate for the servo and logic rails?

Run:  uv run python -m experiments.e5_power_budget   (uses E4 results if present)
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from arm import params as P
from .common import ASSUMPTION_NOTE, load_json, save_json, savefig, table, verdict, write_report

PSU_W, PSU_A = 300.0, 12.5
SERVO_RAIL_V, SERVO_BUCK_A, SERVO_BUCK_EFF = 6.0, 8.0, 0.90     # XL4016, 8 A (spec book); efficiency ASSUMED
LOGIC_RAIL_V, LOGIC_BUCK_A, LOGIC_BUCK_EFF = 5.0, 2.0, 0.85     # LM2596S; 2 A usable ASSUMED
DRIVER_LOSS_W = 1.0            # per TMC2209 (ASSUMED)
SERVO_MOVING_A = 0.7           # typical current of a loaded servo while moving (ASSUMED)
SERVO_HOLDING_A = 0.2          # holding a light load (ASSUMED)
FUSE_A = 20.0
LOGIC_LOADS = [("Arduino UNO", 0.050), ("PCA9685 logic", 0.010), ("TCS34725 + LED", 0.025),
               ("E18-D80NK proximity sensor", 0.025), ("3 x TMC2209 logic", 0.030)]   # ASSUMED typical


def main():
    e4 = load_json("e4.json") or {}
    mech = float(e4.get("peak_mech_power_W", 1.0))
    steppers = [d for d in P.DRIVES if d.is_stepper]
    servos = [d.motor for d in P.DRIVES if not d.is_stepper] + [P.GRIPPER_SERVO]

    # steppers: both windings carry rated/sqrt(2) rms, or one winding the full rated current
    stepper_w = [d.motor.rated_current**2 * d.motor.resistance + DRIVER_LOSS_W for d in steppers]
    stepper_typ = sum(stepper_w)
    stepper_worst = stepper_typ + mech

    servo_typ_a = 2 * SERVO_MOVING_A + (len(servos) - 2) * SERVO_HOLDING_A
    servo_worst_a = sum(s.stall_current for s in servos)
    servo_two_stall_a = sum(sorted((s.stall_current for s in servos), reverse=True)[:2]) + (len(servos) - 2) * SERVO_HOLDING_A
    logic_a = sum(a for _, a in LOGIC_LOADS)

    def input_w(servo_a):
        return servo_a * SERVO_RAIL_V / SERVO_BUCK_EFF

    logic_w = logic_a * LOGIC_RAIL_V / LOGIC_BUCK_EFF
    scenarios = {
        "typical (cycle running)": (stepper_typ, input_w(servo_typ_a), logic_w),
        "two servos stalled": (stepper_worst, input_w(servo_two_stall_a), logic_w),
        "all four servos stalled (fault)": (stepper_worst, input_w(servo_worst_a), logic_w),
    }

    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    names = list(scenarios)
    bottom = np.zeros(len(names))
    for k, (label, colour) in enumerate((("steppers + drivers", "#1f77b4"), ("servo rail", "#ff7f0e"), ("logic rail", "#2ca02c"))):
        vals = np.array([scenarios[n][k] for n in names])
        ax.barh(names, vals, left=bottom, color=colour, label=label)
        bottom += vals
    for y, total in enumerate(bottom):
        ax.text(total + 4, y, f"{total:.0f} W", va="center")
    ax.axvline(PSU_W, color="r", lw=1.5)
    ax.text(PSU_W - 4, 2.42, "supply rating 300 W", color="r", ha="right", fontsize=8)
    ax.set(xlabel="power drawn from the 24 V supply (W)", xlim=(0, 330), title="Power budget by scenario")
    ax.invert_yaxis()
    ax.legend(loc="lower right")
    f_bar = savefig(fig, "e5_power_budget.png")

    rows = []
    for n in names:
        total = sum(scenarios[n])
        rows.append([n, f"{scenarios[n][0]:.0f}", f"{scenarios[n][1]:.0f}", f"{scenarios[n][2]:.1f}", f"{total:.0f}",
                     f"{total / P.SUPPLY_VOLTAGE:.1f}", f"{total / PSU_W:.0%}", verdict(total < 0.8 * PSU_W)])
    worst_total = sum(scenarios[names[-1]])
    rail_rows = [
        ["Servo rail, typical", f"{servo_typ_a:.1f} A", f"{SERVO_BUCK_A:g} A (XL4016)", verdict(servo_typ_a < 0.8 * SERVO_BUCK_A)],
        ["Servo rail, two servos stalled", f"{servo_two_stall_a:.1f} A", f"{SERVO_BUCK_A:g} A (XL4016)", verdict(servo_two_stall_a < SERVO_BUCK_A)],
        ["Servo rail, all four stalled", f"{servo_worst_a:.1f} A", f"{SERVO_BUCK_A:g} A (XL4016)", verdict(servo_worst_a < SERVO_BUCK_A)],
        ["Logic rail", f"{logic_a * 1e3:.0f} mA", f"{LOGIC_BUCK_A:g} A (LM2596S)", verdict(logic_a < 0.8 * LOGIC_BUCK_A)],
        ["24 V input, worst case", f"{worst_total / P.SUPPLY_VOLTAGE:.1f} A", f"{FUSE_A:g} A fuse, {PSU_A:g} A supply",
         verdict(worst_total / P.SUPPLY_VOLTAGE < min(FUSE_A, PSU_A))],
    ]

    save_json("e5.json", {"typical_W": sum(scenarios[names[0]]), "worst_W": worst_total, "supply_W": PSU_W,
                          "servo_rail_typical_A": servo_typ_a, "servo_rail_worst_A": servo_worst_a})

    report = f"""# E5 - Power budget

**Script:** `experiments/e5_power_budget.py` · **Inputs:** `arm/params.py`, peak mechanical power from E4

{ASSUMPTION_NOTE}
> Servo currents, converter efficiencies and logic currents below are typical values, not datasheet
> figures for the delivered parts. Replace them in the script header when the datasheets are available.

## Purpose

Check that the provided 24 V 12.5 A supply and the two buck converters can feed the arm, and
propose which converter feeds which rail. The arm is mains-powered, so there is no battery-life question.

## Rail plan

{table(["Rail", "Source", "Loads"], [
    ["24 V", "power supply, through the emergency stop and the fuse", "3 x TMC2209 on the CNC shield (stepper power)"],
    [f"{SERVO_RAIL_V:g} V", "XL4016 buck converter", "4 servos through the PCA9685 V+ terminal"],
    [f"{LOGIC_RAIL_V:g} V", "LM2596S buck converter", "Arduino UNO, PCA9685 logic, colour sensor, proximity sensor, driver logic"]])}
Keeping servo power and logic power on separate converters stops servo current spikes from resetting the Arduino.

## Method

* **Steppers.** A chopper driver holds the winding current at its set value whether the motor moves
  or not, so each stepper draws about `I^2 R` in copper loss ({stepper_w[0] - DRIVER_LOSS_W:.1f} W at {steppers[0].motor.rated_current} A,
  {steppers[0].motor.resistance} ohm) plus about {DRIVER_LOSS_W:g} W in the driver, all the time. Peak mechanical power from
  the dynamic simulation is {mech:.2f} W in total, which is negligible beside that.
* **Servos.** Typical: two servos moving at {SERVO_MOVING_A} A and two holding at {SERVO_HOLDING_A} A. Worst: stalled
  at {servos[0].stall_current} A each.
* **Logic.** {", ".join(f"{n} {a * 1e3:.0f} mA" for n, a in LOGIC_LOADS)}.
* Converter efficiencies: {SERVO_BUCK_EFF:.0%} (XL4016), {LOGIC_BUCK_EFF:.0%} (LM2596S).

## Results

![Power budget]({f_bar})

{table(["Scenario", "Steppers (W)", "Servo rail (W)", "Logic rail (W)", "Total (W)", "24 V current (A)", "Share of supply", "Below 80 % of supply"], rows)}
{table(["Check", "Demand", "Capacity", "Verdict"], rail_rows)}
## What this decides

1. The 300 W supply is far larger than needed: about {sum(scenarios[names[0]]):.0f} W in normal running and
   {worst_total:.0f} W in the worst fault case.
2. The XL4016 covers normal servo use with a wide margin. {"All four servos stalled together would exceed its rating; "
   "that is a fault, not an operating case, and the firmware should stop the cycle if the gripper or wrist is blocked."
   if servo_worst_a >= SERVO_BUCK_A else "It also covers all four servos stalled."}
3. The 20 A fuse protects the wiring but will not blow on a servo fault. A smaller fuse on the 24 V input
   (5 A) would protect better and is worth requesting.
4. The steppers hold full current all the time, about {stepper_typ:.0f} W of continuous heat in three motors.
   Lower the driver current on the base axis, which carries no gravity load, and check motor temperature in the build phase.

## Limits of this experiment

* A budget from typical values, not a measurement.
* Inrush and servo start-up spikes are not modelled; a 1000 uF capacitor across the servo rail is the usual precaution.
"""
    write_report("E5_power_budget.md", report)


if __name__ == "__main__":
    main()
