"""Single source of truth for the arm: geometry, masses, drives and task layout.

The values a CAD or datasheet update changes are read from `arm_config.json`
(edited through the dashboard: `uv run python -m dashboard`), with defaults in
`arm/config.py`. Until real values are entered they are PLACEHOLDERS.
This module turns them into SI quantities and derived objects.

Conventions
-----------
* SI units (m, kg, s, rad) unless a name ends in `_deg`.
* Base frame: origin on the table at the base axis, z up, x towards the pick spot.
* Zero pose: arm pointing straight up.
* Joint chain: J1 yaw (z) - J2 shoulder pitch (y) - J3 elbow pitch (y)
               - J4 forearm roll (z) - J5 wrist pitch (y) - J6 tool roll (z).
  J4/J5/J6 axes intersect at the wrist centre (spherical wrist), which gives a
  closed-form inverse kinematics.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import JOINT_NAMES, load

CFG = load()
_geo, _mot, _drv, _lay = CFG["geometry_mm"], CFG["motors"], CFG["drives"], CFG["layout_mm"]

G = 9.81
SUPPLY_VOLTAGE = 24.0  # spec book: 24 V 12.5 A supply

# --------------------------------------------------------------------------
# Geometry (ASSUMED - to be replaced by CAD)
# --------------------------------------------------------------------------
D1 = _geo["D1"] / 1e3    # table -> shoulder axis height
L2 = _geo["L2"] / 1e3    # shoulder axis -> elbow axis (upper arm)
L3 = _geo["L3"] / 1e3    # elbow axis -> wrist centre (forearm)
L3A = _geo["L3A"] / 1e3  # elbow axis -> forearm roll joint plane
D6 = _geo["D6"] / 1e3    # wrist centre -> tool centre point (between the fingertips)


@dataclass(frozen=True)
class Joint:
    name: str
    axis: str            # rotation axis in the joint frame: "y" or "z"
    offset: tuple        # joint origin expressed in the parent link frame
    q_min_deg: float
    q_max_deg: float
    vmax_deg: float      # planned speed limit      (ASSUMED)
    amax_deg: float      # planned acceleration limit (ASSUMED)
    actuator: str        # "stepper" or "servo"


_AXES = ["z", "y", "y", "z", "y", "z"]
_OFFSETS = [(0, 0, 0), (0, 0, D1), (0, 0, L2), (0, 0, L3A), (0, 0, L3 - L3A), (0, 0, 0)]
_ACTUATORS = ["stepper"] * 3 + ["servo"] * 3
JOINTS = [Joint(JOINT_NAMES[i], _AXES[i], _OFFSETS[i], j["q_min_deg"], j["q_max_deg"], j["vmax_deg"],
                j["amax_deg"], _ACTUATORS[i]) for i, j in enumerate(CFG["joints"])]
N = len(JOINTS)
Q_MIN = np.radians([j.q_min_deg for j in JOINTS])
Q_MAX = np.radians([j.q_max_deg for j in JOINTS])
V_MAX = np.radians([j.vmax_deg for j in JOINTS])
A_MAX = np.radians([j.amax_deg for j in JOINTS])
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}

LIN_V_MAX = 0.15  # straight-line TCP speed (m/s)      (ASSUMED)
LIN_A_MAX = 0.50  # straight-line TCP acceleration (m/s^2) (ASSUMED)


# --------------------------------------------------------------------------
# Mass budget (ASSUMED - printed part masses are guesses, motor/servo masses
# are typical catalogue values). `link` is the joint number the item moves with.
# `pos` is in that link's frame (origin on the joint axis, z along the link).
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class MassItem:
    name: str
    link: int
    mass: float
    pos: tuple
    finger: bool = False  # modelled as separate sliding bodies in MuJoCo


MASSES = [MassItem(m["name"], m["link"], m["mass_g"] / 1e3, tuple(v / 1e3 for v in m["pos_mm"]), m["finger"])
          for m in CFG["masses"]]
PAYLOAD_MASS = CFG["payload"]["mass_g"] / 1e3  # design payload
PAYLOAD_POS = (0, 0, D6)


# --------------------------------------------------------------------------
# Actuators. Stepper torque of the plain NEMA17 is the spec-book figure;
# everything else is a typical datasheet value and must be VERIFIED against
# the datasheets of the parts actually delivered.
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Stepper:
    name: str
    hold_torque: float      # N.m at the motor shaft
    rated_current: float    # A per phase
    resistance: float       # ohm per phase
    inductance: float       # H per phase
    rotor_inertia: float    # kg.m^2
    steps_per_rev: int = 200


@dataclass(frozen=True)
class Servo:
    name: str
    stall_torque: float     # N.m
    no_load_speed: float    # rad/s
    stall_current: float    # A  (VERIFY)
    idle_current: float     # A  (VERIFY)


_stepper = (_mot["stepper_current_A"], _mot["stepper_resistance_ohm"], _mot["stepper_inductance_mH"] / 1e3, 5.4e-6)
NEMA17 = Stepper("17HS4401", _mot["nema17_hold_Nm"], *_stepper)
NEMA17_S = Stepper("17HS4401S", _mot["nema17_geared_hold_Nm"], *_stepper)
DS3218 = Servo("DS3218MG", _mot["ds3218_kgcm"] * 0.0980665, np.radians(60) / 0.16, _mot["servo_stall_current_A"], 0.01)
MG996 = Servo("MG996", _mot["mg996_kgcm"] * 0.0980665, np.radians(60) / 0.17, _mot["servo_stall_current_A"], 0.01)

STEPPER_DYNAMIC_FACTOR = 0.70  # usable pull-out torque / holding torque at low speed (ASSUMED)
MIN_STATIC_SF = 2.5            # design rule: holding torque >= 2.5 x worst static load


@dataclass(frozen=True)
class Drive:
    joint: int
    motor: object
    ratio: float            # total reduction motor -> joint
    efficiency: float
    description: str
    microstep: int = 8      # TMC2209 stand-alone options: 8, 16, 32, 64

    @property
    def is_stepper(self) -> bool:
        return isinstance(self.motor, Stepper)

    @property
    def hold_torque(self) -> float:
        """Holding / stall torque available at the joint (N.m)."""
        t = self.motor.hold_torque if self.is_stepper else self.motor.stall_torque
        return t * self.ratio * self.efficiency

    @property
    def steps_per_rad(self) -> float:
        return self.motor.steps_per_rev * self.microstep * self.ratio / (2 * np.pi)

    @property
    def reflected_inertia(self) -> float:
        return self.motor.rotor_inertia * self.ratio**2 if self.is_stepper else 2e-3

    def corner_speed(self) -> float:
        """Joint speed (rad/s) above which stepper torque falls (voltage limit)."""
        m = self.motor
        rev_s = SUPPLY_VOLTAGE / (2 * m.inductance * m.rated_current * m.steps_per_rev)
        return 2 * np.pi * rev_s / self.ratio

    def available_torque(self, joint_speed):
        """Usable torque at the joint versus joint speed (rad/s)."""
        w = np.abs(np.asarray(joint_speed, dtype=float))
        if self.is_stepper:
            wc = self.corner_speed()
            derate = np.minimum(1.0, wc / np.maximum(w, 1e-9))
            return STEPPER_DYNAMIC_FACTOR * self.hold_torque * derate
        w0 = self.motor.no_load_speed
        return self.hold_torque * np.clip(1 - w / w0, 0, 1)


MICROSTEP = int(_drv["microstep"])
GEARBOX = _mot["gearbox_ratio"]      # PG518 planetary
GEARBOX_EFF = _mot["gearbox_eff"]
BELT_EFF = _mot["belt_eff"]
PULLEY_TEETH = 16                   # provided GT2 pulleys
SHOULDER_TEETH = int(round(_drv["shoulder_teeth"]))  # printed GT2 pulley on the shoulder axis (see E2)
ELBOW_TEETH = int(round(_drv["elbow_teeth"]))        # printed GT2 pulley on the elbow axis    (see E2)
BASE_GEAR = _drv["base_gear_ratio"]      # printed gear stage on the base          (see E2)

DRIVES = [
    Drive(1, NEMA17, BASE_GEAR, 0.90,
          f"17HS4401 + printed {BASE_GEAR:g}:1 gear stage", MICROSTEP),
    Drive(2, NEMA17_S, GEARBOX * SHOULDER_TEETH / PULLEY_TEETH, GEARBOX_EFF * BELT_EFF,
          f"17HS4401S + PG518 {GEARBOX:g}:1 + GT2 {PULLEY_TEETH}:{SHOULDER_TEETH:g}", MICROSTEP),
    Drive(3, NEMA17, ELBOW_TEETH / PULLEY_TEETH, BELT_EFF,
          f"17HS4401 + GT2 {PULLEY_TEETH}:{ELBOW_TEETH:g}", MICROSTEP),
    Drive(4, DS3218, 1.0, 1.0, "DS3218MG direct"),
    Drive(5, DS3218, 1.0, 1.0, "DS3218MG direct"),
    Drive(6, MG996, 1.0, 1.0, "MG996 direct"),
]
GRIPPER_SERVO = MG996

# --------------------------------------------------------------------------
# Task layout (ASSUMED - positions of the pick spot and the two bins)
# --------------------------------------------------------------------------
OBJECT_SIZE = CFG["payload"]["object_size_mm"] / 1e3   # cube edge
PICK_XY = (_lay["pick_x"] / 1e3, _lay["pick_y"] / 1e3)   # fixed pick location (colour sensor underneath)
BINS = {"red": (_lay["bin_red_x"] / 1e3, _lay["bin_red_y"] / 1e3),
        "blue": (_lay["bin_blue_x"] / 1e3, _lay["bin_blue_y"] / 1e3)}
BIN_INNER = _lay["bin_inner"] / 1e3
BIN_WALL_H = _lay["bin_wall_h"] / 1e3
BIN_WALL_T = 0.004
Z_GRASP = OBJECT_SIZE / 2           # TCP height when gripping
Z_SAFE = _lay["z_safe"] / 1e3       # travel height
Z_DROP = _lay["z_drop"] / 1e3       # TCP height when releasing above a bin
READY_Q_DEG = tuple(CFG["ready_q_deg"])  # tool pointing down above the work area
GRIP_TIME = 0.40                    # s to close / open the gripper
