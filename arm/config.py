"""Editable design values: defaults, the team's saved file, validation.

The values a CAD or datasheet update changes live in `arm_config.json` at the repo root
(written by the dashboard, or by hand). Anything missing from that file falls back to
DEFAULTS below. Units are the ones people read off CAD: mm, g, deg.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent.parent / "arm_config.json"
JOINT_NAMES = ["J1 base yaw", "J2 shoulder", "J3 elbow", "J4 forearm roll", "J5 wrist pitch", "J6 tool roll"]

DEFAULTS = {
    "geometry_mm": {"D1": 120, "L2": 200, "L3": 200, "L3A": 80, "D6": 120},
    "joints": [
        {"q_min_deg": -170, "q_max_deg": 170, "vmax_deg": 90, "amax_deg": 150},
        {"q_min_deg": -100, "q_max_deg": 100, "vmax_deg": 45, "amax_deg": 150},
        {"q_min_deg": -140, "q_max_deg": 140, "vmax_deg": 90, "amax_deg": 150},
        {"q_min_deg": -90, "q_max_deg": 90, "vmax_deg": 200, "amax_deg": 300},
        {"q_min_deg": -90, "q_max_deg": 90, "vmax_deg": 200, "amax_deg": 300},
        {"q_min_deg": -90, "q_max_deg": 90, "vmax_deg": 200, "amax_deg": 300},
    ],
    "masses": [
        {"name": "turret structure", "link": 1, "mass_g": 400, "pos_mm": [0, 0, 60], "finger": False},
        {"name": "shoulder motor + gearbox", "link": 1, "mass_g": 500, "pos_mm": [0, -50, 120], "finger": False},
        {"name": "upper-arm structure", "link": 2, "mass_g": 200, "pos_mm": [0, 0, 100], "finger": False},
        {"name": "elbow motor (behind shoulder)", "link": 2, "mass_g": 280, "pos_mm": [0, 0, -30], "finger": False},
        {"name": "elbow pulley + bearings", "link": 2, "mass_g": 50, "pos_mm": [0, 0, 200], "finger": False},
        {"name": "forearm structure (fixed)", "link": 3, "mass_g": 100, "pos_mm": [0, 0, 40], "finger": False},
        {"name": "J4 servo DS3218MG", "link": 3, "mass_g": 60, "pos_mm": [0, 0, 60], "finger": False},
        {"name": "forearm structure (rolling)", "link": 4, "mass_g": 80, "pos_mm": [0, 0, 60], "finger": False},
        {"name": "J5 servo DS3218MG", "link": 4, "mass_g": 60, "pos_mm": [0, 0, 100], "finger": False},
        {"name": "wrist bracket", "link": 5, "mass_g": 40, "pos_mm": [0, 0, 20], "finger": False},
        {"name": "J6 servo MG996", "link": 5, "mass_g": 55, "pos_mm": [0, 0, 35], "finger": False},
        {"name": "gripper body", "link": 6, "mass_g": 70, "pos_mm": [0, 0, 70], "finger": False},
        {"name": "gripper servo MG996", "link": 6, "mass_g": 55, "pos_mm": [0, 0, 60], "finger": False},
        {"name": "fingers (pair)", "link": 6, "mass_g": 20, "pos_mm": [0, 0, 105], "finger": True},
    ],
    "payload": {"mass_g": 50, "object_size_mm": 30},
    "motors": {
        "nema17_hold_Nm": 0.49, "nema17_geared_hold_Nm": 0.42, "stepper_current_A": 1.7,
        "stepper_resistance_ohm": 1.5, "stepper_inductance_mH": 2.8,
        "ds3218_kgcm": 20, "mg996_kgcm": 13, "servo_stall_current_A": 2.5,
        "gearbox_ratio": 5.18, "gearbox_eff": 0.90, "belt_eff": 0.95,
    },
    "drives": {"shoulder_teeth": 64, "elbow_teeth": 96, "base_gear_ratio": 5.0, "microstep": 8},
    "layout_mm": {
        "pick_x": 280, "pick_y": 0, "bin_red_x": 100, "bin_red_y": 260, "bin_blue_x": 100, "bin_blue_y": -260,
        "bin_inner": 100, "bin_wall_h": 50, "z_safe": 120, "z_drop": 85,
    },
    "ready_q_deg": [0, 10, 90, 0, 80, 0],
}

# label and unit for every scalar field, used by the dashboard
LABELS = {
    "geometry_mm": {
        "D1": ("Table to shoulder axis", "mm"), "L2": ("Upper arm, shoulder to elbow", "mm"),
        "L3": ("Forearm, elbow to wrist centre", "mm"), "L3A": ("Elbow to forearm-roll joint", "mm"),
        "D6": ("Wrist centre to tool tip", "mm"),
    },
    "payload": {"mass_g": ("Object mass", "g"), "object_size_mm": ("Object size (cube edge)", "mm")},
    "motors": {
        "nema17_hold_Nm": ("Plain NEMA17 holding torque", "N.m"),
        "nema17_geared_hold_Nm": ("Geared NEMA17 holding torque (before gearbox)", "N.m"),
        "stepper_current_A": ("Stepper rated current", "A"), "stepper_resistance_ohm": ("Stepper phase resistance", "ohm"),
        "stepper_inductance_mH": ("Stepper phase inductance", "mH"),
        "ds3218_kgcm": ("DS3218MG stall torque", "kg.cm"), "mg996_kgcm": ("MG996 stall torque", "kg.cm"),
        "servo_stall_current_A": ("Servo stall current", "A"),
        "gearbox_ratio": ("Planetary gearbox ratio", ":1"), "gearbox_eff": ("Gearbox efficiency", "0-1"),
        "belt_eff": ("Belt stage efficiency", "0-1"),
    },
    "drives": {
        "shoulder_teeth": ("Shoulder printed pulley", "teeth"), "elbow_teeth": ("Elbow printed pulley", "teeth"),
        "base_gear_ratio": ("Base printed gear ratio", ":1"), "microstep": ("Microstepping (8, 16, 32 or 64)", "1/n"),
    },
    "layout_mm": {
        "pick_x": ("Pick spot x", "mm"), "pick_y": ("Pick spot y", "mm"),
        "bin_red_x": ("Red bin centre x", "mm"), "bin_red_y": ("Red bin centre y", "mm"),
        "bin_blue_x": ("Blue bin centre x", "mm"), "bin_blue_y": ("Blue bin centre y", "mm"),
        "bin_inner": ("Bin inner width", "mm"), "bin_wall_h": ("Bin wall height", "mm"),
        "z_safe": ("Travel height of the tool tip", "mm"), "z_drop": ("Release height above a bin", "mm"),
    },
}
SECTION_TITLES = {"geometry_mm": "Arm geometry", "payload": "Object", "motors": "Motors and efficiencies",
                  "drives": "Reductions", "layout_mm": "Cell layout"}


def _num(v, what):
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{what}: '{v}' is not a number") from None
    if f != f or f in (float("inf"), float("-inf")):
        raise ValueError(f"{what}: not a finite number")
    return f


def normalise(raw: dict | None) -> dict:
    """Merge `raw` over the defaults, coerce types, and reject values that cannot work."""
    raw = raw or {}
    cfg = copy.deepcopy(DEFAULTS)
    for section in LABELS:
        for key in cfg[section]:
            if key in raw.get(section, {}):
                cfg[section][key] = _num(raw[section][key], LABELS[section][key][0])
    if "joints" in raw:
        if len(raw["joints"]) != 6:
            raise ValueError("joints: exactly six joints are required")
        for i, j in enumerate(raw["joints"]):
            for key in cfg["joints"][i]:
                if key in j:
                    cfg["joints"][i][key] = _num(j[key], f"{JOINT_NAMES[i]} {key}")
    if "masses" in raw:
        cfg["masses"] = []
        for k, m in enumerate(raw["masses"]):
            name = str(m.get("name", "")).strip() or f"part {k + 1}"
            link = int(_num(m.get("link", 0), f"{name}: joint"))
            pos = m.get("pos_mm", [0, 0, 0])
            if link < 1 or link > 6:
                raise ValueError(f"{name}: 'moves with' must be J1 to J6")
            if len(pos) != 3:
                raise ValueError(f"{name}: position needs x, y and z")
            cfg["masses"].append({"name": name[:60], "link": link, "mass_g": _num(m.get("mass_g"), f"{name}: mass"),
                                  "pos_mm": [_num(p, f"{name}: position") for p in pos], "finger": bool(m.get("finger"))})
    if "ready_q_deg" in raw:
        if len(raw["ready_q_deg"]) != 6:
            raise ValueError("ready pose: six angles are required")
        cfg["ready_q_deg"] = [_num(v, "ready pose") for v in raw["ready_q_deg"]]

    g = cfg["geometry_mm"]
    for key, (label, _) in LABELS["geometry_mm"].items():
        if g[key] <= 0:
            raise ValueError(f"{label} must be positive")
    if not 0 < g["L3A"] < g["L3"]:
        raise ValueError("Elbow to forearm-roll joint must be shorter than the forearm")
    for i, j in enumerate(cfg["joints"]):
        if j["q_min_deg"] >= j["q_max_deg"]:
            raise ValueError(f"{JOINT_NAMES[i]}: lower limit must be below upper limit")
        if abs(j["q_min_deg"]) > 180 or abs(j["q_max_deg"]) > 180:
            raise ValueError(f"{JOINT_NAMES[i]}: limits must stay within -180 to 180 deg")
        if j["vmax_deg"] <= 0 or j["amax_deg"] <= 0:
            raise ValueError(f"{JOINT_NAMES[i]}: speed and acceleration must be positive")
    for link in range(1, 7):
        if not any(m["link"] == link and not m["finger"] and m["mass_g"] > 0 for m in cfg["masses"]):
            raise ValueError(f"Mass budget: {JOINT_NAMES[link - 1]} needs at least one part with mass")
    if sum(1 for m in cfg["masses"] if m["finger"]) != 1:
        raise ValueError("Mass budget: exactly one row must be ticked as the fingers")
    if any(m["mass_g"] <= 0 for m in cfg["masses"]):
        raise ValueError("Mass budget: every mass must be positive")
    if int(cfg["drives"]["microstep"]) not in (8, 16, 32, 64):
        raise ValueError("Microstepping must be 8, 16, 32 or 64")
    for key in ("shoulder_teeth", "elbow_teeth", "base_gear_ratio"):
        if cfg["drives"][key] <= 0:
            raise ValueError(f"{LABELS['drives'][key][0]} must be positive")
    for key in ("gearbox_eff", "belt_eff"):
        if not 0 < cfg["motors"][key] <= 1:
            raise ValueError(f"{LABELS['motors'][key][0]} must be between 0 and 1")
    for key, (label, _) in LABELS["motors"].items():
        if cfg["motors"][key] <= 0:
            raise ValueError(f"{label} must be positive")
    if cfg["payload"]["mass_g"] <= 0 or cfg["payload"]["object_size_mm"] <= 0:
        raise ValueError("Object mass and size must be positive")
    lay = cfg["layout_mm"]
    if lay["z_drop"] > lay["z_safe"]:
        raise ValueError("Release height must not be above the travel height")
    if lay["z_safe"] <= cfg["payload"]["object_size_mm"]:
        raise ValueError("Travel height must be above the object")
    return cfg


def load() -> dict:
    raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8")) if CONFIG_PATH.exists() else {}
    return normalise(raw)


def save(raw: dict) -> dict:
    cfg = normalise(raw)
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    return cfg


def reset() -> dict:
    CONFIG_PATH.unlink(missing_ok=True)
    return normalise({})


def config_hash(cfg: dict | None = None) -> str:
    cfg = load() if cfg is None else cfg
    return hashlib.sha1(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:12]
