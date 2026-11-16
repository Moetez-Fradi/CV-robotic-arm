"""Print a JSON description of the arm model for the dashboard's Model page.

Run:  uv run python -m arm.describe
Reads the saved design values (arm_config.json), so it always matches the experiments.
"""
from __future__ import annotations

import json

import numpy as np

from . import params as P


def describe() -> dict:
    joints = []
    for i, (j, d) in enumerate(zip(P.JOINTS, P.DRIVES), start=1):
        on_link = [m for m in P.MASSES if m.link == i]
        carried = sum(m.mass for m in P.MASSES if m.link >= i) + P.PAYLOAD_MASS
        joints.append({
            "index": i, "name": j.name, "axis": j.axis, "offset_mm": [round(v * 1e3, 3) for v in j.offset],
            "q_min_deg": j.q_min_deg, "q_max_deg": j.q_max_deg, "vmax_deg": j.vmax_deg, "amax_deg": j.amax_deg,
            "actuator": j.actuator, "drive": d.description, "ratio": round(d.ratio, 3),
            "hold_torque_Nm": round(d.hold_torque, 3), "motor": d.motor.name,
            "steps_per_deg": round(d.steps_per_rad * np.pi / 180, 2) if d.is_stepper else None,
            "microstep": d.microstep if d.is_stepper else None,
            "parts": [{"name": m.name, "mass_g": round(m.mass * 1e3, 1)} for m in on_link],
            "link_mass_g": round(sum(m.mass for m in on_link) * 1e3, 1),
            "carried_mass_g": round(carried * 1e3, 1),
        })
    poses = {}
    try:
        from .task import key_poses
        poses = {k: np.degrees(v).round(2).tolist() for k, v in key_poses().items()}
    except ValueError as exc:       # layout unreachable: the page still shows the arm
        poses = {}
        pose_error = str(exc)
    else:
        pose_error = None
    return {
        "joints": joints, "tcp_offset_mm": P.D6 * 1e3,
        "geometry_mm": {"D1": P.D1 * 1e3, "L2": P.L2 * 1e3, "L3": P.L3 * 1e3, "L3A": P.L3A * 1e3, "D6": P.D6 * 1e3},
        "ready_deg": list(P.READY_Q_DEG), "poses": poses, "pose_error": pose_error,
        "payload_g": P.PAYLOAD_MASS * 1e3, "total_g": sum(m.mass for m in P.MASSES) * 1e3 + P.PAYLOAD_MASS * 1e3,
        "layout_mm": {"pick": [P.PICK_XY[0] * 1e3, P.PICK_XY[1] * 1e3],
                      "bins": {k: [v[0] * 1e3, v[1] * 1e3] for k, v in P.BINS.items()}, "bin_inner": P.BIN_INNER * 1e3},
    }


if __name__ == "__main__":
    print(json.dumps(describe()))
