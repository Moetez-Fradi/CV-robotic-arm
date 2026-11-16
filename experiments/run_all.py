"""Check the models against each other, then regenerate experiments and reports.

Run everything:      uv run python -m experiments.run_all
Run a selection:     uv run python -m experiments.run_all e1 e4

Each experiment is run even if an earlier one fails. The outcome of every run is
recorded in reports/data/run_meta.json so the dashboard can show what is up to date.
"""
from __future__ import annotations

import importlib
import sys
import time
import traceback
from datetime import datetime

import mujoco
import numpy as np

from arm import params as P
from arm.config import config_hash
from arm.dynamics import gravity_torque
from arm.kinematics import fk
from arm.mjcf import build_xml
from .common import load_json, save_json

MODULES = {
    "e1": "e1_kinematics_workspace",
    "e2": "e2_torque_gearing",
    "e3": "e3_motion_timing",
    "e4": "e4_pick_place_sim",
    "e5": "e5_power_budget",
}


def cross_check(n: int = 300) -> None:
    """The analytic model and the MuJoCo model must agree on geometry and gravity loads."""
    m = mujoco.MjModel.from_xml_string(build_xml(scene=False))
    d = mujoco.MjData(m)
    tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    rng = np.random.default_rng(0)
    e_pos = e_tau = 0.0
    for _ in range(n):
        q = rng.uniform(P.Q_MIN, P.Q_MAX)
        d.qpos[:P.N] = q
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        e_pos = max(e_pos, float(np.abs(d.site_xpos[tcp] - fk(q)[:3, 3]).max()))
        e_tau = max(e_tau, float(np.abs(d.qfrc_bias[:P.N] - gravity_torque(q, payload=0.0)).max()))
    print(f"cross-check on {n} poses: tool position {e_pos:.1e} m, gravity torque {e_tau:.1e} N.m", flush=True)
    assert e_pos < 1e-9 and e_tau < 1e-6, "analytic and MuJoCo models disagree"


def main(argv=None) -> int:
    names = [a.lower() for a in (sys.argv[1:] if argv is None else argv)] or list(MODULES)
    unknown = [n for n in names if n not in MODULES]
    if unknown:
        print(f"unknown experiment(s): {', '.join(unknown)}. Choose from {', '.join(MODULES)}.")
        return 2
    cfg = config_hash(P.CFG)
    meta = load_json("run_meta.json") or {}
    failed = 0
    try:
        cross_check()
        check_error = None
    except Exception as exc:  # noqa: BLE001 - reported per experiment below
        traceback.print_exc()
        check_error = f"model cross-check failed: {exc}"
    for name in names:
        t0 = time.time()
        print(f"[{name}] started", flush=True)
        entry = {"config": cfg, "time": datetime.now().isoformat(timespec="seconds")}
        try:
            if check_error:
                raise RuntimeError(check_error)
            importlib.import_module(f"experiments.{MODULES[name]}").main()
            entry["status"] = "ok"
        except Exception as exc:  # noqa: BLE001 - keep going, record the reason
            traceback.print_exc()
            entry["status"] = "failed"
            entry["error"] = f"{type(exc).__name__}: {exc}"
            failed += 1
        entry["seconds"] = round(time.time() - t0, 1)
        meta[name] = entry
        save_json("run_meta.json", meta)
        print(f"[{name}] {entry['status']} in {entry['seconds']:.0f} s", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
