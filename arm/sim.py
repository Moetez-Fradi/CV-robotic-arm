"""Run the planned sort cycle in MuJoCo and log what the actuators have to deliver.

Controller: computed-torque tracking of the planned joint trajectory,
    tau = M(q) (qdd_ref + Kp e + Kd de) + bias(q, qd)
clipped to each drive's holding torque. The logged torque is therefore the torque the
real drive must produce to follow the plan, including gravity, inertia and the payload.

Grasping is not simulated as finger friction: at the grip event the object is welded
to the hand (if the tool tip is close enough to it) and released at the drop event.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from . import params as P
from .mjcf import FINGER_CLOSED, build_xml
from .task import DT, Trajectory

KP, KD = 2500.0, 100.0      # critically damped, 50 rad/s
GRASP_TOLERANCE = 0.008     # tool tip must be within 8 mm of the object centre to grip
DISTAL = ["arm_fore_b", "arm_wrist_hub", "arm_wrist", "arm_hand", "arm_finger_l", "arm_finger_r"]


@dataclass
class SimLog:
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    q_ref: np.ndarray
    tau: np.ndarray
    tcp: np.ndarray
    tcp_ref: np.ndarray
    clear_table: np.ndarray
    clear_bins: np.ndarray
    objects: np.ndarray              # final object positions
    placed: list                     # per object: landed inside its bin?
    grasp_error: list                # tool-tip to object distance at each grip (m)
    saturated: bool
    frames: list = field(default_factory=list)
    stills: dict = field(default_factory=dict)


def _gid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)


def run_cycle(traj: Trajectory, colours=("red", "blue"), payload_mass: float = P.PAYLOAD_MASS,
              render: bool = True, fps: int = 30, size=(540, 960), camera: str = "iso",
              log_every: int = 5, frame_cb=None) -> SimLog:
    from .kinematics import fk

    xml = build_xml(colours).replace(f'mass="{P.PAYLOAD_MASS}"', f'mass="{payload_mass}"')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    assert abs(m.opt.timestep - DT) < 1e-12
    d.qpos[:P.N] = traj.q[0]
    mujoco.mj_forward(m, d)

    hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link6")
    tcp_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    obj_body = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"obj{i}") for i in range(len(colours))]
    obj_qadr = [m.jnt_qposadr[m.body_jntadr[b]] for b in obj_body]
    obj_vadr = [m.jnt_dofadr[m.body_jntadr[b]] for b in obj_body]
    distal = [_gid(m, n) for n in DISTAL]
    table = _gid(m, "table")
    bins = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("bin_")]
    limit = np.array([dr.hold_torque for dr in P.DRIVES])

    renderer = mujoco.Renderer(m, *size) if render else None
    frame_every = int(round(1 / fps / DT))
    events = sorted(traj.events, key=lambda e: e[0])
    ev = 0
    held = None
    acc = np.zeros(m.nv)
    mv = np.zeros(m.nv)
    ft = np.zeros(6)
    rows = {k: [] for k in ("t", "q", "qd", "q_ref", "tau", "tcp", "tcp_ref", "ct", "cb")}
    frames, stills, grasp_error = [], {}, []
    saturated = False

    def snapshot(label):
        if renderer is not None:
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera=camera)
            stills[label] = renderer.render().copy()

    for k in range(len(traj.t)):
        t = k * DT
        while ev < len(events) and events[ev][0] <= t + 1e-9:
            _, kind, i = events[ev]
            ev += 1
            if kind == "present":
                a, v = obj_qadr[i], obj_vadr[i]
                d.qpos[a:a + 7] = [*P.PICK_XY, P.OBJECT_SIZE / 2, 1, 0, 0, 0]
                d.qvel[v:v + 6] = 0
                mujoco.mj_forward(m, d)
            elif kind == "attach":
                err = float(np.linalg.norm(d.site_xpos[tcp_site] - d.xpos[obj_body[i]]))
                grasp_error.append(err)
                if err <= GRASP_TOLERANCE:
                    neg_p, neg_q, rel_p, rel_q = np.zeros(3), np.zeros(4), np.zeros(3), np.zeros(4)
                    mujoco.mju_negPose(neg_p, neg_q, d.xpos[hand], d.xquat[hand])
                    mujoco.mju_mulPose(rel_p, rel_q, neg_p, neg_q, d.xpos[obj_body[i]], d.xquat[obj_body[i]])
                    m.eq_data[i, 0:3] = 0
                    m.eq_data[i, 3:6] = rel_p
                    m.eq_data[i, 6:10] = rel_q
                    d.eq_active[i] = 1
                    held = i
                snapshot(f"grip object {i + 1}")
            elif kind == "detach":
                snapshot(f"above {colours[i]} bin")
                d.eq_active[i] = 0
                held = None
            elif kind == "read colour":
                snapshot(f"read colour {i + 1}")

        mujoco.mj_step1(m, d)
        e = traj.q[k] - d.qpos[:P.N]
        de = traj.qd[k] - d.qvel[:P.N]
        acc[:] = 0
        acc[:P.N] = traj.qdd[k] + KP * e + KD * de
        mujoco.mj_mulM(m, d, mv, acc)
        tau_cmd = mv[:P.N] + d.qfrc_bias[:P.N]
        tau = np.clip(tau_cmd, -limit, limit)
        saturated |= bool(np.any(np.abs(tau_cmd) > limit))
        d.ctrl[:P.N] = tau
        d.ctrl[P.N:P.N + 2] = traj.grip[k] * FINGER_CLOSED
        mujoco.mj_step2(m, d)

        if k % log_every == 0:
            rows["t"].append(t)
            rows["q"].append(d.qpos[:P.N].copy())
            rows["qd"].append(d.qvel[:P.N].copy())
            rows["q_ref"].append(traj.q[k])
            rows["tau"].append(tau.copy())
            rows["tcp"].append(d.site_xpos[tcp_site].copy())
            rows["tcp_ref"].append(fk(traj.q[k])[:3, 3])
            rows["ct"].append(min(mujoco.mj_geomDistance(m, d, g, table, 1.0, ft) for g in distal))
            movers = distal + ([_gid(m, f"obj{held}")] if held is not None else [])
            rows["cb"].append(min(mujoco.mj_geomDistance(m, d, g, b, 1.0, ft) for g in movers for b in bins))
        if renderer is not None and k % frame_every == 0:
            renderer.update_scene(d, camera=camera)
            frame = renderer.render().copy()
            if frame_cb is not None:
                frame_cb(t, frame)       # stream the frame instead of keeping it in memory
            else:
                frames.append(frame)

    for _ in range(500):                      # let the last object settle
        mujoco.mj_step(m, d)
    snapshot("final")
    half = P.BIN_INNER / 2
    placed, final = [], []
    for i, colour in enumerate(colours):
        p = d.xpos[obj_body[i]].copy()
        final.append(p)
        bx, by = P.BINS[colour]
        placed.append(bool(abs(p[0] - bx) < half and abs(p[1] - by) < half and p[2] < P.BIN_WALL_H))
    if renderer is not None:
        renderer.close()

    return SimLog(np.array(rows["t"]), np.array(rows["q"]), np.array(rows["qd"]), np.array(rows["q_ref"]),
                  np.array(rows["tau"]), np.array(rows["tcp"]), np.array(rows["tcp_ref"]),
                  np.array(rows["ct"]), np.array(rows["cb"]), np.array(final), placed, grasp_error,
                  saturated, frames, stills)
