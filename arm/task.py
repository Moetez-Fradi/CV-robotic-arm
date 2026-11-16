"""Motion planning for the pick-and-sort cycle.

Two move types, both using a trapezoidal velocity profile (1/3 accel, 1/3 cruise,
1/3 decel):
  * PTP - joint-space move, all joints start and stop together.
  * LIN - straight TCP line (used for the vertical approach / retreat).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import params as P
from .kinematics import fk, fk_frames, ik, tool_down

DT = 0.001


def profile(T: float, n: int):
    """Normalised trapezoid s(t) in [0, 1] sampled at n points; returns s, ds, dds."""
    t = np.arange(n) * DT
    ta = T / 3
    vm = 1.5 / T
    a = vm / ta
    s = np.where(t < ta, 0.5 * a * t**2,
                 np.where(t < 2 * ta, 0.5 * a * ta**2 + vm * (t - ta),
                          1 - 0.5 * a * np.clip(T - t, 0, None) ** 2))
    ds = np.where(t < ta, a * t, np.where(t < 2 * ta, vm, a * np.clip(T - t, 0, None)))
    dds = np.where(t < ta, a, np.where(t < 2 * ta, 0.0, np.where(t < T, -a, 0.0)))
    return s, ds, dds


def move_time(dist, vmax, amax, minimum: float = 0.25) -> float:
    dist = np.abs(np.atleast_1d(dist))
    return float(max(np.max(1.5 * dist / vmax), np.max(np.sqrt(4.5 * dist / amax)), minimum))


@dataclass
class Trajectory:
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    grip: np.ndarray                       # 0 = open, 1 = closed
    events: list = field(default_factory=list)     # (time, kind, object index)
    segments: list = field(default_factory=list)   # (name, t_start, t_end)

    @property
    def duration(self) -> float:
        return float(self.t[-1])


class _Builder:
    def __init__(self, q0, speed: float):
        self.q_now = np.asarray(q0, dtype=float)
        self.grip_now = 0.0
        self.speed = speed
        self.q, self.qd, self.qdd, self.grip = [], [], [], []
        self.events, self.segments = [], []
        self.n = 0

    def _push(self, name, q, qd, qdd, grip):
        self.segments.append((name, self.n * DT, (self.n + len(q)) * DT))
        self.q.append(q), self.qd.append(qd), self.qdd.append(qdd), self.grip.append(grip)
        self.n += len(q)
        self.q_now = q[-1].copy()
        self.grip_now = float(grip[-1])

    def ptp(self, name, q_goal):
        dq = np.asarray(q_goal) - self.q_now
        T = move_time(dq, P.V_MAX * self.speed, P.A_MAX * self.speed**2)
        n = int(np.ceil(T / DT)) + 1
        s, ds, dds = profile(T, n)
        self._push(name, self.q_now + np.outer(s, dq), np.outer(ds, dq), np.outer(dds, dq),
                   np.full(n, self.grip_now))

    def lin(self, name, p_goal):
        T0 = fk(self.q_now)
        p0 = T0[:3, 3].copy()
        dist = float(np.linalg.norm(np.asarray(p_goal) - p0))
        T = move_time(dist, P.LIN_V_MAX * self.speed, P.LIN_A_MAX * self.speed**2)
        for _ in range(4):
            n = int(np.ceil(T / DT)) + 1
            s, _, _ = profile(T, n)
            q = np.empty((n, P.N))
            ref = self.q_now
            for k in range(n):
                Tk = T0.copy()
                Tk[:3, 3] = p0 + s[k] * (np.asarray(p_goal) - p0)
                sols = ik(Tk, ref)
                if not sols:
                    raise ValueError(f"The straight-line move '{name}' leaves the workspace or the joint limits")
                ref = q[k] = sols[0]
            qd = np.gradient(q, DT, axis=0)
            qdd = np.gradient(qd, DT, axis=0)
            over = max(np.max(np.abs(qd) / (P.V_MAX * self.speed)),
                       np.sqrt(np.max(np.abs(qdd) / (P.A_MAX * self.speed**2))))
            if over <= 1.0:
                break
            T *= over * 1.03
        self._push(name, q, qd, qdd, np.full(n, self.grip_now))

    def grip_to(self, name, target, event=None):
        n = int(np.ceil(P.GRIP_TIME / self.speed / DT)) + 1
        g = np.linspace(self.grip_now, target, n)
        if event:
            # attach once the fingers have closed; release as soon as they start opening
            when = (self.n + n - 1) * DT if target > 0.5 else self.n * DT
            self.events.append((when, *event))
        zeros = np.zeros((n, P.N))
        self._push(name, np.tile(self.q_now, (n, 1)), zeros, zeros.copy(), g)

    def build(self) -> Trajectory:
        q = np.vstack(self.q)
        return Trajectory(np.arange(len(q)) * DT, q, np.vstack(self.qd), np.vstack(self.qdd),
                          np.concatenate(self.grip), self.events, self.segments)


def solve(T, q_ref) -> np.ndarray:
    sols = ik(T, q_ref)
    if not sols:
        x, y, z = T[:3, 3] * 1e3
        raise ValueError(f"The tool tip cannot reach x={x:.0f}, y={y:.0f}, z={z:.0f} mm with the tool pointing down "
                         f"(outside the workspace or the joint limits). Check the cell layout, link lengths and joint limits.")
    return sols[0]


EXERCISE_FRACTION = 0.75    # all-axes check drives each joint to this fraction of its range, both ways
EXERCISE_MIN_Z = 0.06       # ...but never lets a joint axis or the tool tip below this height (m)
BIN_WRIST_TURN = np.radians(60)   # tool roll applied at the bins (red +, blue -)


def exercise_targets(ready) -> list:
    """(joint index, low pose, high pose): each joint alone is taken towards both limits from the ready pose."""
    out = []
    for i in range(P.N):
        ends = []
        for limit in (P.Q_MIN[i], P.Q_MAX[i]):
            v = limit * EXERCISE_FRACTION
            for _ in range(60):
                q = np.array(ready, dtype=float)
                q[i] = v
                if min([T[2, 3] for T in fk_frames(q)[1:]] + [fk(q)[2, 3]]) >= EXERCISE_MIN_Z:
                    break
                v = ready[i] + 0.95 * (v - ready[i])        # shrink towards the ready pose
            ends.append(v)
        out.append((i, ends[0], ends[1]))
    return out


def key_poses() -> dict:
    """Named joint configurations used by the cycle (all with the tool pointing down)."""
    ready = np.radians(P.READY_Q_DEG)
    px, py = P.PICK_XY
    poses = {"ready": ready,
             "above pick": solve(tool_down(px, py, P.Z_SAFE), ready)}
    poses["pick"] = solve(tool_down(px, py, P.Z_GRASP), poses["above pick"])
    for colour, (bx, by) in P.BINS.items():
        turn = BIN_WRIST_TURN if colour == "red" else -BIN_WRIST_TURN   # turn the object over the bin
        poses[f"above {colour} bin"] = solve(tool_down(bx, by, P.Z_SAFE, yaw=np.arctan2(by, bx) + turn), poses["above pick"])
        poses[f"drop {colour} bin"] = solve(tool_down(bx, by, P.Z_DROP, yaw=np.arctan2(by, bx) + turn), poses[f"above {colour} bin"])
    return poses


def plan_cycle(colours=("red", "blue"), speed: float = 1.0, axes_check: bool = True) -> Trajectory:
    """All-axes check, then the sort cycle: for each object, pick it at the pick spot and place it in its bin.

    `speed` scales every velocity limit by `speed` and every acceleration limit by
    `speed**2`, so the whole trajectory is the nominal one stretched in time by 1/speed.
    """
    poses = key_poses()
    px, py = P.PICK_XY
    b = _Builder(poses["ready"], speed)
    if axes_check:      # spec book: "full movement in all axes and rotations"
        for j, lo, hi in exercise_targets(poses["ready"]):
            for label, v in (("low", lo), ("high", hi)):
                q = poses["ready"].copy()
                q[j] = v
                b.ptp(f"axes check J{j + 1} {label}", q)
            b.ptp(f"axes check J{j + 1} back", poses["ready"])
    for i, colour in enumerate(colours):
        bx, by = P.BINS[colour]
        b.events.append((b.n * DT, "present", i))          # object i appears at the pick spot
        b.ptp(f"{i + 1}: to above pick", poses["above pick"])
        b.events.append((b.n * DT, "read colour", i))
        b.lin(f"{i + 1}: descend", (px, py, P.Z_GRASP))
        b.grip_to(f"{i + 1}: grip", 1.0, ("attach", i))
        b.lin(f"{i + 1}: lift", (px, py, P.Z_SAFE))
        b.ptp(f"{i + 1}: to {colour} bin", poses[f"above {colour} bin"])
        b.lin(f"{i + 1}: lower", (bx, by, P.Z_DROP))
        b.grip_to(f"{i + 1}: release", 0.0, ("detach", i))
        b.lin(f"{i + 1}: retreat", (bx, by, P.Z_SAFE))
    b.ptp("return to ready", poses["ready"])
    b.grip_to("settle", 0.0)
    return b.build()
