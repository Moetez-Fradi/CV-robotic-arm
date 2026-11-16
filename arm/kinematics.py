"""Forward kinematics, closed-form inverse kinematics and Jacobian."""
from __future__ import annotations

import numpy as np

from .params import AXIS_INDEX, D1, D6, JOINTS, L2, L3, Q_MAX, Q_MIN


def rot(axis: str, q: float) -> np.ndarray:
    c, s = np.cos(q), np.sin(q)
    T = np.eye(4)
    if axis == "z":
        T[:2, :2] = [[c, -s], [s, c]]
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c
    return T


def trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = v
    return T


def fk_frames(q) -> list[np.ndarray]:
    """World pose of every link frame (frame i moves with joint i)."""
    T = np.eye(4)
    frames = []
    for joint, qi in zip(JOINTS, q):
        T = T @ trans(joint.offset) @ rot(joint.axis, qi)
        frames.append(T)
    return frames


def fk(q) -> np.ndarray:
    """World pose of the tool centre point."""
    return fk_frames(q)[-1] @ trans((0, 0, D6))


def jacobian(q) -> np.ndarray:
    """6x6 geometric Jacobian of the TCP (linear rows first)."""
    frames = fk_frames(q)
    p = (frames[-1] @ trans((0, 0, D6)))[:3, 3]
    J = np.zeros((6, len(JOINTS)))
    for i, (joint, T) in enumerate(zip(JOINTS, frames)):
        a = T[:3, AXIS_INDEX[joint.axis]]
        J[:3, i] = np.cross(a, p - T[:3, 3])
        J[3:, i] = a
    return J


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def within_limits(q, tol: float = 1e-9) -> bool:
    q = np.asarray(q)
    return bool(np.all(q >= Q_MIN - tol) and np.all(q <= Q_MAX + tol))


def tool_down(x: float, y: float, z: float, yaw=None) -> np.ndarray:
    """TCP pose at (x, y, z) with the tool pointing straight down."""
    yaw = np.arctan2(y, x) if yaw is None else yaw
    T = rot("z", yaw) @ rot("y", np.pi)
    T[:3, 3] = (x, y, z)
    return T


def ik(T: np.ndarray, q_ref=None, respect_limits: bool = True) -> list[np.ndarray]:
    """All closed-form IK solutions for TCP pose T, nearest to q_ref first.

    Up to 8 solutions: base front/back x elbow up/down x wrist flip.
    """
    R, p = T[:3, :3], T[:3, 3]
    q_ref = np.zeros(6) if q_ref is None else np.asarray(q_ref, dtype=float)
    pw = p - D6 * R[:, 2]                       # wrist centre
    base = np.arctan2(pw[1], pw[0]) if np.hypot(pw[0], pw[1]) > 1e-9 else q_ref[0]
    sols = []
    for q1 in (base, base + np.pi):
        r = pw[0] * np.cos(q1) + pw[1] * np.sin(q1)   # signed radial distance
        s = pw[2] - D1
        c3 = (r * r + s * s - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(c3) > 1 + 1e-12:
            continue
        a3 = np.arccos(np.clip(c3, -1, 1))
        for q3 in (a3, -a3):
            q2 = np.arctan2(r, s) - np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))
            R03 = (rot("z", q1) @ rot("y", q2 + q3))[:3, :3]
            M = R03.T @ R                         # = Rz(q4) Ry(q5) Rz(q6)
            s5 = np.hypot(M[0, 2], M[1, 2])
            if s5 < 1e-9:                         # wrist singularity: q4 and q6 collinear
                q4 = q_ref[3]
                if M[2, 2] > 0:
                    cands = [(q4, 0.0, np.arctan2(M[1, 0], M[0, 0]) - q4)]
                else:
                    cands = [(q4, np.pi, np.arctan2(M[1, 0], M[1, 1]) + q4)]
            else:
                cands = []
                for sg in (1.0, -1.0):
                    cands.append((np.arctan2(sg * M[1, 2], sg * M[0, 2]),
                                  np.arctan2(sg * s5, M[2, 2]),
                                  np.arctan2(sg * M[2, 1], -sg * M[2, 0])))
            for q4, q5, q6 in cands:
                q = wrap([q1, q2, q3, q4, q5, q6])
                if respect_limits and not within_limits(q):
                    continue
                sols.append(q)
    sols.sort(key=lambda q: float(np.sum(wrap(q - q_ref) ** 2)))
    return sols
