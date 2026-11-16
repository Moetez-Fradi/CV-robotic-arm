"""Static load model: gravity torque at every joint from the mass budget."""
from __future__ import annotations

import numpy as np

from .kinematics import fk_frames
from .params import AXIS_INDEX, G, JOINTS, MASSES, PAYLOAD_MASS, PAYLOAD_POS


def point_masses(payload: float = PAYLOAD_MASS, mass_scale: float = 1.0):
    """(link, mass, position-in-link-frame) for every mass item plus the payload."""
    items = [(m.link, m.mass * mass_scale, m.pos) for m in MASSES]
    if payload > 0:
        items.append((len(JOINTS), payload, PAYLOAD_POS))
    return items


def gravity_torque(q, payload: float = PAYLOAD_MASS, mass_scale: float = 1.0) -> np.ndarray:
    """Torque (N.m) each joint must supply to hold pose q against gravity."""
    frames = fk_frames(q)
    axes = [T[:3, AXIS_INDEX[j.axis]] for j, T in zip(JOINTS, frames)]
    origins = [T[:3, 3] for T in frames]
    tau = np.zeros(len(JOINTS))
    for link, mass, pos in point_masses(payload, mass_scale):
        c = (frames[link - 1] @ np.array([*pos, 1.0]))[:3]
        for j in range(link):
            tau[j] += mass * G * np.cross(axes[j], c - origins[j])[2]
    return tau


def link_inertials():
    """Per link: (mass, centre of mass, 3x3 inertia about the COM), fingers excluded."""
    out = []
    for link in range(1, len(JOINTS) + 1):
        items = [m for m in MASSES if m.link == link and not m.finger]
        mass = sum(m.mass for m in items)
        com = sum(m.mass * np.array(m.pos, dtype=float) for m in items) / mass
        inertia = np.zeros((3, 3))
        for m in items:
            d = np.array(m.pos, dtype=float) - com
            inertia += m.mass * (d @ d * np.eye(3) - np.outer(d, d))
            inertia += 0.4 * m.mass * 0.025**2 * np.eye(3)   # each item ~ 25 mm sphere
        out.append((mass, com, inertia))
    return out


def moving_mass(payload: float = PAYLOAD_MASS) -> float:
    return sum(m for _, m, _ in point_masses(payload))
