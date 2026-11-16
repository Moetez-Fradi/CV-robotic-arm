"""Generate the MuJoCo model of the arm and its work cell from params.py.

The arm is drawn with simple primitives (no CAD meshes needed); masses, centres
of mass and joint axes come from the same parameter file as the analytic model.
"""
from __future__ import annotations

import numpy as np
from pathlib import Path

from . import params as P
from .dynamics import link_inertials

OBJECT_COLOURS = {"red": "0.85 0.15 0.15 1", "blue": "0.15 0.35 0.9 1"}
FINGER_OPEN = 0.0
FINGER_CLOSED = 0.009      # slide travel of each finger when gripping
STOCK_XY = (-0.22, 0.0)    # where objects wait before being presented at the pick spot


def _f(v) -> str:
    return " ".join(f"{x:.6g}" for x in np.atleast_1d(v))


def _bin(name: str, x: float, y: float, rgba: str) -> str:
    h, t, a = P.BIN_WALL_H, P.BIN_WALL_T, P.BIN_INNER / 2
    walls = [((a + t / 2, 0), (t / 2, a + t)), ((-a - t / 2, 0), (t / 2, a + t)),
             ((0, a + t / 2), (a + t, t / 2)), ((0, -a - t / 2), (a + t, t / 2))]
    out = [f'<body name="{name}" pos="{x} {y} 0">',
           f'<geom name="{name}_floor" class="solid" type="box" size="{a + t} {a + t} 0.002" '
           f'pos="0 0 0.002" rgba="{rgba}"/>']
    for i, ((wx, wy), (sx, sy)) in enumerate(walls):
        out.append(f'<geom name="{name}_wall{i}" class="solid" type="box" size="{sx} {sy} {h / 2}" '
                   f'pos="{wx} {wy} {h / 2}" rgba="{rgba}"/>')
    out.append("</body>")
    return "\n".join(out)


# fixed USB camera for the pick location (vision): offset from the pick spot, looking at it
VISION_CAM_OFFSET = (0.22, 0.0, 0.30)
VISION_CAM_FOVY = 45.0
VISION_CAM_SIZE = (1280, 720)


def vision_mat_texture() -> str:
    """Write the pick-mat image used as a texture in the simulation and return its path."""
    import cv2
    from vision import mat
    path = Path(__file__).resolve().parent.parent / "vision" / "assets" / "pick_mat_sim.png"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), cv2.cvtColor(mat.render(px_per_mm=4.0), cv2.COLOR_GRAY2BGR))
    return path.as_posix()


def build_xml(colours=("red", "blue"), scene: bool = True, vision: bool = False) -> str:
    inert = link_inertials()
    finger_mass = sum(m.mass for m in P.MASSES if m.finger) / 2
    finger_z = next(m.pos[2] for m in P.MASSES if m.finger)
    j = P.JOINTS
    grey, dark, accent = "0.82 0.82 0.85 1", "0.25 0.27 0.3 1", "0.95 0.55 0.1 1"

    def inertial(i):
        m, c, I = inert[i]
        return (f'<inertial pos="{_f(c)}" mass="{m:.6g}" fullinertia="{_f([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>')

    def joint(i):
        ax = {"y": "0 1 0", "z": "0 0 1"}[j[i].axis]
        arm = P.DRIVES[i].reflected_inertia
        return (f'<joint name="j{i + 1}" type="hinge" axis="{ax}" range="{_f([P.Q_MIN[i], P.Q_MAX[i]])}" '
                f'armature="{arm:.6g}" damping="0.02"/>')

    arm = f"""
    <body name="link1" pos="{_f(j[0].offset)}">
      {inertial(0)} {joint(0)}
      <geom name="arm_turret" type="cylinder" fromto="0 0 0.035 0 0 {P.D1 - 0.02}" size="0.045" rgba="{grey}"/>
      <geom name="arm_shoulder_motor" type="box" pos="0 -0.05 {P.D1}" size="0.021 0.03 0.021" rgba="{dark}"/>
      <body name="link2" pos="{_f(j[1].offset)}">
        {inertial(1)} {joint(1)}
        <geom name="arm_shoulder_hub" type="cylinder" fromto="0 -0.03 0 0 0.03 0" size="0.032" rgba="{accent}"/>
        <geom name="arm_upper" type="capsule" fromto="0 0 0 0 0 {P.L2}" size="0.02" rgba="{grey}"/>
        <geom name="arm_elbow_motor" type="box" pos="0 0.045 -0.03" size="0.021 0.021 0.021" rgba="{dark}"/>
        <body name="link3" pos="{_f(j[2].offset)}">
          {inertial(2)} {joint(2)}
          <geom name="arm_elbow_hub" type="cylinder" fromto="0 -0.026 0 0 0.026 0" size="0.03" rgba="{accent}"/>
          <geom name="arm_fore_a" type="capsule" fromto="0 0 0 0 0 {P.L3A}" size="0.017" rgba="{grey}"/>
          <body name="link4" pos="{_f(j[3].offset)}">
            {inertial(3)} {joint(3)}
            <geom name="arm_roll_ring" type="cylinder" fromto="0 0 -0.006 0 0 0.006" size="0.022" rgba="{dark}"/>
            <geom name="arm_fore_b" type="capsule" fromto="0 0 0 0 0 {P.L3 - P.L3A}" size="0.015" rgba="{grey}"/>
            <body name="link5" pos="{_f(j[4].offset)}">
              {inertial(4)} {joint(4)}
              <geom name="arm_wrist_hub" type="cylinder" fromto="0 -0.022 0 0 0.022 0" size="0.02" rgba="{accent}"/>
              <geom name="arm_wrist" type="box" pos="0 0 0.028" size="0.012 0.02 0.022" rgba="{dark}"/>
              <body name="link6" pos="{_f(j[5].offset)}">
                {inertial(5)} {joint(5)}
                <geom name="arm_hand" type="box" pos="0 0 0.068" size="0.034 0.016 0.016" rgba="{grey}"/>
                <site name="tcp" pos="0 0 {P.D6}" size="0.004" rgba="0 1 0 0.6"/>
                <body name="finger_l" pos="0.026 0 {finger_z}">
                  <inertial pos="0 0 0" mass="{finger_mass}" diaginertia="1e-6 1e-6 1e-6"/>
                  <joint name="finger_l" type="slide" axis="-1 0 0" range="0 0.011" damping="2"/>
                  <geom name="arm_finger_l" type="box" size="0.004 0.011 0.021" rgba="{dark}"/>
                </body>
                <body name="finger_r" pos="-0.026 0 {finger_z}">
                  <inertial pos="0 0 0" mass="{finger_mass}" diaginertia="1e-6 1e-6 1e-6"/>
                  <joint name="finger_r" type="slide" axis="1 0 0" range="0 0.011" damping="2"/>
                  <geom name="arm_finger_r" type="box" size="0.004 0.011 0.021" rgba="{dark}"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>"""

    cell, objects, welds = "", "", ""
    if scene:
        px, py = P.PICK_XY
        if vision:
            from vision import mat as _mat
            ox, oy, oz = VISION_CAM_OFFSET
            vis = f"""
    <geom name="pick_mat" type="plane" pos="{px} {py} 0.0006" euler="0 0 -1.5707963" size="{_mat.MAT_W / 2000} {_mat.MAT_H / 2000} 0.01" material="pickmat"/>
    <body name="pick_target" pos="{px} {py} 0"/>
    <camera name="vision" pos="{px + ox} {py + oy} {oz}" mode="targetbody" target="pick_target" fovy="{VISION_CAM_FOVY}"/>"""
        else:
            vis = f"""
    <geom name="pick_pad" type="cylinder" pos="{px} {py} 0.0005" size="0.03 0.0005" rgba="0.95 0.9 0.2 1"/>"""
        cell = f"""
    <geom name="pedestal" type="cylinder" fromto="0 0 0 0 0 0.035" size="0.06" rgba="{dark}"/>
    {vis}
    <geom name="stock_pad" type="box" pos="{STOCK_XY[0]} {STOCK_XY[1]} 0.0005" size="0.03 0.06 0.0005" rgba="0.6 0.6 0.6 1"/>
    {_bin("bin_red", *P.BINS["red"], "0.8 0.45 0.45 1")}
    {_bin("bin_blue", *P.BINS["blue"], "0.45 0.55 0.85 1")}"""
        h = P.OBJECT_SIZE / 2
        for i, colour in enumerate(colours):
            sx, sy = STOCK_XY[0], STOCK_XY[1] + (i - (len(colours) - 1) / 2) * 0.06
            objects += f"""
    <body name="obj{i}" pos="{sx} {sy} {h}">
      <freejoint name="obj{i}"/>
      <geom name="obj{i}" class="solid" type="box" size="{h} {h} {h}" mass="{P.PAYLOAD_MASS}"
            rgba="{OBJECT_COLOURS[colour]}" friction="1 0.01 0.001"/>
    </body>"""
            welds += f'\n    <weld name="grasp{i}" body1="link6" body2="obj{i}" active="false"/>'

    motors = "\n".join(
        f'    <motor name="m{i + 1}" joint="j{i + 1}" ctrlrange="{-d.hold_torque:.5g} {d.hold_torque:.5g}"/>'
        for i, d in enumerate(P.DRIVES))

    mat_asset = ""
    if vision and scene:
        mat_asset = f"""
    <texture name="pickmat" type="2d" file="{vision_mat_texture()}"/>
    <material name="pickmat" texture="pickmat" texrepeat="1 1" texuniform="false" reflectance="0"/>"""
    return f"""<mujoco model="ras_arm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -{P.G}"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
    <quality shadowsize="4096"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.93 0.95 0.98" rgb2="0.75 0.8 0.88" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.9" rgb2="0.86 0.86 0.83" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="10 10" reflectance="0.05"/>{mat_asset}
  </asset>
  <default>
    <geom contype="0" conaffinity="0"/>
    <default class="solid"><geom contype="1" conaffinity="1"/></default>
  </default>
  <worldbody>
    <light pos="0.4 -0.4 1.2" dir="-0.3 0.3 -1" diffuse="0.5 0.5 0.5" castshadow="true"/>
    <geom name="table" class="solid" type="plane" size="1 1 0.1" material="grid"/>
    <body name="cam_target" pos="0.1 0 0.12"/>
    <camera name="iso" pos="0.66 -0.62 0.47" mode="targetbody" target="cam_target" fovy="42"/>
    <camera name="side" pos="0.14 -1.05 0.22" mode="targetbody" target="cam_target" fovy="38"/>
    <camera name="top" pos="0.1 0 1.15" xyaxes="0 -1 0 1 0 0" fovy="42"/>
    {cell}
    {arm}
    {objects}
  </worldbody>
  <actuator>
{motors}
    <position name="grip_l" joint="finger_l" kp="60" ctrlrange="0 0.011"/>
    <position name="grip_r" joint="finger_r" kp="60" ctrlrange="0 0.011"/>
  </actuator>
  <equality>{welds}
  </equality>
</mujoco>
"""
