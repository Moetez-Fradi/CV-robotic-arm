"""Slides of the explainer video: narration and the picture that goes with each sentence.

Each slide has `lines` = (spoken text, caption text or None). `steps[i]` says which build step of the picture
is on screen while sentence i is spoken. `draw(canvas, step, ctx)` draws everything up to that step.
Spoken text is plain words (numbers and units spelled out) so the text-to-speech reads it correctly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image

from arm import kinematics as K
from arm import params as P
from .draw import (ACC, BAD, BG, BLUE, INK, JC, LINE, MUTED, OK, PANEL, PANEL2, YELLOW, Canvas, W)

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "reports" / "figures"


@dataclass
class Slide:
    key: str
    kicker: str
    title: str
    lines: list
    draw: callable = None
    steps: list = field(default_factory=list)
    clip: str = None           # "check" or "sort": play a simulation clip instead of a still picture

    def __post_init__(self):
        if not self.steps:
            self.steps = list(range(len(self.lines)))


def bullets(c, x, y, items, size=20, w=520, gap=14, color=INK, dot=ACC):
    for it in items:
        c.circle(x + 6, y + size * 0.62, 4, dot)
        h = c.text(x + 24, y, it, size, color, maxw=w)
        y += h + gap
    return y


def card(c, x, y, w, h, title, body=None, accent=ACC, tsize=18, bsize=15):
    c.rect(x, y, w, h, PANEL, LINE, 12, 2)
    c.rect(x, y + 10, 5, h - 20, accent, None, 3)
    c.text(x + 18, y + 10, title, tsize, INK, True)
    if body:
        c.text(x + 18, y + 12 + tsize * 1.35, body, bsize, MUTED, maxw=w - 34, spacing=1.3)


# ------------------------------------------------------------------ 1 title
def s_title(c, k, ctx):
    c.text(70, 150, "ROBOTIC ARM PROJECT", 18, ACC, True)
    c.text(70, 178, "Everything you need to\nunderstand, explained for\na software engineer", 44, INK, True, spacing=1.15)
    c.text(70, 350, "RAS Build Challenge, Project 3: six-axis arm that picks and sorts by colour", 18, MUTED, maxw=520)
    if k >= 1:
        for i, (t, s) in enumerate([("Mechanical", "joints, torque, gears"), ("Electrical", "motors, power, sensors"),
                                    ("Software", "control, simulation, plan")]):
            c.rect(70 + i * 190, 430, 175, 80, PANEL, LINE, 12, 2)
            c.rect(70 + i * 190, 440, 5, 60, JC[[0, 1, 2][i]], None, 3)
            c.text(88 + i * 190, 444, t, 20, INK, True)
            c.text(88 + i * 190, 472, s, 14, MUTED)
    if k >= 2:
        c.rect(70, 535, 560, 52, (60, 44, 16), YELLOW, 10, 2)
        c.text(90, 550, "Every dimension and mass shown is a placeholder until the CAD exists.", 16, YELLOW, maxw=520)
    c.rect(700, 120, 500, 470, PANEL, LINE, 16, 2)
    if ctx.get("still") is not None:
        c.paste(ctx["still"], 710, 130, w=480)


# ------------------------------------------------------------------ 2 challenge
def s_challenge(c, k, ctx):
    c.rect(48, 118, 560, 330, PANEL, LINE, 14, 2)
    c.text(70, 132, "WHAT IT MUST DO", 15, ACC, True)
    tasks = ["Home every axis on start-up, using limit switches", "Pick a light object from one fixed spot",
             "Read its colour and choose the bin", "Place it in the right bin, moving all six axes"]
    y = 170
    for i, t in enumerate(tasks):
        if k >= 0 and (k >= 1 or True):
            c.circle(86, y + 16, 15, ACC)
            c.text(86, y + 5, str(i + 1), 18, BG, True, anchor="ma")
            y += c.text(114, y + 2, t, 20, INK, maxw=470) + 22
    if k >= 1:
        c.rect(70, 372, 516, 58, (22, 56, 38), OK, 10, 2)
        c.text(90, 380, "Minimum demonstration: two colours, two bins,\nfull movement in all axes.", 16, OK, bold=True)
    if k >= 2:
        c.rect(640, 118, 592, 330, PANEL, LINE, 14, 2)
        c.text(662, 132, "THE HARDWARE IS FIXED", 15, ACC, True)
        rows = [("3 stepper motors", "base, shoulder, elbow", JC[0]), ("4 hobby servos", "wrist x3 and gripper", JC[3]),
                ("Arduino Uno + CNC shield", "the real-time brain", JC[2]), ("3 TMC2209 drivers + PCA9685", "motor and servo drivers", JC[4]),
                ("Colour, proximity, 3 limit switches", "the sensors", JC[1]), ("24 V supply + 2 buck converters", "the power", JC[5])]
        for i, (a, b, col) in enumerate(rows):
            yy = 166 + i * 45
            c.rect(662, yy, 6, 34, col, None, 3)
            c.text(680, yy, a, 17, INK, True)
            c.text(680, yy + 19, b, 13, MUTED)
    if k >= 3:
        c.rect(48, 470, 1184, 100, PANEL, LINE, 14, 2)
        c.text(70, 484, "HOW IT IS JUDGED", 15, ACC, True)
        c.text(70, 510, "Feasibility, technical quality, simulation evidence, fabrication readiness, team organisation.\n"
                        "Justified decisions backed by calculations and simulation win.", 17, INK, spacing=1.35)
        c.rect(960, 484, 250, 72, (60, 30, 28), BAD, 12, 2)
        c.text(1085, 492, "DESIGN BOOK DUE", 14, BAD, True, anchor="ma")
        c.text(1085, 514, "16 Nov, 23:59", 24, INK, True, anchor="ma")


# ------------------------------------------------------------------ 3 architecture
def s_arch(c, k, ctx):
    def arrow_label(p1, p2, label, col=MUTED):
        c.arrow(p1, p2, col, 3)
        if p1[1] == p2[1]:
            c.text((p1[0] + p2[0]) / 2, p1[1] - 26, label, 13, col, anchor="ma")
        else:
            c.text((p1[0] + p2[0]) / 2 - 14, (p1[1] + p2[1]) / 2 - 8, label, 13, col, anchor="ra")

    if k >= 1:
        c.box(48, 270, 230, 130, "PC (Python)", "planner, inverse kinematics,\nsimulation, dashboard", accent=BLUE)
    if k >= 2:
        arrow_label((282, 335), (390, 335), "USB serial")
        c.box(394, 270, 250, 130, "Arduino UNO", "real-time firmware:\nexact timing, simple code", accent=ACC)
        c.text(520, 410, "software view: this is the driver / firmware layer", 13, ACC, anchor="ma")
    if k >= 3:
        arrow_label((648, 290), (730, 190), "STEP / DIR pins")
        c.box(734, 140, 210, 100, "3 x TMC2209", "stepper drivers", accent=JC[0])
        c.arrow((948, 190), (1010, 190), MUTED, 3)
        c.box(1014, 140, 200, 100, "3 steppers", "J1  J2  J3", accent=JC[0])
    if k >= 4:
        arrow_label((648, 335), (730, 335), "I2C bus")
        c.box(734, 285, 210, 100, "PCA9685", "PWM board", accent=JC[3])
        c.arrow((948, 335), (1010, 335), MUTED, 3)
        c.box(1014, 285, 200, 100, "4 servos", "J4  J5  J6  gripper", accent=JC[3])
        c.box(734, 400, 210, 70, "TCS34725", "colour sensor (same bus)", accent=JC[3], ssize=13)
        c.line((690, 335), (690, 435), MUTED, 3)
        c.arrow((690, 435), (730, 435), MUTED, 3)
    if k >= 5:
        arrow_label((648, 380), (730, 520), "digital inputs")
        c.box(734, 490, 480, 80, "3 limit switches + proximity sensor", "plain on / off signals", accent=JC[1])


# ------------------------------------------------------------------ 4 degrees of freedom
def skeleton(c, ox, oy, sc, q_deg, mark=True, sel=None, widths=(14, 10)):
    q = np.radians(q_deg)
    fr = K.fk_frames(q)
    pts = [(0.0, 0.0)] + [(T[0, 3], T[2, 3]) for T in fr] + [(K.fk(q)[0, 3], K.fk(q)[2, 3])]
    sp = lambda p: (ox + p[0] * 1e3 * sc, oy - p[1] * 1e3 * sc)
    c.line(sp((-0.09, 0)), sp((0.3, 0)), INK, 3)
    c.rect(ox - 38, oy - 22, 76, 22, LINE, None, 4)
    chain = [pts[0], pts[2], pts[3], pts[4], pts[5]]
    for a, b in zip(chain, chain[1:]):
        c.line(sp(a), sp(b), LINE, widths[0])
    c.line(sp(pts[5]), sp(pts[7]), MUTED, widths[1])
    for i in [0, 1, 2, 3, 5, 4]:
        p = sp(pts[i + 1])
        c.circle(p[0], p[1], 14 if i != 5 else 20, JC[i] if i != 5 else None, JC[i], 3 if i != sel else 6)
        off = (-26, 18) if i == 4 else ((26, -22) if i == 5 else (22, -20))
        if mark:
            c.text(p[0] + off[0], p[1] + off[1], f"J{i + 1}", 18, JC[i], True, anchor="ma")
    t = sp(pts[7])
    c.line((t[0] - 8, t[1] - 8), (t[0] + 8, t[1] + 8), INK, 3)
    c.line((t[0] + 8, t[1] - 8), (t[0] - 8, t[1] + 8), INK, 3)
    return sp


def s_dof(c, k, ctx):
    skeleton(c, 90, 575, 0.78, [0, 35, 70, 0, 55, 0])
    names = [("J1", "base yaw", "turns about the vertical"), ("J2", "shoulder pitch", "tilts the whole arm"),
             ("J3", "elbow pitch", "folds the forearm"), ("J4", "forearm roll", "rolls the wrist"),
             ("J5", "wrist pitch", "nods the hand"), ("J6", "tool roll", "spins the gripper")]
    for i, (a, b, d) in enumerate(names):
        y = 128 + i * 52
        c.circle(448, y + 14, 8, JC[i])
        c.text(466, y, f"{a}  {b}", 18, INK, True)
        c.text(466, y + 23, d, 13, MUTED)
    c.rect(680, 118, 552, 450, PANEL, LINE, 14, 2)
    c.text(702, 132, "THE SOFTWARE VIEW", 15, ACC, True)
    code = [("q = [q1, q2, q3, q4, q5, q6]", 1, "six angles = the whole pose"),
            ("pose = fk(q)", 2, "x, y, z + orientation. One answer."),
            ("q_list = ik(pose)", 3, "0 to 8 answers. Maybe none."),
            ("# wrist axes meet at a point", 4, "so ik() is a formula, not a search")]
    y = 168
    for text, need, note in code:
        if k >= need:
            c.rect(702, y, 508, 56, PANEL2, None, 8)
            c.text(718, y + 6, text, 20, OK if text.startswith("#") is False else MUTED, mono=True)
            c.text(718, y + 32, note, 13, MUTED)
            y += 68
    if k >= 3:
        c.text(702, y + 8, "FK is a pure function.\nIK is the inverse problem:\nmany, one or no solutions.", 17, INK, spacing=1.35)


# ------------------------------------------------------------------ 6 workspace
def s_workspace(c, k, ctx):
    im = Image.open(FIG / "e1_reach_side.png").convert("RGB")
    c.rect(48, 112, 640, 470, (255, 255, 255), LINE, 12, 2)
    c.paste(im, 58, 118, h=458)
    if k >= 0:
        card(c, 720, 118, 512, 120, "Green = where the tip can go", "Every tool-tip position reachable while the tool points straight down, inside all joint limits.", OK)
    if k >= 1:
        card(c, 720, 252, 512, 120, "Pick spot and both bins fit", "Each task pose sits at least 35 mm inside the edge of the green area, and at least 5 degrees from every joint limit.", BLUE)
    if k >= 2:
        card(c, 720, 386, 512, 120, "Inverse kinematics tested", "2000 random poses in, 2000 recovered. Worst error about 2e-12: the formula is exact.", ACC)


# ------------------------------------------------------------------ 7 torque
def s_torque(c, k, ctx):
    px, py = 150, 330
    c.rect(48, 118, 700, 460, PANEL, LINE, 14, 2)
    c.circle(px, py, 20, JC[1], INK, 3)
    c.text(px, py + 36, "shoulder\n(pivot)", 14, MUTED, anchor="ma")
    c.line((px, py), (640, py), LINE, 16)
    c.rect(580, py - 28, 70, 56, JC[4], None, 8)
    c.text(615, py - 11, "load", 15, BG, True, anchor="ma")
    c.arrow((615, py + 30), (615, py + 130), BAD, 5, 16)
    c.text(630, py + 70, "weight\nm x g", 16, BAD, True)
    c.line((px, py - 70), (615, py - 70), MUTED, 2)
    c.line((px, py - 80), (px, py - 60), MUTED, 2)
    c.line((615, py - 80), (615, py - 60), MUTED, 2)
    c.text((px + 615) / 2, py - 108, "lever arm  d", 17, MUTED, anchor="ma")
    c.arc(px, py, 58, 150, 20, ACC, 5)
    c.text(px + 66, py - 52, "torque", 17, ACC, True)
    x = 780
    c.rect(x, 118, 452, 460, PANEL, LINE, 14, 2)
    if k >= 1:
        c.text(x + 24, 140, "torque = force x distance", 22, INK, True)
        c.text(x + 24, 176, "T = m x g x d", 28, ACC, True, mono=True)
        c.text(x + 24, 220, "newton metres (N.m)", 16, MUTED)
    if k >= 2:
        c.text(x + 24, 262, "OUR SHOULDER, ARM STRETCHED OUT", 14, ACC, True)
        c.text(x + 24, 288, "1.12 kg beyond the shoulder\n x 9.81 m/s^2\n x 0.22 m average lever", 19, INK, spacing=1.4)
        c.text(x + 24, 384, "= 2.4 N.m", 34, OK, True)
        c.text(x + 24, 430, "the worst load of any joint", 15, MUTED)
    if k >= 3:
        c.rect(x + 20, 466, 412, 94, (60, 44, 16), YELLOW, 10, 2)
        c.text(x + 36, 476, "Design rule: holding torque >= 2.5 x worst load.\nMotors give less while moving, and our masses are estimates.", 15, YELLOW, maxw=380, spacing=1.35)


# ------------------------------------------------------------------ 8 gears
def s_gears(c, k, ctx):
    def bar(y, label, val, vmax, col, w=640):
        c.text(70, y, label, 16, INK)
        c.rect(330, y + 2, w * val / vmax, 24, col, None, 5)
        c.text(330 + w * val / vmax + 10, y + 1, f"{val:.2f} N.m", 16, col, True)

    if k == 0:
        c.rect(48, 118, 1184, 270, PANEL, LINE, 14, 2)
        bar(160, "Stepper motor alone", 0.49, 7.6, BAD)
        bar(214, "Shoulder needs (worst case)", 2.42, 7.6, ACC)
        c.text(70, 290, "A gap of 5x. The motor cannot hold the arm up by itself.", 22, INK, True)
    else:
        bx = [60, 330, 600, 870]
        names = [("Stepper", "17HS4401S"), ("Gearbox", "5.18 : 1,  90 %"), ("Belt", "16 -> 64 teeth\n4 : 1,  95 %"), ("Shoulder joint", "")]
        for i, (a, b) in enumerate(names):
            c.box(bx[i], 150, 250, 100, a, b, accent=[JC[0], JC[1], JC[1], JC[4]][i], ssize=15)
            if i < 3:
                c.arrow((bx[i] + 254, 200), (bx[i + 1] - 4, 200), MUTED, 3)
        c.text(640, 118, "torque x ratio x efficiency,   speed / ratio", 17, MUTED, anchor="ma")
        if k >= 2:
            for xx, v in ((185, "0.42"), (455, "1.96"), (725, "7.44"), (995, "needs 2.42")):
                c.text(xx, 262, v + (" N.m" if "needs" not in v else " N.m"), 22, OK if v != "0.42" else MUTED, True, anchor="ma")
            c.rect(70, 330, 1090, 56, (22, 56, 38), OK, 10, 2)
            c.text(90, 342, "Total 20.7 : 1  ->  7.4 N.m available vs 2.4 needed  =  safety factor 3.1", 21, OK, True)
        if k >= 3:
            c.rect(70, 410, 520, 150, PANEL, LINE, 12, 2)
            c.text(90, 422, "THE PRICE", 14, ACC, True)
            c.text(90, 448, "Speed is divided by the same ratio.\nThe shoulder turns about 20x slower\nthan its motor shaft.", 18, INK, spacing=1.35)
            c.rect(620, 410, 540, 150, PANEL, LINE, 12, 2)
            c.text(640, 422, "ELBOW: plain motor, 16 -> 96 belt", 14, ACC, True)
            c.text(640, 448, "0.49 N.m x 6 x 0.95 = 2.8 N.m\nneeds 1.05 N.m: safety factor 2.7", 18, INK, spacing=1.35)


# ------------------------------------------------------------------ 9 steppers
def panel(c, x, y, w, h, head):
    c.rect(x, y, w, h, PANEL, LINE, 14, 2)
    c.text(x + 20, y + 12, head, 14, ACC, True)


def s_stepper(c, k, ctx):
    A, B, C, D = (48, 112, 580, 235), (652, 112, 580, 235), (48, 360, 580, 225), (652, 360, 580, 225)
    if k >= 0:
        panel(c, *A, "A STEPPER MOVES IN FIXED STEPS")
        cx, cy = 170, 235
        c.circle(cx, cy, 62, PANEL2, JC[0], 4)
        for i in range(40):
            a = math.radians(i * 9)
            c.line((cx + 62 * math.cos(a), cy + 62 * math.sin(a)), (cx + 74 * math.cos(a), cy + 74 * math.sin(a)), MUTED, 2)
        c.circle(cx, cy, 8, JC[0])
        c.text(300, 190, "200 steps per turn", 22, INK, True)
        c.text(300, 224, "= 1.8 degrees per step", 20, MUTED)
        c.text(300, 262, "like a rotary counter:\nyou count pulses, not angles", 16, MUTED, spacing=1.3)
    if k >= 1:
        panel(c, *B, "TWO WIRES: DIR AND STEP")
        x0, y1, y2 = 680, 195, 270
        c.text(x0, y1 - 10, "DIR", 15, INK, True)
        c.line((x0 + 50, y1 + 6), (1210, y1 + 6), BLUE, 3)
        c.text(x0, y2 - 10, "STEP", 15, INK, True)
        pts, x = [(x0 + 60, y2 + 14)], x0 + 60
        for _ in range(7):
            pts += [(x, y2 - 6), (x + 22, y2 - 6), (x + 22, y2 + 14), (x + 44, y2 + 14)]
            x += 44
        c.poly(pts, ACC, 3)
        c.text(670, 312, "each pulse = one move; DIR level = which way", 14, MUTED)
    if k >= 2:
        panel(c, *C, "MICROSTEPPING AND GEARING")
        c.text(70, 398, "1 pulse = 1/8 of a step = 0.225 deg at the motor", 17, INK)
        c.text(70, 430, "1600 pulses per motor turn", 17, INK)
        c.text(70, 462, "after 20.7 : 1 gearing, the shoulder gets", 17, INK)
        c.text(70, 494, "92 pulses per degree", 28, OK, True)
        c.text(70, 538, "= 0.011 degree per pulse", 15, MUTED)
    if k >= 3:
        panel(c, *D, "THE CATCH: NO SENSOR")
        c.rect(676, 402, 250, 60, PANEL2, None, 8)
        c.text(801, 412, "program believes\n90 degrees", 15, INK, anchor="ma", spacing=1.3)
        c.rect(956, 402, 250, 60, (60, 30, 28), BAD, 8, 2)
        c.text(1081, 412, "reality\n70 degrees", 15, BAD, anchor="ma", spacing=1.3)
        c.text(676, 480, "Overload makes it skip steps silently.\nFix: home against a limit switch at every start-up.", 15, INK, spacing=1.35)
    if k >= 4:
        c.rect(676, 534, 530, 40, (60, 44, 16), YELLOW, 8, 2)
        c.text(691, 543, "It holds position with full current: about 4 W of heat per motor, always.", 14, YELLOW, maxw=500)


# ------------------------------------------------------------------ 10 servos
def s_servo(c, k, ctx):
    A, B, C, D = (48, 112, 580, 235), (652, 112, 580, 235), (48, 360, 580, 225), (652, 360, 580, 225)
    if k >= 0:
        panel(c, *A, "A SERVO IS A CLOSED LOOP IN A BOX")
        xs = [70, 205, 340, 475]
        for x, (a, b) in zip(xs, [("Motor", ""), ("Gearbox", ""), ("Position\nsensor", ""), ("Controller", "")]):
            c.box(x, 180, 110, 70, a, None, tsize=15, accent=JC[3])
        for x in xs[:3]:
            c.arrow((x + 112, 215), (x + 133, 215), MUTED, 2, 9)
        c.line((530, 252), (530, 280), ACC, 2)
        c.line((530, 280), (125, 280), ACC, 2)
        c.arrow((125, 280), (125, 254), ACC, 2, 9)
        c.text(330, 288, "feedback: like a thermostat", 14, ACC, anchor="ma")
        c.text(70, 318, "You give a setpoint, it holds it.", 15, MUTED)
    if k >= 1:
        panel(c, *B, "THE SETPOINT IS A PULSE WIDTH")
        for i, (ms, ang) in enumerate([(1.0, "45"), (1.5, "90"), (2.0, "135")]):
            y = 168 + i * 50
            x0 = 690
            pts = [(x0, y + 28), (x0, y), (x0 + 24 * ms, y), (x0 + 24 * ms, y + 28), (x0 + 480, y + 28)]
            c.poly(pts, ACC if i == 1 else BLUE, 3)
            c.text(x0 + 24 * ms + 14, y + 2, f"{ms} ms  ->  {ang} deg", 15, INK)
        c.text(690, 318, "repeated every 20 ms (50 times per second)", 13, MUTED)
    if k >= 2:
        panel(c, *C, "A DEDICATED CHIP MAKES THE PULSES")
        c.box(70, 410, 150, 60, "Arduino", None, tsize=17, accent=ACC)
        c.arrow((222, 440), (285, 440), MUTED, 3)
        c.text(253, 418, "I2C", 13, MUTED, anchor="ma")
        c.text(253, 447, "angles", 13, MUTED, anchor="ma")
        c.box(290, 400, 150, 80, "PCA9685", "16 PWM channels", tsize=17, ssize=13, accent=JC[3])
        c.arrow((442, 440), (505, 440), MUTED, 3)
        c.box(510, 410, 100, 60, "servos", None, tsize=17, accent=JC[3])
        c.text(70, 506, "Timing is exact and the Arduino stays free.", 15, MUTED)
    if k >= 3:
        panel(c, *D, "TORQUE UNITS AND THE LIMIT")
        c.text(676, 396, "1 kg.cm = 0.098 N.m", 19, INK, True)
        c.text(676, 428, "DS3218MG 20 kg.cm = 1.96 N.m\nMG996 13 kg.cm = 1.27 N.m", 16, MUTED, spacing=1.35)
    if k >= 4:
        c.rect(676, 492, 530, 72, (60, 30, 28), BAD, 10, 2)
        c.text(692, 500, "No angle readback: a servo cannot be homed with a\nswitch. The wrist needs a hard-stop or start pose.", 15, BAD, spacing=1.35)


# ------------------------------------------------------------------ 11 power
def s_power(c, k, ctx):
    if k >= 1:
        c.box(48, 270, 150, 110, "Supply", "24 V\n12.5 A, 300 W", accent=JC[4], ssize=14)
        c.arrow((200, 325), (240, 325), MUTED, 3)
        c.box(244, 285, 130, 80, "Fuse +\nE-stop", None, tsize=16, accent=BAD)
        c.line((376, 325), (410, 325), MUTED, 3)
        c.line((410, 170), (410, 480), MUTED, 3)
        rows = [(170, "24 V", "TMC2209 x3", "3 steppers", "about 16 W", JC[0]),
                (325, "6 V buck", "PCA9685", "4 servos", "about 12 W typical", JC[3]),
                (480, "5 V buck", "Arduino", "+ sensors", "under 1 W", JC[2])]
        for y, rail, mid, end, pw, col in rows:
            c.arrow((410, y), (440, y), MUTED, 3)
            c.box(444, y - 36, 110, 72, rail, None, tsize=16, accent=col)
            c.arrow((556, y), (576, y), MUTED, 3)
            c.box(580, y - 36, 130, 72, mid, None, tsize=16, accent=col)
            c.arrow((712, y), (732, y), MUTED, 3)
            c.box(736, y - 36, 120, 72, end, None, tsize=16, accent=col)
            c.text(796, y + 42, pw, 13, MUTED, anchor="ma")
        c.text(48, 536, "all grounds joined", 14, MUTED)
        c.line((48, 560), (856, 560), INK, 3)
        c.text(452, 566, "common ground (0 V reference)", 13, MUTED, anchor="ma")
    x, w = 880, 352
    if k >= 0:
        card(c, x, 112, w, 108, "Voltage, current, power", "Voltage = pressure. Current = flow.\nPower (watts) = volts x amps.", BLUE, 17, 14)
    if k >= 2:
        card(c, x, 230, w, 108, "Why separate rails?", "A servo's start-up current spike dips the\nvoltage. On a shared rail the Arduino\nwould reset mid-move.", ACC, 17, 14)
    if k >= 3:
        card(c, x, 348, w, 108, "Power is not the problem", "Typical 29 W, worst fault 84 W,\nfrom a 300 W supply.", OK, 17, 14)
    if k >= 4:
        card(c, x, 466, w, 112, "The real risk: a fault", "All 4 servos stalled = 10 A, but the buck\nis rated 8 A. Firmware must detect it.", BAD, 17, 14)


# ------------------------------------------------------------------ 12 sensing
def s_sensing(c, k, ctx):
    if k >= 0:
        panel(c, 48, 112, 600, 150, "I2C: TWO WIRES, MANY DEVICES, EACH WITH AN ADDRESS")
        c.box(64, 156, 130, 80, "Arduino", "master", tsize=16, ssize=13, accent=ACC)
        c.line((196, 186), (556, 186), BLUE, 3)
        c.line((196, 214), (556, 214), YELLOW, 3)
        c.text(566, 176, "SDA data", 13, BLUE)
        c.text(566, 204, "SCL clock", 13, YELLOW)
        for x, a, b in ((260, "PCA9685", "address 0x40"), (430, "TCS34725", "address 0x29")):
            c.line((x + 50, 186), (x + 50, 156), BLUE, 3)
            c.line((x + 70, 214), (x + 70, 156), YELLOW, 3)
        c.rect(255, 224, 170, 30, PANEL2, None, 6)
        c.text(340, 228, "PCA9685  0x40", 14, INK, anchor="ma")
        c.rect(430, 224, 190, 30, PANEL2, None, 6)
        c.text(525, 228, "TCS34725  0x29", 14, INK, anchor="ma")
    if k >= 1:
        panel(c, 48, 272, 600, 150, "COLOUR: COMPARE RATIOS, NOT RAW VALUES")
        for i, (h, col) in enumerate([(90, (230, 80, 70)), (50, (80, 200, 100)), (40, (90, 130, 240))]):
            c.rect(80 + i * 60, 400 - h, 44, h, col, None, 4)
        c.text(80, 404, "R", 13, INK)
        c.text(140, 404, "G", 13, INK)
        c.text(200, 404, "B", 13, INK)
        c.text(290, 320, "red / blue is large  ->  red bin\nred / blue is small  ->  blue bin", 17, INK, spacing=1.4)
        c.text(290, 380, "room light changes the raw counts,\nnot the ratio", 14, MUTED, spacing=1.3)
    if k >= 2:
        panel(c, 48, 432, 600, 150, "PROXIMITY: ONE ON / OFF SIGNAL")
        c.box(70, 480, 120, 60, "IR sensor", None, tsize=15, accent=JC[1])
        c.line((196, 510), (330, 510), YELLOW, 2)
        c.rect(334, 488, 44, 44, JC[4], None, 6)
        c.arrow((384, 510), (470, 510), MUTED, 3)
        c.text(480, 494, "pin = 1:\nobject at the pick spot", 15, INK, spacing=1.3)
    if k >= 3:
        panel(c, 672, 112, 560, 470, "LIMIT SWITCHES AND HOMING (EVERY POWER-ON)")
        steps = ["Creep towards the switch, slowly", "Stop the instant it closes", "Declare: this angle is the known reference",
                 "Back off a little", "Now trust the step counter"]
        for i, t in enumerate(steps):
            y = 160 + i * 80
            c.rect(700, y, 500, 58, PANEL2, JC[1] if i == 1 else LINE, 10, 2)
            c.circle(730, y + 29, 14, ACC)
            c.text(730, y + 17, str(i + 1), 17, BG, True, anchor="ma")
            c.text(758, y + 15, t, 17, INK)
            if i < 4:
                c.arrow((950, y + 60), (950, y + 78), MUTED, 2, 8)


# ------------------------------------------------------------------ 13 firmware
def s_firmware(c, k, ctx):
    if k >= 0:
        panel(c, 48, 112, 600, 150, "THE ARDUINO IS SMALL")
        c.text(70, 150, "16 MHz,  2 KB of memory", 22, INK, True)
        c.text(70, 188, "So the PC does the geometry and sends\njoint targets. The Arduino only executes.", 17, MUTED, spacing=1.35)
    if k >= 1:
        panel(c, 48, 272, 600, 310, "NEVER MOVE INSTANTLY: A TRAPEZOID")
        x0, y0 = 90, 430
        c.line((x0, y0), (610, y0), MUTED, 2)
        c.line((x0, y0), (x0, 312), MUTED, 2)
        c.poly([(x0, y0), (x0 + 130, 336), (x0 + 350, 336), (x0 + 480, y0)], ACC, 4)
        c.text(x0 + 65, y0 + 6, "1 speed up", 14, INK, anchor="ma")
        c.text(x0 + 240, y0 + 6, "2 cruise", 14, INK, anchor="ma")
        c.text(x0 + 415, y0 + 6, "3 brake", 14, INK, anchor="ma")
        c.text(610, y0 - 22, "time", 13, MUTED, anchor="ra")
        c.text(x0 + 8, 300, "speed", 13, MUTED)
    if k >= 2:
        c.rect(70, 482, 556, 84, PANEL2, None, 8)
        c.text(88, 490, "Step pulses needed at peak: 3.8 kHz", 16, YELLOW, bold=True)
        c.text(88, 512, "A simple polling loop manages about 4 kHz", 15, MUTED)
        c.rect(88, 540, 500, 12, LINE, None, 4)
        c.rect(88, 540, 500 * 3.8 / 4.4, 12, YELLOW, None, 4)
        c.line((88 + 500 * 4 / 4.4, 534), (88 + 500 * 4 / 4.4, 558), BAD, 3)
    if k >= 3:
        panel(c, 672, 112, 560, 470, "THE BEHAVIOUR IS A STATE MACHINE")
        names = ["HOME", "WAIT for object", "READ colour", "PICK", "PLACE in bin", "RETURN"]
        pos = [(760, 190), (1000, 190), (1000, 300), (1000, 410), (760, 410), (760, 300)]
        for i, (n, (x, y)) in enumerate(zip(names, pos)):
            c.rect(x - 90, y - 28, 180, 56, PANEL2, JC[i % 6], 12, 2)
            c.text(x, y - 12, n, 17, INK, True, anchor="ma")
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)]:
            (x1, y1), (x2, y2) = pos[a], pos[b]
            dx, dy = x2 - x1, y2 - y1
            n = math.hypot(dx, dy)
            c.arrow((x1 + dx / n * 96 if dy == 0 else x1, y1 + dy / n * 34 if dx == 0 else y1),
                    (x2 - dx / n * 96 if dy == 0 else x2, y2 - dy / n * 34 if dx == 0 else y2), MUTED, 3)
        c.arrow((760, 272), (760, 222), MUTED, 3)
        c.text(952, 500, "after RETURN, back to WAIT for the next object", 14, MUTED, anchor="ma")


# ------------------------------------------------------------------ 14 results
def s_results(c, k, ctx):
    rows = [("E1", "Kinematics and workspace", "Can the arm reach everything?", "all poses reachable, IK exact", JC[0]),
            ("E2", "Torque and gearing", "Which motor, which ratio?", "safety factors 3.1 and 2.7", JC[1]),
            ("E3", "Motion timing", "How fast, how many pulses?", "6.5 s per object, 3.8 kHz", JC[2]),
            ("E4", "Physics simulation", "Does the whole cycle work?", "all 6 axes move, 45 % peak torque", JC[4]),
            ("E5", "Power budget", "Is the supply enough?", "29 W typical, 84 W worst fault", JC[3])]
    for i, (e, name, q, res, col) in enumerate(rows):
        y = 118 + i * 90
        c.rect(48, y, 1184, 76, PANEL, LINE, 12, 2)
        c.rect(48, y + 8, 7, 60, col, None, 3)
        c.text(76, y + 12, e, 28, col, True)
        c.text(150, y + 10, name, 21, INK, True)
        c.text(150, y + 40, q, 15, MUTED)
        if k >= 1:
            c.text(700, y + 22, res, 21, OK, True)


# ------------------------------------------------------------------ 16 uncertainty and next steps
def s_next(c, k, ctx):
    panel(c, 48, 112, 570, 470, "WHAT WE DO NOT KNOW YET")
    items = ["Every length and mass: placeholders until the CAD exists", "Geared motor torque and gearbox efficiency: assumed",
             "Pulse rate on a real Arduino: untested", "The grip itself: simulated as an attachment", "Collisions between arm parts: not checked"]
    y = bullets(c, 70, 150, items[:2], 18, 520, 16, INK, BAD)
    if k >= 1:
        bullets(c, 70, y, items[2:], 18, 520, 16, INK, BAD)
    if k >= 2:
        panel(c, 650, 112, 582, 470, "NEXT STEPS")
        nxt = ["Get dimensions, masses and centres of mass from the CAD", "Type them into the dashboard, rerun everything",
               "Check the datasheets; email ras@ieee.tn early (third pulley, smaller fuse)",
               "Write the design book from the reports", "Record the video, test every link, submit before 16 Nov 23:59"]
        y = 150
        for i, t in enumerate(nxt[: 3 if k == 2 else 5]):
            c.circle(676, y + 12, 13, OK)
            c.text(676, y + 1, str(i + 1), 16, BG, True, anchor="ma")
            y += c.text(702, y, t, 17, INK, maxw=500) + 18


# ------------------------------------------------------------------ 17 recap
def s_recap(c, k, ctx):
    items = [("Six joints = six numbers", "kinematics converts between angles and tool position", JC[0]),
             ("Torque = force x distance", "gears and belts trade speed for torque", JC[1]),
             ("Steppers count pulses, servos take angles", "steppers need homing, servos cannot report back", JC[2]),
             ("Separate power rails", "faults matter more than normal running", JC[4]),
             ("The PC plans, the Arduino executes", "open the dashboard to explore the numbers", JC[3])]
    for i, (a, b, col) in enumerate(items):
        if k >= i:
            y = 122 + i * 92
            c.rect(48, y, 1184, 78, PANEL, LINE, 12, 2)
            c.rect(48, y + 8, 7, 62, col, None, 3)
            c.text(80, y + 10, a, 24, INK, True)
            c.text(80, y + 44, b, 16, MUTED)
    if k >= 4:
        c.text(1200, 122 + 4 * 92 + 26, "uv run python -m dashboard", 18, ACC, mono=True, anchor="ra")


def build_slides() -> list:
    L = lambda *xs: [(x, None) for x in xs]
    return [
        Slide("title", "Introduction", "Welcome", L(
            "This video is a crash course on our robotic arm project, written for a software engineer.",
            "By the end you should understand what the machine is made of, why each part was chosen, and what the numbers in our reports mean.",
            "One warning first: every dimension and mass you will see is a placeholder until the CAD exists. The method is final, the numbers are not."),
            s_title),
        Slide("challenge", "The challenge", "What we have to build", L(
            "The job, from the spec book, is easy to state. Home every axis against its limit switch, pick a light object from one fixed spot, read its colour, and drop it in the matching bin, moving all six axes.",
            "The minimum demonstration is two colours and two bins.",
            "The hardware is fixed: three stepper motors for the big joints, four hobby servos for the wrist and gripper, an Arduino Uno with a motor shield, and a colour sensor.",
            "The jury rewards justified decisions backed by calculation and simulation, and the design book is due on the sixteenth of November at midnight."),
            s_challenge),
        Slide("arch", "System architecture", "The whole system as a software architecture", L(
            "Here is the whole system, seen as a software architecture.",
            "On the left, a normal computer runs Python. It plans the motions, solves the geometry, and runs our simulations.",
            "It talks over a USB serial cable, with simple text commands, to the Arduino. Think of the Arduino as the firmware layer: it must produce exact timing, which a desktop operating system cannot guarantee.",
            "For the three big joints, the Arduino sends step and direction pulses to stepper drivers, which energise the motor coils.",
            "For the wrist and gripper, it sends angle commands over an I two C bus to a board that drives the four servos. The same bus reads the colour sensor.",
            "Finally, plain digital inputs read the limit switches and the proximity sensor."),
            s_arch),
        Slide("dof", "Mechanical 1", "Joints, degrees of freedom, kinematics", [
            ("Mechanically, an arm is a chain of rigid links connected by joints, and each joint is one degree of freedom.", None),
            ("Six joints means the arm's whole pose is a vector of six numbers, like a struct of six floats.", None),
            ("Forward kinematics is a pure function from those six angles to where the tool tip is and how it is oriented. It always has exactly one answer.", None),
            ("Inverse kinematics is the reverse: given where you want the tip, find the angles. That one can have several answers, or none, if the target is out of reach.", None),
            ("Our wrist is built so its last three axes cross at one point, which makes the inverse problem a formula, instead of a slow numerical search.", None)],
            s_dof),
        Slide("check", "Mechanical 2", "The six axes, one at a time", L(
            "Here is our simulated arm moving each joint alone, so you can see what the six axes really are. J one is the base: it spins the whole arm left and right.",
            "J two, the shoulder, tilts everything forward and back, and J three, the elbow, folds the forearm.",
            "J four rolls the forearm like turning a screwdriver, J five nods the hand up and down, and J six spins the gripper.",
            "Together they let the hand reach a point, and also choose which way the gripper faces."),
            None, clip="check"),
        Slide("workspace", "Mechanical 3", "Workspace: where can the tip go?", L(
            "The workspace is the set of places the tip can reach. In this side view, the green area is everywhere the tip can go while pointing straight down, which is how we grab objects.",
            "The pick spot and both bins sit inside it, with comfortable margin from the edge and from every joint limit, so the layout is feasible.",
            "We also tested the inverse kinematics code on two thousand random poses. It recovered every one, with errors around ten to the minus twelve."),
            s_workspace),
        Slide("torque", "Mechanical 4", "Torque: the physics behind most decisions", L(
            "Now the physics that drives most mechanical decisions: torque.",
            "Torque is a twisting force: the force times the distance from the pivot. Hold a heavy bag at arm's length and your shoulder works far harder than with the bag against your chest.",
            "For our arm stretched out horizontally, about one point one kilograms hang beyond the shoulder, with an average lever of about twenty two centimetres. That is about two point four newton metres: the worst load any joint sees.",
            "Our rule is that every joint must be able to hold at least two and a half times its worst load, because motors deliver less torque while moving, and because our masses are only estimates."),
            s_torque),
        Slide("gears", "Mechanical 5", "Gears and belts: trading speed for torque", L(
            "The problem: a stepper motor holds only about half a newton metre, but the shoulder needs two point four.",
            "The fix is a mechanical transformer. A gearbox, or a belt between two pulleys, multiplies torque by its ratio and divides speed by the same ratio, minus a few percent lost to friction.",
            "Our geared motor goes on the shoulder. The gearbox gives five point one eight to one, and a belt from a sixteen tooth pulley to a sixty four tooth pulley adds four to one. Together that is about twenty one to one, turning point four two newton metres into seven point four, a safety factor of three.",
            "The price is speed: the shoulder turns twenty times slower than its motor. The elbow gets a plain motor and a six to one belt, with a safety factor of two point seven."),
            s_gears),
        Slide("stepper", "Electrical 1", "Stepper motors", L(
            "Now the electrical side, starting with the stepper motor. It turns in small fixed increments: two hundred steps per revolution, so one point eight degrees each.",
            "You control it like a counter. A driver chip, the T M C twenty two oh nine, takes two signals: direction, a single level, and step, where every pulse means move one increment.",
            "Microstepping lets the driver split each step into eight, so sixteen hundred pulses make one turn. Combined with our gearing, that is about ninety two pulses per degree at the shoulder.",
            "The catch is that a stepper has no sensor. The program only believes it knows the angle, and an overloaded motor silently skips steps. That is why every start-up begins with homing against a limit switch, to re-establish zero.",
            "Also, a stepper holding position still draws full current, so it runs warm even when the arm is still."),
            s_stepper),
        Slide("servo", "Electrical 2", "Hobby servos", L(
            "Servos work the opposite way. A hobby servo hides a motor, a gearbox, a position sensor, and a small controller in one box, so it closes its own feedback loop.",
            "You give it a setpoint: a pulse repeated fifty times per second, whose width, between half a millisecond and two and a half milliseconds, encodes the target angle.",
            "Generating those pulses precisely for several servos would eat the Arduino's time, so a P C A nine six eight five board does it in hardware. We only send it angles over I two C.",
            "Servo torque is quoted in kilogram centimetres, a hobbyist unit. Multiply by zero point zero nine eight to get newton metres: our twenty kilogram centimetre servos give about two newton metres.",
            "The limitation: you cannot read the angle back, so a servo cannot be homed with a switch, and the spec book's home every axis needs a workaround for the wrist."),
            s_servo),
        Slide("power", "Electrical 3", "Power: voltage, current and the rails", L(
            "Next, power. Three quantities matter: voltage, which you can think of as pressure; current, the flow; and power, the product of the two, measured in watts.",
            "Our supply makes twenty four volts. That goes through a fuse and the emergency stop to the stepper drivers. A buck converter, which is an efficient step down regulator, makes six volts for the servos, and a second one makes five volts for the Arduino and sensors.",
            "Why not one rail? When a servo starts moving it pulls a sudden current spike, which makes the voltage dip. If the Arduino shared that rail it would reset in the middle of a motion. Separate regulators work like separate services, so one noisy neighbour cannot take down the others. All the grounds are joined, so every signal shares a common reference.",
            "Our estimate is about twenty nine watts in normal running, and eighty four watts in the worst fault, from a three hundred watt supply. So raw power is not the problem.",
            "The real risk is a fault. If all four servos stall at once they would draw ten amps, more than the eight amp converter can give, so the firmware must detect a blocked gripper and stop."),
            s_power),
        Slide("sensing", "Electrical 4", "Sensors, I²C and homing", L(
            "Now the sensors. I two C is a two wire bus that many chips share. Each device has an address, like an endpoint: ours are the servo board and the colour sensor.",
            "The colour sensor reports counts of red, green, blue and clear light. We classify by comparing ratios, for example red divided by blue, instead of trusting absolute values, which change with room light.",
            "The infrared proximity sensor is simpler: one digital signal that flips when an object is in front of it, telling us an object has arrived at the pick spot.",
            "The three limit switches are mechanical buttons at the end of each stepper joint. At power on we creep toward each switch, stop the instant it closes, declare that angle to be the known reference, back off slightly, and only then trust the step counter."),
            s_sensing),
        Slide("firmware", "Software", "Firmware and motion control", L(
            "Now the software. The Arduino Uno has a sixteen megahertz processor and two kilobytes of memory, so we keep it simple: the P C does the geometry and sends joint targets, and the Arduino only executes them.",
            "Motion is never instant. We use a trapezoidal speed profile: accelerate, cruise, brake. That avoids jerks that would make a stepper skip steps or shake the arm.",
            "The Arduino must emit step pulses at exactly the right moments. Our plan needs about three point eight thousand pulses per second at peak, close to the roughly four thousand a simple polling loop can do. If pulses are missed, we move stepping into a timer interrupt.",
            "The behaviour itself is a small state machine: home, wait for an object, read the colour, pick, place in the matching bin, return, and repeat."),
            s_firmware),
        Slide("results", "Simulation", "Five experiments, five answers", L(
            "We checked the design with five experiments. Each produces a report with plots, and all of them regenerate from the dashboard when the values change.",
            "Kinematics shows every task pose is reachable. The torque study chose the gearing. Timing gives about six and a half seconds per object. The physics simulation runs the whole cycle with the real masses, and the busiest joint, the elbow, uses forty five percent of its available torque. Power is far below the supply's limit."),
            s_results),
        Slide("sort", "Simulation", "The full cycle in the physics simulator", L(
            "Watch the sort cycle. The arm goes above the pick spot, straight down, and closes the gripper.",
            "It lifts, swings to the red bin, turns the wrist, lowers, and opens.",
            "Then it repeats for the blue cube, turning the wrist the other way, and returns to its ready pose.",
            "Notice that every move starts and ends gently. That is the trapezoid speed profile from the firmware section."),
            None, clip="sort"),
        Slide("next", "What is next", "Uncertainties and next steps", L(
            "Let us be honest about what is uncertain. Every length and mass is a placeholder until your mechanical teammate shares the CAD. The geared motor's torque and the gearbox efficiency are assumed from typical values.",
            "The pulse rate of the Arduino is untested, the grip itself is not simulated, and collisions between parts of the arm are not checked.",
            "So the next steps are: get the real dimensions, masses and centres of mass, type them into the dashboard and rerun everything. Check the datasheets, and email the organisers early about a third pulley and a smaller fuse.",
            "Then write the design book from the reports, record the final video, test every link, and submit before the deadline."),
            s_next, steps=[0, 1, 2, 3]),
        Slide("recap", "Recap", "Five things to remember", L(
            "To recap. Six joints are six numbers, and kinematics converts between joint angles and tool position.",
            "Torque is force times distance, and gears and belts trade speed for torque.",
            "Steppers count pulses and need homing. Servos take angle commands, but cannot report back.",
            "Separate power rails keep the motors from resetting the brain, and fault cases matter more than normal running.",
            "The computer plans and the Arduino executes. Open the dashboard to explore the numbers, and good luck with the design book."),
            s_recap),
    ]
