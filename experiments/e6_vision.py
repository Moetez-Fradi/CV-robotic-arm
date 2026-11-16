"""E6 - Computer vision: colour detection and image-to-arm calibration, tested on simulated camera frames.

Questions answered:
  1. Does the camera classify the object colour correctly under realistic lighting changes?
  2. Does it ever send an object to the wrong bin, and does it refuse unknown colours and an empty mat?
  3. How accurately does it locate the object in arm coordinates?
  4. Why white balance on the grey mat is needed (same pipeline with and without it).

Run:  uv run python -m experiments.e6_vision
"""
from __future__ import annotations

import colorsys
import shutil
import time
from dataclasses import replace

import cv2
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from arm import params as P
from arm.mjcf import VISION_CAM_FOVY, VISION_CAM_OFFSET, VISION_CAM_SIZE, build_xml
from vision import mat
from vision.detector import Calib, analyse, calibrate_colours, draw_overlay, intrinsics_from_fov
from .common import FIGS, ROOT, save_json, savefig, table, verdict, write_report

N_TEST = 600
SEED = 7
NOMINAL_LIGHT = 0.68                  # scene light scale giving a mat grey of ~170/255
INTENSITY = (0.35, 1.5)               # x nominal
TEMPS = {2700: (1.00, 0.70, 0.42), 3500: (1.00, 0.80, 0.60), 4500: (1.00, 0.90, 0.80),
         5500: (1.00, 0.96, 0.92), 6500: (1.00, 1.00, 1.00), 8000: (0.84, 0.92, 1.00)}   # RGB light tint
HUES = {"red": (-12, 12), "blue": (205, 235), "green": (100, 140), "yellow": (45, 62)}   # object hue ranges (deg)
CLASS_P = {"red": 0.38, "blue": 0.38, "green": 0.06, "yellow": 0.06, "none": 0.12}
OFFSET_MM = 20.0                      # random placement error of the object around the cross
NOISE = 3.0                           # sensor noise (grey levels, 1 sigma)
JPEG_Q = 80                           # webcams stream MJPEG
BOOK_IMG = ROOT / "design_book" / "images" / "vision"


class Cell:
    """MuJoCo cell with the pick mat, one object and the vision camera."""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_string(build_xml(("red",), vision=True))
        self.d = mujoco.MjData(self.m)
        self.d.qpos[:6] = np.radians(P.READY_Q_DEG)
        b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "obj0")
        self.qa = self.m.jnt_qposadr[self.m.body_jntadr[b]]
        self.gid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "obj0")
        self.W, self.H = VISION_CAM_SIZE
        self.r = mujoco.Renderer(self.m, self.H, self.W)

    def frame(self, rgb=None, dx=0.0, dy=0.0, yaw=0.0, intensity=1.0, temp=6500, noise=0.0, jpeg=None, rng=None,
              camera="vision"):
        m, d = self.m, self.d
        k = NOMINAL_LIGHT * intensity
        tint = np.array(TEMPS[temp])
        m.vis.headlight.ambient[:] = 0.45 * k * tint
        m.vis.headlight.diffuse[:] = 0.60 * k * tint
        m.light_diffuse[0][:] = 0.50 * k * tint
        px, py = P.PICK_XY
        if rgb is None:
            d.qpos[self.qa:self.qa + 3] = [px, py, -0.5]          # no object: hide it under the table
        else:
            m.geom_rgba[self.gid] = [*rgb, 1.0]
            d.qpos[self.qa:self.qa + 7] = [px + dx / 1e3, py + dy / 1e3, P.OBJECT_SIZE / 2,
                                           np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        self.r.update_scene(d, camera=camera)
        img = cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR).astype(np.float64)
        if noise and rng is not None:
            img += rng.normal(0, noise, img.shape)
        img = np.clip(img, 0, 255).astype(np.uint8)
        if jpeg:
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg])
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img


def obj_rgb(cls, rng):
    lo, hi = HUES[cls]
    h = (rng.uniform(lo, hi) % 360) / 360
    return colorsys.hsv_to_rgb(h, rng.uniform(0.65, 0.9), rng.uniform(0.55, 0.9))


def main():
    rng = np.random.default_rng(SEED)
    cell = Cell()
    W, H = cell.W, cell.H
    pick_mm = [P.PICK_XY[0] * 1e3, P.PICK_XY[1] * 1e3]
    calib = Calib(K=intrinsics_from_fov(W, H, VISION_CAM_FOVY).tolist(), dist=[0, 0, 0, 0, 0], pick_xy=pick_mm,
                  object_height_mm=P.OBJECT_SIZE * 1e3)

    # ---------------- calibration: 6 frames per colour, nominal light, object on the cross
    samples = {c: [cell.frame(colorsys.hsv_to_rgb((np.mean(HUES[c]) % 360) / 360, 0.78, 0.72),
                              dx=rng.uniform(-3, 3), dy=rng.uniform(-3, 3), yaw=rng.uniform(-0.3, 0.3))
                   for _ in range(6)] for c in ("red", "blue")}
    calibrate_colours(samples, calib, wb=True)
    calib_nowb = replace(calib, colours={})
    calibrate_colours(samples, calib_nowb, wb=False)
    calib.save(ROOT / "vision" / "calib_sim.json")

    # ---------------- test campaign
    classes = list(CLASS_P)
    rows = []
    t0 = time.time()
    for n in range(N_TEST):
        cls = rng.choice(classes, p=list(CLASS_P.values()))
        rgb = None if cls == "none" else obj_rgb(cls, rng)
        r_off = OFFSET_MM * np.sqrt(rng.uniform())
        a = rng.uniform(0, 2 * np.pi)
        dx, dy = r_off * np.cos(a), r_off * np.sin(a)
        cond = dict(intensity=float(rng.uniform(*INTENSITY)), temp=int(rng.choice(list(TEMPS))))
        img = cell.frame(rgb, dx, dy, rng.uniform(-np.pi / 4, np.pi / 4), noise=NOISE, jpeg=JPEG_Q, rng=rng, **cond)
        res = analyse(img, calib)
        res_nowb = analyse(img, calib_nowb, wb=False)
        res_noparallax = analyse(img, replace(calib, centroid_height_frac=0.0))
        true_xy = np.array(pick_mm) + [dx, dy]
        rows.append(dict(cls=cls, **cond, res=res, nowb=res_nowb,
                         err=float(np.hypot(res.x_mm - true_xy[0], res.y_mm - true_xy[1])) if res.present else np.nan,
                         err_np=float(np.hypot(res_noparallax.x_mm - true_xy[0], res_noparallax.y_mm - true_xy[1]))
                         if res_noparallax.present else np.nan,
                         img=img if n < 400 else None))
    elapsed = time.time() - t0

    def expected(cls):
        return {"red": "red", "blue": "blue", "none": "none"}.get(cls, "unknown")

    def outcome(cls, r):
        got = r.colour if r.present else "none"
        return got == expected(cls), got

    labels_true = ["red", "blue", "green", "yellow", "none"]
    labels_pred = ["red", "blue", "unknown", "none"]

    def confusion(key):
        Cm = np.zeros((len(labels_true), len(labels_pred)), int)
        for rw in rows:
            got = rw[key].colour if rw[key].present else "none"
            Cm[labels_true.index(rw["cls"]), labels_pred.index(got)] += 1
        return Cm

    C_wb, C_nowb = confusion("res"), confusion("nowb")

    def stats(key):
        ok = np.array([outcome(r["cls"], r[key])[0] for r in rows])
        wrong_bin = sum(1 for r in rows if r["cls"] in ("red", "blue") and r[key].present
                        and r[key].colour in ("red", "blue") and r[key].colour != r["cls"])
        distractor_to_bin = sum(1 for r in rows if r["cls"] in ("green", "yellow") and r[key].colour in ("red", "blue"))
        missed = sum(1 for r in rows if r["cls"] != "none" and not r[key].present)
        ghost = sum(1 for r in rows if r["cls"] == "none" and r[key].present)
        return dict(acc=float(ok.mean()), wrong_bin=wrong_bin, distractor_to_bin=distractor_to_bin,
                    missed=missed, ghost=ghost)

    S_wb, S_nowb = stats("res"), stats("nowb")
    n_rb = sum(1 for r in rows if r["cls"] in ("red", "blue"))
    errs = np.array([r["err"] for r in rows if r["cls"] in ("red", "blue") and r["res"].present])
    errs_np = np.array([r["err_np"] for r in rows if r["cls"] in ("red", "blue") and not np.isnan(r["err_np"])])
    ms = np.array([r["res"].ms for r in rows])
    reproj = np.array([r["res"].reproj_px for r in rows if r["res"].mat_found])
    mat_found = sum(r["res"].mat_found for r in rows)

    # ---------------- figures
    # 1: camera placement in the cell
    cell_img = cell.frame(obj_rgb("red", np.random.default_rng(1)), camera="iso")
    fig, axes = plt.subplots(1, 2, figsize=(12, 3.9))
    axes[0].imshow(cv2.cvtColor(cell_img, cv2.COLOR_BGR2RGB))
    axes[0].set_title("Cell seen from the side: pick mat, bins, camera above the far edge")
    ex = cell.frame(obj_rgb("red", np.random.default_rng(2)))
    dbg = {}
    r_ex = analyse(ex, calib, debug=dbg)
    axes[1].imshow(cv2.cvtColor(draw_overlay(ex, r_ex, dbg), cv2.COLOR_BGR2RGB))
    axes[1].set_title(f"What the camera sees ({W}x{H}, {VISION_CAM_FOVY:.0f} deg vertical field)")
    for ax in axes:
        ax.axis("off")
    f_place = savefig(fig, "e6_camera_placement.png")

    # 2: pipeline steps on one frame (warm, dim light)
    img = cell.frame(obj_rgb("blue", np.random.default_rng(3)), dx=8, dy=-12, yaw=0.4, intensity=0.55, temp=2700)
    dbg_raw, dbg = {}, {}
    analyse(img, calib_nowb, wb=False, debug=dbg_raw)
    r_p = analyse(img, calib, debug=dbg)
    fig, axes = plt.subplots(1, 4, figsize=(15, 3.4))
    axes[0].imshow(cv2.cvtColor(draw_overlay(img, r_p, dbg), cv2.COLOR_BGR2RGB))
    axes[0].set_title("1. markers found, decision")
    axes[1].imshow(cv2.cvtColor(dbg_raw["top"], cv2.COLOR_BGR2RGB))
    axes[1].set_title("2. top-down view (warm, dim light)")
    axes[2].imshow(cv2.cvtColor(dbg["top"], cv2.COLOR_BGR2RGB))
    axes[2].set_title("3. after white balance on the grey mat")
    axes[3].imshow(dbg["mask"], cmap="gray")
    axes[3].set_title(f"4. object mask: {r_p.colour}, hue {r_p.hue:.0f} deg")
    for ax in axes:
        ax.axis("off")
    f_steps = savefig(fig, "e6_pipeline_steps.png")

    # 3: examples grid
    picks = []
    for want in [("red", 2700), ("blue", 2700), ("red", 8000), ("blue", 6500), ("green", 4500), ("none", 5500)]:
        rw = next((r for r in rows if r["cls"] == want[0] and r["temp"] == want[1] and r["img"] is not None), None)
        if rw:
            picks.append(rw)
    fig, axes = plt.subplots(2, 3, figsize=(15, 5.8))
    for ax, rw in zip(axes.ravel(), picks):
        dbg = {}
        rr = analyse(rw["img"], calib, debug=dbg)
        ax.imshow(cv2.cvtColor(draw_overlay(rw["img"], rr, dbg), cv2.COLOR_BGR2RGB)[0:640, 0:1180])
        ax.set_title(f"true: {rw['cls']}, light {rw['intensity']:.2f}x, {rw['temp']} K")
        ax.axis("off")
    f_grid = savefig(fig, "e6_examples.png")

    # 4: hue/saturation scatter with the calibrated centroids
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    colmap = {"red": "#d62728", "blue": "#1f77b4", "green": "#2ca02c", "yellow": "#bcbd22"}
    for ax, key, cal, title in ((axes[0], "nowb", calib_nowb, "without white balance"),
                                (axes[1], "res", calib, "with white balance on the grey mat")):
        for cls, col in colmap.items():
            pts = [(r[key].hue, r[key].sat) for r in rows if r["cls"] == cls and r[key].present]
            if pts:
                hh, ss = np.array(pts).T
                ax.scatter(hh, ss, s=10, color=col, alpha=0.6, label=cls)
        for name, c in cal.colours.items():
            for h0 in (c["hue"], c["hue"] - 360, c["hue"] + 360):
                ax.axvspan(h0 - calib.hue_tol, h0 + calib.hue_tol, color=colmap[name], alpha=0.12)
                ax.axvline(h0, color=colmap[name], lw=1)
        ax.set(xlim=(-40, 360), xlabel="measured hue of the object (deg)", title=title)
        ax.legend(loc="lower right", fontsize=7)
    axes[0].set_ylabel("measured saturation")
    f_hue = savefig(fig, "e6_hue_scatter.png")

    # 5: accuracy vs colour temperature and intensity
    fig, axes = plt.subplots(1, 2, figsize=(12, 3.8))
    temps = sorted(TEMPS)
    for key, lab, ls in (("res", "with white balance", "-o"), ("nowb", "without", "--s")):
        acc_t = [np.mean([outcome(r["cls"], r[key])[0] for r in rows if r["temp"] == t]) for t in temps]
        axes[0].plot(temps, np.array(acc_t) * 100, ls, label=lab)
        bins = np.linspace(*INTENSITY, 7)
        mid = (bins[1:] + bins[:-1]) / 2
        acc_i = [np.mean([outcome(r["cls"], r[key])[0] for r in rows if lo <= r["intensity"] < hi] or [np.nan])
                 for lo, hi in zip(bins[:-1], bins[1:])]
        axes[1].plot(mid, np.array(acc_i) * 100, ls, label=lab)
    axes[0].set(xlabel="colour temperature of the light (K)", ylabel="correct decisions (%)", ylim=(0, 105),
                title="Accuracy against light colour")
    axes[1].set(xlabel="light intensity (x nominal)", ylim=(0, 105), title="Accuracy against light intensity")
    for ax in axes:
        ax.legend(loc="lower right")
    f_acc = savefig(fig, "e6_accuracy_lighting.png")

    # 6: confusion matrices
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9))
    for ax, Cm, title in ((axes[0], C_nowb, "without white balance"), (axes[1], C_wb, "with white balance")):
        ax.imshow(Cm, cmap="Blues")
        for i in range(Cm.shape[0]):
            for j in range(Cm.shape[1]):
                ax.text(j, i, Cm[i, j], ha="center", va="center", color="w" if Cm[i, j] > Cm.max() / 2 else "k")
        ax.set_xticks(range(len(labels_pred)), labels_pred)
        ax.set_yticks(range(len(labels_true)), labels_true)
        ax.set(xlabel="decision", ylabel="true object", title=title)
        ax.grid(False)
    f_conf = savefig(fig, "e6_confusion.png")

    # 7: position error
    fig, ax = plt.subplots(figsize=(7, 3.6))
    hb = np.linspace(0, max(10, np.nanpercentile(errs_np, 99) * 1.1), 40)
    ax.hist(errs_np, hb, alpha=0.5, label=f"blob centre on the table plane (median {np.median(errs_np):.1f} mm)", color="#ff7f0e")
    ax.hist(errs, hb, alpha=0.7, label=f"with parallax correction (median {np.median(errs):.1f} mm)", color="#1f77b4")
    ax.set(xlabel="position error of the object centre (mm)", ylabel="frames",
           title="Image-to-arm localisation error, objects placed up to 20 mm off the cross")
    ax.legend()
    f_pos = savefig(fig, "e6_position_error.png")

    # copies for the design book
    BOOK_IMG.mkdir(parents=True, exist_ok=True)
    for f in (f_place, f_steps, f_grid, f_hue, f_acc, f_conf, f_pos):
        shutil.copy(ROOT / "reports" / f, BOOK_IMG / f.split("/")[-1])
    shutil.copy(ROOT / "vision" / "assets" / "pick_mat_A4_300dpi.png", BOOK_IMG / "pick_mat.png") \
        if (ROOT / "vision" / "assets" / "pick_mat_A4_300dpi.png").exists() else None

    save_json("e6.json", {
        "n": N_TEST, "accuracy": S_wb["acc"], "accuracy_no_wb": S_nowb["acc"], "wrong_bin": S_wb["wrong_bin"],
        "wrong_bin_no_wb": S_nowb["wrong_bin"], "distractor_to_bin": S_wb["distractor_to_bin"],
        "missed": S_wb["missed"], "ghost": S_wb["ghost"], "n_red_blue": n_rb,
        "pos_err_median_mm": float(np.median(errs)), "pos_err_p95_mm": float(np.percentile(errs, 95)),
        "pos_err_max_mm": float(errs.max()), "pos_err_np_median_mm": float(np.median(errs_np)),
        "ms_median": float(np.median(ms)), "ms_p95": float(np.percentile(ms, 95)),
        "reproj_median_px": float(np.median(reproj)), "mat_found": int(mat_found),
        "calib": calib.colours, "calib_no_wb": calib_nowb.colours})

    unknown_rb = sum(1 for r in rows if r["cls"] in ("red", "blue") and r["res"].present and r["res"].colour == "unknown")
    summary = [
        ["Correct decisions, all classes", f"{S_wb['acc']:.1%}", f"{S_nowb['acc']:.1%}"],
        ["Red or blue object sent to the wrong bin", f"{S_wb['wrong_bin']} / {n_rb}", f"{S_nowb['wrong_bin']} / {n_rb}"],
        ["Green or yellow object sent to a bin", str(S_wb["distractor_to_bin"]), str(S_nowb["distractor_to_bin"])],
        ["Object present but not seen", str(S_wb["missed"]), str(S_nowb["missed"])],
        ["Empty mat reported as an object", str(S_wb["ghost"]), str(S_nowb["ghost"])],
    ]
    calib_rows = [[n, f"{c['hue']:.1f}", f"{c['sat']:.2f}", str(c["n"])] for n, c in calib.colours.items()]

    report = f"""# E6 - Computer vision: colour detection and image-to-arm calibration

**Scripts:** `vision/detector.py` (the code the laptop runs), `experiments/e6_vision.py` (this test)

> **Status of the evidence.** The frames are rendered by the physics simulator from a virtual
> {W}x{H} camera, with sensor noise and JPEG compression added, under lighting that varies in
> intensity and colour. They test the algorithm, not the real webcam: the same script must be
> repeated with real frames in the build phase (`uv run python -m vision.calibrate` then `vision.live`).

## Setup

* **Camera:** fixed USB webcam {VISION_CAM_OFFSET[2] * 1e3:.0f} mm above the table and {VISION_CAM_OFFSET[0] * 1e3:.0f} mm beyond the
  pick spot, on the side away from the arm, looking down at the pick spot ({np.degrees(np.arctan2(VISION_CAM_OFFSET[2], VISION_CAM_OFFSET[0])):.0f} deg below horizontal).
  The arm never passes between the camera and the object while it waits at the ready pose.
* **Pick mat:** A4 sheet printed in neutral grey with four ArUco markers ({mat.MARKER:.0f} mm, IDs 0-3) at the corners and a
  cross at the pick spot. It is the calibration target and the white-balance reference at the same time.
* **Pipeline** (`vision/detector.py`): markers -> homography and camera pose -> top-down view at 2 px/mm ->
  white balance on the bare mat -> saturated blob within {60} mm of the cross -> hue -> nearest calibrated colour
  (accepted within {calib.hue_tol:.0f} deg, otherwise "unknown") -> blob centre -> camera ray -> object centre in arm mm.

![Camera placement]({f_place})

![Pipeline steps]({f_steps})

## Colour calibration

Six frames per colour at nominal light, object on the cross. Stored in `vision/calib_sim.json`.

{table(["Colour", "Hue centroid (deg)", "Saturation", "Frames"], calib_rows)}
## Test campaign

{N_TEST} frames, each with a random object (38 % red, 38 % blue, 6 % green, 6 % yellow, 12 % empty mat), placed up to
{OFFSET_MM:.0f} mm off the cross with a random rotation of up to 45 deg, light intensity {INTENSITY[0]}x to {INTENSITY[1]}x nominal,
light colour from 2700 K (warm lamp) to 8000 K (cold daylight), noise {NOISE:.0f} grey levels, JPEG quality {JPEG_Q}.
Expected decision: red -> red bin, blue -> blue bin, green or yellow -> "unknown" (no bin), empty -> "none".

{table(["Result", "Our pipeline", "Same pipeline without white balance"], summary)}
Of the red and blue objects, {unknown_rb} were rejected as "unknown" instead of sorted. "Unknown" is a safe outcome: the
sequencer reads again or leaves the object.

![Accuracy against lighting]({f_acc})

![Confusion matrices]({f_conf})

![Hue and saturation of every object]({f_hue})

The scatter shows why the white balance matters: under warm or cold light the raw hue of the same object drifts by tens of
degrees, and red and yellow, or blue and green, move into each other's band. Dividing by the grey mat brings every object back
next to its calibrated centroid.

![Examples]({f_grid})

## Image-to-arm calibration

* Markers found in {mat_found} / {N_TEST} frames; median reprojection error of the marker corners **{np.median(reproj):.2f} px**.
* Object centre error over the red and blue objects: median **{np.median(errs):.1f} mm**, 95 % below **{np.percentile(errs, 95):.1f} mm**,
  worst {errs.max():.1f} mm. Without the parallax correction (blob centre taken on the table plane) the median is
  {np.median(errs_np):.1f} mm: the camera sees the top and sides of a {P.OBJECT_SIZE * 1e3:.0f} mm object, not its footprint.

![Position error]({f_pos})

## Timing

Median {np.median(ms):.0f} ms per frame, 95 % below {np.percentile(ms, 95):.0f} ms, on the team laptop in Python
({elapsed / N_TEST * 1e3:.0f} ms per test including rendering). One decision is needed per object, so this is not a constraint.

## What this decides

1. A fixed webcam on the far side of the pick spot, with the printed grey mat under the object.
2. White balance on the mat, then classification by hue with a rejection band: lighting changes do not send objects to the wrong bin,
   and unknown colours are refused instead of guessed.
3. Pose from the four markers: the camera can be bumped or re-mounted and only needs the mat in view; no manual re-calibration.
4. The measured object position can correct the grasp when an object is placed off the cross.

## Limits

* Simulated frames: real webcam auto-exposure, lens distortion, motion blur and specular highlights on glossy objects are not
  reproduced. Lens distortion is handled by the intrinsic calibration (`vision.calibrate intrinsics`).
* Object colours are drawn from fixed hue ranges; the real objects' colours are unknown until the kit is delivered and must be
  calibrated with `vision.calibrate colours`.
* Shadows of the arm only appear in the ready pose used here.
"""
    write_report("E6_computer_vision.md", report)
    print(f"accuracy {S_wb['acc']:.3f} (no WB {S_nowb['acc']:.3f}), wrong bin {S_wb['wrong_bin']}/{n_rb}, "
          f"pos median {np.median(errs):.1f} mm, {np.median(ms):.0f} ms")


if __name__ == "__main__":
    main()
