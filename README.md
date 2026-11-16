# Robotic arm for colour sorting (RAS Build Challenge 2026, Project 3)

Team **engineers over engineering**. Six-axis arm that picks a cube at a fixed spot, reads its colour with a camera and
places it in one of two bins.

This repository holds the **laptop side and the design calculations**: computer vision, kinematics, motion planning, the
physics simulation, and the experiments behind the design book. The firmware, the PCBs and the CAD live elsewhere
(see "What is not in this repository").

> **Status.** The arm dimensions, masses and motor figures used by the simulation are placeholders until the CAD is final.
> The vision code is tested on simulated camera frames only; it has not seen the real webcam yet.

## System overview

```
Team laptop (Python)                              ESP32 master        Joint boards x4 (STM32)
  camera -> colour detection  ->  task sequencer -USB-> CAN master -CAN-> TMC2209 steppers, AS5600 encoders,
            (this repo: vision/)   inverse kinematics                      homing, servo PWM (board 4)
                                   + trajectory (arm/)
```

| Part | Where | State |
|---|---|---|
| Colour detection and object position (`vision/`) | this repo | works on simulated frames; real camera to test |
| Kinematics, trajectories, MuJoCo simulation (`arm/`) | this repo | works; models the earlier design, see "Known gaps" |
| Experiments E1-E6, dashboard, explainer video | this repo | works |
| Task sequencer (vision + IK + serial) | not written | next step |
| ESP32 and STM32 firmware, KiCad project | not in this repo | see the design book, section 5 |
| CAD, print files | not in this repo | see the design book, sections 2 and 3 |

## Setup

Needs [uv](https://docs.astral.sh/uv/) (it installs Python 3.12 and the dependencies into `.venv`).

```bash
uv sync
```

Libraries: NumPy, OpenCV (camera and markers), MuJoCo (physics and rendering), Matplotlib, imageio, Pillow.
No ROS, no GPU, no system packages.

## Computer vision (`vision/`)

One webcam on its own stand looks at the pick spot. A printed A4 **pick mat** under the object carries four ArUco markers
(camera pose), a grey surface (white balance) and a cross (pick spot). Per frame: find the markers, straighten the mat,
white balance on the grey, find the coloured blob, compare its hue with the two stored colours, and compute the object's
x, y, z in the arm's frame. No trained model. Details and results: section 6 of the design book
(`design_book/Section6_ComputerVision.pdf`) and `reports/E6_computer_vision.md`.

| Step | Command |
|---|---|
| Print the mat (100 % scale, A4) | `uv run python -m vision.mat` then print `vision/assets/pick_mat_A4_300dpi.png` |
| Calibrate the lens (chessboard 9x6, 25 mm) | `uv run python -m vision.calibrate intrinsics --camera 0` |
| Calibrate the two colours | `uv run python -m vision.calibrate colours --camera 0 --names red blue` |
| Check the marker error and the position | `uv run python -m vision.calibrate check --camera 0` |
| Viewer / one JSON answer per decision | `uv run python -m vision.live --camera 0` (add `--json`) |
| Test on simulated frames (about 4 min) | `uv run python -m experiments.e6_vision` |

In code, the sequencer does:

```python
from vision.live import Vision
cam = Vision(camera=0)
r = cam.read_stable()     # r.present, r.colour ("red" / "blue" / "unknown" / "none"), r.confidence, r.x_mm, r.y_mm, r.z_mm
```

Calibration is saved in `vision/calib.json` (commit it once it is made at the workshop). `vision/calib_sim.json` is the
simulation's.

## Simulation and design calculations

```bash
uv run python -m experiments.run_all          # E1 to E5, about 90 s
uv run python -m experiments.run_all e1 e4    # a selection
uv run python -m experiments.e6_vision        # E6, vision (separate, slower)
uv run python -m dashboard                    # then open http://127.0.0.1:8765
```

The dashboard edits the design values (`arm_config.json`), runs the experiments, shows the reports and has a "model" page
with the arm drawn joint by joint. Everything regenerates from the saved values.

| # | Report | Question | Result with the current values |
|---|---|---|---|
| E1 | [Kinematics and workspace](reports/E1_kinematics_workspace.md) | Is the IK right? Are the pick spot and bins reachable? | IK exact on 2000 poses; all task poses reachable |
| E2 | [Torque and gearing](reports/E2_torque_gearing.md) | Which motor, which reduction? | Safety factors 3.1 (shoulder) and 2.7 (elbow) for the old belt design |
| E3 | [Motion timing](reports/E3_motion_timing.md) | Cycle time, step rate | 6.5 s per object; 3.8 kHz of step pulses |
| E4 | [Pick-and-sort simulation](reports/E4_pick_place_simulation.md) | Do all six axes move and does the cycle work? | Both cubes sorted; peak torque use 45 % |
| E5 | [Power budget](reports/E5_power_budget.md) | Is the supply enough? | 29 W running, 84 W worst fault |
| E6 | [Computer vision](reports/E6_computer_vision.md) | Is the colour right under different lights? | 100 % correct on 600 simulated frames, 0 wrong bins |

## Layout

| Path | Content |
|---|---|
| `vision/` | `mat.py` printed mat, `detector.py` one-frame processing, `calibrate.py`, `live.py` |
| `arm/` | `params.py`, `config.py` design values; `kinematics.py`, `dynamics.py`, `task.py`; `mjcf.py`, `sim.py` MuJoCo |
| `experiments/` | one script per experiment (`e1` to `e6`), `run_all.py` |
| `dashboard/` | local web dashboard (standard library only) |
| `video/` | builds the narrated explainer video (`python -m video.build`, Windows voices) |
| `design_book/` | section 6 of the book (`.tex`, figures, compiled pages) and `MISSING.md`, the list of what the book still lacks |
| `reports/` | generated reports, figures, data |

## Known gaps

* **The simulation models the earlier design** (belts, PG518 gearbox, CNC shield). The v0 design book uses 1:14 printed
  cycloidal gearboxes, a stepper on the wrist roll, and joint boards. Update the drives and dimensions in `arm/params.py`
  and rerun E1-E5 for the numbers of the final design.
* Vision tested on simulated frames only. Real webcam, lens, lighting and cube colours are not tested.
* The grasp is simulated as an attachment (no finger force). No self-collision check.
* The task sequencer (vision + inverse kinematics + serial link to the ESP32) is not written yet.

## What is not in this repository

Joint-board and master firmware, KiCad project, wiring and pinout, CAD and print files, Gazebo model. Add links here when
they exist.

## Explainer video

`video/` builds a 13-minute narrated video that explains the project to a software engineer
(`uv run python -m video.build`, about 20 MB, not stored in git).
