# Design book v0: what is still missing (deadline 16 Nov, 23:59)

Section 6 (computer vision) is now written: `design_book/section6_computer_vision.tex`, figures in `design_book/images/vision/`.
Everything below is still a "[To be completed]" in `DesignBookV0.pdf`, or a mismatch with the spec book's deliverables.

## Blocking: required deliverables in the spec book

| Deliverable (spec book) | State in v0 | Who / with what |
|---|---|---|
| 03 Print-ready files: one STL/3MF per part, named, numbered, with quantity; **orientation, infill, supports, material for each part** | 3.1 lists 11 parts, 6 with sizes; 3.2 print settings empty; no link to the print folder | CAD owner. Without this the FabLab cannot check the files |
| 02 CAD: full assembly with every provided component, exploded views, dimensioned drawings, STEP + native | All 7 images are placeholders (cover, 2.1 assembly, links, 3 exploded views, 2 drawings); CAD folder link missing | CAD owner: screenshots from SolidWorks |
| 04 Simulation supporting the key decisions | 4.1 to 4.3 promise Gazebo results (screenshot, joint plots, clearance, torques) that do not exist. 4.4 (ngspice) is done | See "Simulation" below |
| 05 Code on GitHub with README | 5.1 link and 5.2 code structure empty; no repository exists yet (this folder has no commit) | Create the repo, push, paste the link |
| 06 Explanatory video, 3 to 5 min | Section 7 link empty | Record and upload; link must open for the organisers |
| Submission form: PDF + links to CAD/print folder, GitHub, video | - | Test every link in a private window |

## Numbers that the jury checks

* **1.4.1 Table 4, required torques and margins:** all "-". Needs link lengths and masses from the CAD. J1 (base) is missing
  from the table: it carries no gravity load but must accelerate the arm's inertia.
* **1.4.2 cycle time** of one pick-and-place.
* **1.4.3 Denavit-Hartenberg table and workspace.**
* **1.3.2 pick and bin positions, approach heights.**
* **1.3.3 algorithms:** inverse kinematics method, trajectory generation (colour classification: text ready in the comments
  at the end of `section6_computer_vision.tex`).

## Simulation: our MuJoCo work does not match the v0 design

The experiments E1 to E5 (`reports/`) model the **old** design: CNC shield, belts, the PG518 gearbox on the shoulder and a servo on J4.
v0 uses printed cycloidal 1:14 gearboxes (70 % efficiency, 0.40 N.m at 1.2 A) on J1 to J4, a stepper on the wrist roll,
DS3218MG on J5 and J6 and an MG996R gripper. Once the drives and the CAD dimensions are updated in `arm/params.py`,
the same scripts give, for v0:
Table 4 (torques and margins), the workspace (1.4.3), the cycle time (1.4.2), clearance along the path (4.2), torques
along the trajectory (4.3) and a video of the cycle (4.1). Section 4.1 should then say MuJoCo instead of Gazebo, unless
someone has the Gazebo model running today.

## Smaller items

* 1.3.1 structure and material of the links; 1.6 mechanical bill of materials (bearings, rods, screws, filament).
* 1.8 Team roles: names without roles.
* 2.1 overall dimensions, reach, total mass, base fixing; 2.2 rotor lobes, eccentricity, pins; 2.3 wrist dimensions and
  J5/J6 range; 2.4 jaw stroke, gripping force, **object size range** (the vision assumes a 30 mm object; adjust
  `object_height_mm` in `vision/calib.json`).
* R2 says "home every **stepper** joint"; the spec book says "home every axis". One sentence: the servos are absolute
  (position commanded by pulse width), so they need no homing.
* 1.3.2 cycle: read the colour before the grasp, while the arm is at the ready pose (suggested text in the .tex comments).
* Risk table: update the colour-detection line (suggested text in the .tex comments).
