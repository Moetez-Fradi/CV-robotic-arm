# Section 6 (computer vision): how to merge into the design book

Files in this package
* `section6_computer_vision.tex`  the text of section 6 (from "The vision part uses ..." to the end). Paste it right after
  `\section{Computer vision}`, replacing the four "[To be completed]" blocks (6.1 to 6.4). It has its own `\subsection`s.
* `images/vision/`  the figures it uses. Copy the folder into the book's `images/` folder (paths are `images/vision/...`).
* `Section6_ComputerVision.pdf`  how the pages should look (compiled with the book's header style, pages 18 to 21).
* `section6_standalone.tex`  the wrapper that produced that PDF (only for reference).

Packages needed: `graphicx` and `booktabs` (the book's tables look like booktabs, so probably already loaded).
Table and figure numbers follow automatically. In the stand-alone PDF they start at Table 12 / Figure 15 because the V0 book ends
at Table 11 / Figure 14.

Also change elsewhere in the book (suggested wording is in the comments at the bottom of `section6_computer_vision.tex`):
* 1.3.2 Motion strategy: the colour is read before the grasp, while the arm waits at the ready pose.
* 1.3.3 Algorithms: add the one-paragraph description of the colour classification.
* 1.6 Bill of materials: camera line "USB webcam + printed A4 pick mat + LED light". The pick mat to print is
  `images/vision/pick_mat.png` (A4, print at 100 %).
* 1.7 Risks: update the colour-detection line.
* Section 4.1 says Gazebo with ROS 2, but the vision code is plain Python: make sure the book is consistent.

Code: https://github.com/Moetez-Fradi/CV-robotic-arm  (README explains how to run everything).
