# Portfolio content: Vision-Based Haptic Navigation (BU Robotics Lab)

Brief for the chat building the portfolio page. All numbers come from the `haptic-teleop` repo: README, `docs/haptic_testing/README.md` and `System Architecture.md`. Items marked **[CONFIRM]** need a check from Ellen before they're published.

---

## 0. Design principles, from portfolio guidance and employer surveys

1. **Skimmable in 30–60 s.** Reviewers spend about that long per project, so lead with visuals and a summary card. Use paragraphs only on demand, in expandable sections. *(MIT CommLab)*
2. **Problem → approach → result → lesson**, with *my role* stated explicitly. *(The Construct, UX Planet, UMass)*
3. **Show the thinking, including failures.** Documented debugging and decision points are what set a project apart from a tutorial. *(UMass, ASU London)*
4. **Quantify outcomes.** Use numbers rather than adjectives.
5. **Make the competencies visible.** Employers rank critical thinking and communication as top competencies, but only about 56% rate new grads as proficient in critical thinking (NACE Job Outlook 2025). The page should *prove* those two with evidence, not claim them.

---

## 1. Summary card (top of page)

**Title:** Vision-Based Haptic Navigation
**One-liner:** I made a robot's ordinary camera reliable enough to warn its remote driver, through a force they feel in their hand, before a collision.

| | |
|---|---|
| **Who** | Ellen Zheng, undergraduate researcher. Advisors: Prof. Roberto Tron, Dr. Dawei Zhang |
| **Where** | Boston University Robotics Lab (UROP) |
| **When** | Oct 2025 – Oct 2026; presented at the BU UROP Symposium, 16 Oct 2026 |
| **What** | A haptic shared-control system for a ground robot. One camera estimates obstacle distance, a safety controller brakes the robot, and a force-feedback stylus lets the operator *feel* each correction. |
| **How** | ROS 2 · ZoeDepth monocular depth network · control barrier function (CBF) safety filter · Geomagic Touch haptic device · AgileX LIMO robot · Python, C++ |
| **Why** | Remote drivers misjudge distance through a flat video feed and crash. Full autonomy takes them out of control. Haptic feedback keeps the human in charge while warning them. Prior haptic CBF work (Zhang et al. 2020) assumes obstacle positions are *known*. I closed that loop with real perception from a cheap camera. |

**My contribution (badges or bullets):**
- Built the full pipeline end to end: 4 ROS 2 nodes plus visualization and validation tools.
- Designed a ground-truth validation that exposed a hidden calibration error, and **cut depth error 8×** (18.7 → 2.3 cm RMSE).
- Diagnosed and patched 3 bugs in the haptic device's C++ driver, plus a mm/m units bug.
- Ran staged human-in-the-loop tests and **found the force level operators actually notice** (0.8 N).
- Found and fixed 2 safety hazards before hardware testing.

**Impact line:** The safety margin is 30 cm. Before my calibration fix, depth error could eat 13.5 cm of it (median); after the fix, about 2 cm. That is the difference between a camera that *can* and *cannot* be trusted to drive a safety filter.

---

## 2. Page sections (visual first)

### 2.1 Hero: demo media
- **Visual:** a 10–20 s looping muted video or GIF. Split screen: camera view with depth overlay (from the `overlay` node) next to a hand on the stylus being pushed back. Caption: "Obstacle crosses 30 cm → robot brakes → stylus pushes back."
- **[CONFIRM]** Footage needs recording. Fallback: a photo of the LIMO + Touch setup, or the poster.

### 2.2 The problem in one graphic
Use three side-by-side panels instead of a paragraph:
1. **Teleoperation:** "Through a camera, distance is hard to judge → collisions."
2. **Full autonomy:** "Robot takes over → the human loses control."
3. **Haptic shared control (this project):** "Human drives; the robot pushes back only when needed."

Footnote under panel 3: "Existing systems are *told* where obstacles are. Mine has to *see* them with one camera."

### 2.3 How it works: interactive pipeline diagram
**Visual:** a flow diagram, left to right on desktop and top to bottom on mobile.

```
Camera → Depth AI (ZoeDepth) → Nearest obstacle → Safety filter (CBF, 20 Hz) → Robot
                                                        ↓ correction
Operator ← Force on stylus (Geomagic Touch) ←───────────┘
Operator → stylus motion → speed request → Safety filter
```

**Interactivity:** click or tap a node to open a popover with three lines: *What it does / Why it matters / What I did.* Animate dots along the edges to show the data flow (respect `prefers-reduced-motion`).

| Node | What it does | Why it matters | What I did |
|---|---|---|---|
| Camera + ZoeDepth | Estimates distance from a single image | No expensive or glass-blind depth sensor needed | Integrated the model, built the calibration, cropped the floor out of the image |
| Safety filter (CBF) | Limits approach speed as the gap shrinks: `cos(α)·v ≤ γ·(d − 0.30 m)` | Smooth braking instead of an on/off cliff | Implemented a closed-form solution (no optimizer) at 20 Hz, plus a 0.2 s command timeout |
| Force | `F = k_f·(v_safe − v_requested)`, scaled and clamped to 0.8 N | You feel *only* the correction; a safe command gives zero force | Tuned the scale and clamp through human testing |
| Stylus driver | Sends force to the device at 1 kHz | Without the fixes, the force was silently dropped or could reach ~20 N | Patched 3 driver bugs in C++ |

### 2.4 Interactive "try the safety filter" widget (highest-value interactive)
Shows the core idea in 10 seconds of play.

- **Inputs:** a slider for **Obstacle distance d** (0.20–1.50 m) and a slider for **Requested speed v_ref** (0–0.30 m/s). Assume the obstacle is straight ahead (α = 0).
- **Computation:**
  ```
  r = 0.30, γ = 0.8, kf = 0.5
  v_safe   = min(v_ref, γ·(d − r))                   // negative inside 0.30 m means the robot backs away
  F_ctrl   = clamp(kf·(v_safe − v_ref), −0.3, 0.3)   // newtons, controller side
  F_stylus = clamp(8·F_ctrl, −0.8, 0.8)              // newtons, at the hand
  ```
- **Outputs:** a top-down robot-and-box sketch with a 30 cm red ring around the box. Show two speed bars (requested vs allowed), and a force arrow on a stylus icon whose length is proportional to |F_stylus|. Add a text readout like "Allowed 0.04 m/s · Force 0.64 N".
- **Sanity check:** at d = 0.30 and v_ref = 0.20, the readout should show allowed 0, F_ctrl = −0.1 and F_stylus = −0.8 N, which matches the recorded test (braked 0.20 → −0.01 m/s).
- Label it "simplified (straight-ahead obstacle)".

### 2.5 Timeline: interactive, color-coded by phase
**Visual:** a horizontal stepper on desktop and a vertical list on mobile. Clicking a milestone expands 1–2 lines. Highlight the **pivot** (Sep 2026) with an accent color. Dates come from commit history. **[CONFIRM]** the start date and what happened between May and Aug 2026; omit that gap or label it "summer".

| Date | Phase | Milestone |
|---|---|---|
| Oct 2025 | Setup | Joined lab; project repo started |
| Nov 2025 | Build | ROS 2 + OpenCV + LIMO camera integration |
| Feb 2026 | Build | Camera node; ZoeDepth running on GPU / Apple MPS; first depth calibration |
| Apr 2026 | Build | Linear calibration (Apr 1) → CBF safety filter (Apr 27) → haptic teleop + driver patch (Apr 29–30) |
| **Sep 22, 2026** | **Validate (pivot)** | **Tape-measure validation reveals depth error growing with distance** |
| Sep 30, 2026 | Fix | Quadratic calibration: 18.7 → 2.3 cm RMSE |
| Oct 7, 2026 | Test | Human-in-the-loop haptic tests; dead-man switch + stop-on-exit added |
| Oct 16, 2026 | Communicate | BU UROP Symposium poster |
| Next | — | Live robot trials, steering around obstacles |

### 2.6 Result 1: calibration (lead result)
**Visual A (simple bar chart):** RMSE for each method, with a dashed line at the 30 cm safety margin for scale.

| Method | Median error | RMSE |
|---|---|---|
| Linear fit (calibrated against the onboard IR sensor) | 13.5 cm | 18.7 cm |
| **Quadratic fit (tape-measured)** | **1.0 cm** | **2.3 cm** |
| IR sensor itself (matte box) | 0.8 cm | 1.5 cm |

**Visual B (optional, stronger):** error vs true distance (0.30–1.50 m, 7 points), with lines for linear and quadratic. It shows the linear error *growing* with distance while the quadratic stays flat. Add a toggle: Linear / Quadratic / Both.
**[CONFIRM]** Per-point values aren't saved in the repo. Get them by re-running `depth_estimation_pkg/validate_gt.py`, which prints a per-position table. Don't estimate the points.

**Caption (the story):** "My first calibration looked fine against the robot's IR sensor. Testing against a tape measure over the full 0.3–1.5 m range showed the error was curved, not random: the IR sensor reads through transparent objects, and my first check stopped at 0.77 m. A quadratic model fixed it. I also kept the curve monotone, so a closer object can never read as farther away."

### 2.7 Result 2: does the operator feel it?
**Visual A:** the existing plot `docs/haptic_testing/stage2c_force_f08.png` (force at the stylus over time; shaded where the filter is active). Caption: "Force at the hand tracks the controller exactly (measured 2.98 for a 3× gain). It never pushed the wrong way."

**Visual B: interactive "how strong is 0.8 N?" scale.** A horizontal scale from 0 to 0.8 N with markers from the tests. Hovering or tapping a marker shows the operator's verdict and a gram-weight equivalent (N ÷ 9.81).

| Force | ≈ weight held | Verdict from tests |
|---|---|---|
| 0.05 N | 5 g (about a nickel) | Lost in device friction |
| 0.15 N | 15 g | Faint |
| 0.30 N | 31 g | Held against without noticing |
| 0.48 N | 49 g | Felt, but too weak as a cue |
| **0.80 N** | **82 g** | **Clearly felt; operator's hand pushed back; preferred; still possible to override** |

### 2.8 Problem solving: debug log (expandable cards)
Each card follows Symptom → Diagnosis → Fix → Lesson. Collapsed, a card shows the symptom only, as a hook.

| Symptom | Diagnosis | Fix | Lesson |
|---|---|---|---|
| Stylus shook violently; force always maxed out | Driver reports **mm**, the code assumed **m**: a 5 cm push became a 250 m/s command | Converted units at the boundary | Check units and frames end to end on day one |
| Force sometimes did nothing at all | Touch driver dropped external force unless "lock" mode was on, and never zeroed it between cycles | Patched C++ driver; added a 0.8 N hard clamp (one gain could request ~20 N on a ~3.3 N-peak device) | Don't trust a driver; test it with synthetic inputs |
| Calibration looked good, but distances were off far away | The reference sensor was fooled by transparent objects, and the test range was too short | Independent tape-measure ground truth over the full range; quadratic model | Validate against independent ground truth over the operating range |
| Docked stylus = robot full speed ahead | The dock position maps to the maximum speed command | Dead-man switch: the robot moves only while a button is held | Hunt for failure modes before hardware |
| Ctrl-C didn't stop the robot | ROS shut down *before* the stop command was sent | Nodes send zero command and zero force before exiting; 0.2 s command timeout | Shutdown is part of the safety design |

### 2.9 Skills shown (map to what employers look for)
**Visual:** a compact grid of tags. Clicking a tag highlights the matching card or section above.

| Competency | Evidence on this page |
|---|---|
| Problem identification | Spotted the gap: haptic safety systems assume known obstacles |
| Research design | Staged testing: synthetic → recorded → human-in-the-loop; independent ground truth |
| Critical thinking | Recognized the error pattern as model mis-specification, not noise |
| Debugging | Driver, units and safety bugs (debug log) |
| Technical breadth | ROS 2, deep learning inference, control theory, C++ device drivers, human testing |
| Safety mindset | Dead-man switch, timeouts, monotone calibration, force clamp |
| Communication | Symposium poster; talking points at three audience levels; documentation |
| Independence | **[CONFIRM]** Ellen's scope vs advisors' (e.g., "sole developer, advised weekly") |

### 2.10 Honest scope + next steps (small, near the bottom)
Stating these builds credibility; keep them short.
- Tested on **recorded** camera data with a live human on the stylus. Live robot trials are next (lab wifi was too slow for live inference).
- Pilot testing with one operator; not a controlled user study yet.
- Brakes only. Next: collision-cone CBFs so the robot can steer around obstacles; a guard against stale depth data.

### 2.11 Materials and links
- Poster PDF (BU UROP Symposium 2026) **[CONFIRM file]**
- GitHub: `ezheng05/haptic-teleop` **[CONFIRM the repo is public before linking]**
- Haptic test write-up and plots: `docs/haptic_testing/`
- System architecture walkthrough: `System Architecture.md`

**References**
- D. Zhang, G. Yang, R. P. Khurshid. Haptic teleoperation of UAVs through control barrier functions. *IEEE Trans. Haptics*, 2020.
- A. D. Ames et al. Control barrier functions: theory and applications. *ECC*, 2019.
- S. F. Bhat et al. ZoeDepth: zero-shot transfer by combining relative and metric depth. arXiv:2302.12288, 2023.

---

## 3. Build notes for the portfolio chat
- **Responsive:** pipeline goes left-right → top-bottom and timeline goes horizontal → vertical below ~700 px. Charts fill the container width; tables collapse into cards on mobile.
- **Interactives**, in priority order: (1) safety-filter widget, (2) pipeline popovers, (3) calibration toggle, (4) force scale, (5) timeline expand, (6) skill-tag highlighting. If time is short, ship 1–3.
- **Accessibility:** every chart gets a 1-sentence text takeaway. Popovers must be keyboard-reachable. No information conveyed by color alone. Honor `prefers-reduced-motion`.
- **Colors:** one accent for "my contribution / pivot", one for "safe" vs "intervention". Keep it consistent across charts.

## 4. Wording guardrails (don't overclaim)
- Say "safety filter", not "guarantees safety". The filter is provably safe only with perfect depth, and the pipeline isn't.
- Say "pilot human-in-the-loop test", not "user study".
- Say "close to the IR sensor's accuracy" (2.3 vs 1.5 cm RMSE), not "matches".
- The symposium figure (0.11 N, braking result) was recorded with the *linear* calibration.
- Use the citation Zhang, **Yang** & Khurshid 2020. **[CONFIRM]** whether Dr. Dawei Zhang (advisor) is the first author; if so, say "extends my advisor's UAV method to a ground robot with real perception", which is a strong framing.
