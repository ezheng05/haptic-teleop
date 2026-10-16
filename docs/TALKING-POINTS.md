# Talking points: UROP Symposium, Fri 16 Oct

**An Implementation of Vision-Based Haptic Navigation**, BU Robotics Lab, advised by Prof. Roberto Tron and Dr. Dawei Zhang.

The same story at three depths. Choose the version from the first question someone asks, not from how they look.

**Open with this every time:**
> I'm making a robot's camera good enough to warn the person driving it, through a force they feel in their hand, before they hit something.

---

## 1 · General public (~45 seconds)

- "Imagine driving a robot from another room, only seeing through one small camera. It's really hard to tell how far away things are, so people crash."
- One fix: let the driver *feel* obstacles. They steer with a little robotic pen (point at the stylus), and when the robot gets too close to something, the pen pushes back. You stay in charge; it's a warning, not a takeover.
- The catch is that the robot has to know where obstacles are. The usual sensors cost a lot or get fooled: infrared depth sensors can't see glass or clear plastic.
- So I use a normal camera plus an AI model that estimates distance from a single picture.
- **The interesting part (Fig. 3):** the AI's distance estimates were off by about 13 cm at first, which is a lot when the safety bubble is 30 cm. I checked it with a tape measure, found the pattern in the error, and corrected it to about 2 cm.
- **Then (Fig. 5):** when something comes within 30 cm, the system slows the robot to a stop and pushes back on your hand.
- **And I've tested it on the real pen.** A gentle push was too subtle to notice. At 0.8 newtons, a bit less than the weight of a deck of cards, it's clearly felt, and you can still push through it if you need to.

**If they ask...**
- *Why does this matter?* Robots for inspection, disaster response, hospitals and warehouses are often driven remotely, and a cheap camera is much easier to add than a specialized sensor.
- *Why not let the robot drive itself?* Many situations still need human judgment. This keeps the person in control and just warns them.

---

## 2 · Undergraduates in robotics / ECE / CS (~90 seconds)

**Problem.** Haptic shared autonomy: the operator steers, and a safety controller pushes back through a force-feedback device when they're about to collide. Most systems that do this are *told* where obstacles are, by a map or motion capture. A real robot has to perceive them.

**Approach (walk Fig. 1 top to bottom).** A ROS 2 pipeline on an AgileX LIMO.
1. **Perception.** ZoeDepth turns one RGB image into a depth map in metres. I take the nearest point, ignoring the image edges and the bottom of the frame so it doesn't pick the floor.
2. **Safety filter.** A control barrier function. Barrier = distance − 30 cm, positive when you're safe. The rule `cos(α)·v ≤ γ·b` means the speed you're allowed to approach at shrinks as the margin shrinks, so you glide to a stop at the boundary instead of hitting a wall. Runs at 20 Hz.
3. **Force.** Whatever the filter takes away from your command becomes a force on the stylus, `F = kf·(v_safe − v_ref)` (Zhang et al. 2020). If your command is already safe, you feel nothing.

**Results.**
- **Fig. 3 is the main finding.** I first calibrated the network against the robot's own infrared sensor, and it looked fine. Then I measured a box at seven tape-measured distances. The linear calibration was off by 13.5 cm, and the error grew with distance: 8 cm at 0.3 m, 39 cm at 1.5 m. Two things hid it: the IR reference reads through transparent and thin objects in the cluttered scene, and my first check only went out to 0.77 m. The relationship is curved, so no straight line can fix it. A quadratic brings it to **2.3 cm**, and that's what the pipeline runs now.
- **Fig. 5.** The filter is silent until the obstacle crosses 30 cm, then brakes 0.20 m/s down to zero.
- **Fig. 4.** The IR sensor is more accurate on matte surfaces (0.8 cm) but fails on transparent and thin objects. That's where a camera earns its place.
- **Stylus tests (7 Oct, not on the poster).** Geomagic Touch in the loop, recorded approach as input. Force arrives at exactly the commanded size (measured 2.98 for a 3× gain), always in the right direction. Up to 0.5 N was too weak to be a useful cue. 0.8 N is clearly felt, and I preferred it.

**What went wrong (tell one of these if they're into it; this is where most of the time went).**
- **Units bug:** the stylus driver reports millimetres, the code assumed metres. A 5 cm push became a 250 m/s command, the force was constantly maxed out, and the stylus shook violently.
- **Driver bugs:** the Touch driver silently dropped my force unless a "lock" mode was on, never zeroed force between cycles, and one gain could request ~20 N from a device rated ~3.3 N peak. I patched the C++ driver.
- **Safety bugs found in testing:** the docked stylus maps to *full forward speed*, so I added a dead-man switch (robot only moves while a button is held). And Ctrl-C used to shut ROS down *before* the stop command went out.

**What I learned.** Test against an independent ground truth over the *full* range, and check units and frames end to end on day one.

**Next.** Hardware-test the dead-man switch and stop-on-exit, then drive the live robot with the stylus. Then collision-cone CBFs so the robot can steer around obstacles, not just brake.

**If they ask...**
- *Why a CBF and not "stop if closer than X"?* A threshold is an on/off cliff. The CBF slows you smoothly and only as much as needed, and that smooth correction is what makes a useful force signal.
- *Why one camera, not LiDAR or a depth camera?* Cheaper and lighter, and I wanted to know whether learned depth is good enough for safety. With calibration, 2.3 cm is small next to a 30 cm margin.

---

## 3 · PhD students / professionals (~2–3 minutes, then let them steer)

**Framing.** CBF-based haptic shared control, after Zhang, Yang & Khurshid (IEEE ToH 2020) for UAVs, which assumes known obstacle geometry. I close the perception loop on a ground robot with monocular depth, and quantify how perception error eats into the safety margin.

**System.**
- ZoeDepth (NYU+KITTI weights) → nearest point in a cropped ROI (border margin, lower part excluded for the ground plane) → bearing via the pinhole model.
- Barrier h = d − r_safe, r_safe = 0.30 m. With unicycle kinematics ḣ ≈ −cos(α)·v, so ḣ + γh ≥ 0 gives one affine constraint cos(α)·v ≤ γ(d − r_safe), γ = 0.8.
- Min-norm QP with one affine constraint → closed-form projection, no solver. 20 Hz, decoupled from the depth rate.
- F = K_f(u* − u_ref), K_f = 0.5, clamped at 0.3 N in the filter. Device side: ×8, low-pass, 0.8 N hard clamp (Touch rated ~0.88 N continuous) → 1 kHz driver loop. I patched `omni_common`: force was gated behind lock mode, never zeroed per cycle, and lock gain could command ~20 N.
- **Fail-safes:** dead-man switch on the stylus button; 0.2 s command timeout in the filter. Subtle point: inside the barrier the filter maps u_ref = 0 to a *reverse* command, so loss of operator input must mean "stop", not "zero reference". Both nodes send zero command / zero force on SIGINT/SIGTERM before ROS shuts down.

**Results. Lead with calibration; it's the most defensible contribution.**
- Initial linear fit regressed against on-board IR: 19 frames, cluttered scene, nearest-point range 0.20–0.77 m.
- Independent validation: matte box at 7 tape-measured positions, 0.30–1.50 m.
  - IR vs tape: 0.8 cm median, 1.5 cm RMSE. A good reference on cooperative surfaces.
  - Linear ZoeDepth: 13.5 cm median, 18.7 cm RMSE, with **monotonic distance-dependent bias** (+15 cm near, −39 cm at 1.5 m). That signature is model mis-specification, not noise.
  - Quadratic refit: 1.0 cm median, 2.3 cm RMSE.
- **Deployment detail:** the parabola's vertex is at raw ≈ 1.21. Below it the map turns back up, so a closer object would read *farther*, the unsafe direction for a minimum-seeking detector. Below the vertex I continue with the linear slope so the map stays monotone.
- **Effect on the controller:** in simulation the linear calibration engages the barrier ~11 cm late at 0.12 m/s closing speed. On the recorded approach the quadratic gives a graded ~6 s force ramp to the boundary, against ~2 s with the linear one.
- Closed loop (Fig. 5, recorded with the *linear* calibration): silent above r_safe; at the crossing u_ref = 0.20 → u* ≈ −0.01 m/s, peak 0.11 N controller-side, active 29 % of the run.
- **Human in the loop (7 Oct, not on the poster):** Touch, recorded approach as input, operator holding u_ref ≈ 0.20 m/s.
  - Fidelity: device force = f_scale × controller force (ratio 2.98 at f_scale 3). Direction never reversed.
  - ~0.05 N is lost in device friction. 0.3 N was held against with no change in u_ref (0.199 ± 0.002 m/s).
  - At 0.8 N (f_scale 8): clearly felt and preferred. u_ref varied ~5× more, the force visibly displaced the hand before the operator compensated, and mean u_ref *rose*, consistent with bracing.
  - Depth-noise ripple ±0.02 N on 0.30 N (~7%), felt as "mostly steady". No depth filtering needed yet.
  - Onset: near the boundary the CBF slows approach with time constant 1/γ = 1.25 s, so the force creeps in rather than stepping, which is harder to notice. In this test the ramp took ~8.5 s because the target was slid by hand at ~4 cm/s; I expect 3–4 s live.
  - At f_scale 8 the request (~0.9 N) hits the 0.8 N clamp before the boundary: a growing warning that turns into a flat push. Whether that, a steeper onset (higher K_f or γ), or a fully graded ramp is the better cue is open.

**Limitations (say these before they're asked).**
- The quadratic is 3 parameters fit to 7 points, one scene, one object, one camera. It needs held-out validation.
- Depth runs at ~1.3 Hz against a 20 Hz filter, so the filter acts on a held, stale value, and if `depth_node` dies it keeps using the last one. There's no depth uncertainty in the barrier.
- Single nearest point, static obstacle, only v constrained: it brakes but never steers.
- One operator (me), recorded perception, qualitative reports. Not a user study, not yet on the live robot (wifi streaming to the desktop GPU was too slow for live inference).
- No passivity or stability analysis of the haptic loop: 20 Hz force into a 1 kHz device loop, smoothed by the low-pass.

**Next.** Hardware-test the fail-safes, then live closed-loop trials on the LIMO. Tune the force shape. Collision-cone CBFs so the safe command can steer, turning resistance into guidance. Margins from calibration error (or a probabilistic barrier) plus a stale-data watchdog. Then a proper user study: collisions, completion time and workload, with and without force, across force profiles.

---

## Likely questions, with honest answers

**"Is it provably safe?"**
The filter is: given exact h, the CBF condition makes the safe set forward-invariant. The *pipeline* isn't, because h comes from a measurement with error and latency. That's why I quantified the error: it tells you how much margin r_safe has to absorb. With the linear calibration, 13.5 cm of a 30 cm bubble went to error. With the quadratic, ~2 cm.

**"Why is the error curved?"**
I haven't established the mechanism, so I'd say that. My hypothesis is that ZoeDepth's metric scale is learned for the training cameras' intrinsics, and the LIMO camera differs, which needn't give a pure scale error. A cleaner test would refit with a different camera or object.

**"Why not just use the IR sensor?"**
On matte surfaces it's better: 0.8 cm against ~1 cm. But it fails on transparent and thin objects, and a camera is cheaper and more common. The natural next step is fusing the two, not choosing one.

**"Why ZoeDepth?"**
I benchmarked three monocular models. ZoeDepth was the one with metric output, which a controller needs. Relative-depth models give an ordering, not metres.

**"Why not a QP?"**
With one affine constraint the QP solution *is* the projection. A QP becomes necessary with multiple obstacles or input limits.

**"How fast is it? / Latency?"**
Control runs at 20 Hz; depth at ~1.3 Hz on the lab GPU, which is the bottleneck. The filter uses the last depth value, so depth latency directly shrinks the effective margin. One reason live testing is next.

**"Does the force reflect safety, or just the gap from what I asked for?"**
It reflects how much the filter intervened, which is what the operator can act on. It's zero whenever their command is already safe.

**"What does it feel like?"**
Below about 0.5 N it's lost in the device's own friction: you can hold against 0.3 N without noticing. At 0.8 N it's clearly a push, and you can still override it, which is what you want from shared control. The onset is gradual, which is honestly harder to notice than a sudden force.

**"The poster says 0.11 N but you felt 0.8 N?"**
0.11 N is what the controller asks for. The device side scales it by 8 and clamps at 0.8 N because the raw controller force was too weak to feel. Tuning that scale was the point of the stylus tests.

**"Why 0.30 m?"**
Roughly the robot's footprint plus margin. More importantly, it has to be several times the perception error. With the old calibration it wasn't; with the new one it is.

---

## Things to be careful about

- **Fig. 5 and the 0.11 N number were recorded with the linear calibration.** If asked, say so. The deployed quadratic engages earlier and more gradually.
- **The stylus results aren't on the poster.** They're your newest, most tangible result, so bring them up yourself. Have `docs/haptic_testing/stage2c_force_f08.png` on your phone.
- **The equation in "1 · Perception" on the poster is the original linear fit.** The pipeline runs the quadratic. Frame it as the story: "I started with a linear fit; validation showed it was wrong; Result 1 is the fix."
- **"Close to the depth sensor"** is the honest phrasing: 2.3 cm against IR's 1.5 cm RMSE. Don't say "matches."
- **Two ramp numbers, two contexts:** ~6 s is the recorded approach (Fig. 5 data, quadratic); ~8.5 s is the stylus test with a hand-slid box. Don't mix them.
- **Citation:** the repo cites Zhang, Yang & Khurshid, IEEE ToH 2020. Use that name list.
- **Motivation says "time-of-flight."** If the LIMO's depth camera is an Orbbec structured-light or active-stereo unit, say "infrared depth camera" out loud instead.

## Numbers to have ready

| | |
|---|---|
| Safety radius | 0.30 m |
| Filter / depth / device rates | 20 Hz / ~1.3 Hz / 1 kHz |
| Depth error, linear (vs tape, 0.3–1.5 m) | 13.5 cm median, 18.7 cm RMSE |
| Depth error, quadratic | 1.0 cm median, 2.3 cm RMSE |
| IR vs tape (matte) | 0.8 cm median, 1.5 cm RMSE |
| Braking at the boundary | 0.20 → −0.01 m/s |
| Filter params | γ 0.8 (1/γ = 1.25 s), K_f 0.5 |
| Force scale / clamp | 8× / 0.8 N (Touch ~0.88 N continuous, 3.3 N peak) |
| Perceptibility | ~0.05 N lost in friction · up to 0.5 N too weak · 0.8 N clearly felt |
| Force fidelity | 2.98 measured for 3× gain |
| Depth-noise ripple | ±0.02 N on 0.30 N (~7%) |
| Command timeout | 0.2 s |
