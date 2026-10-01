## System Architecture

The full data flow is:

```
Geomagic Touch hardware
    ↓ position (mm)
omni_state.cpp  →  /phantom/state (OmniState)
    ↑ force (N)
/phantom/force_feedback (OmniFeedback)
    ↑
haptic_teleop.py  →  /cmd_vel_ref (Twist)
    ↑                      ↓
/haptic/force (WrenchStamped)
    ↑
cbf_node.py  →  /cmd_vel (Twist)  →  LIMO robot
    ↑
/depth/closest (PointStamped)
    ↑
depth_node  ←  camera
```

The haptic device does two things: it **inputs** (you move the stylus → robot command) and **outputs** (the CBF pushes back on the stylus when you're near an obstacle).

---

## omni_state.cpp — the hardware driver

This is the lowest-level piece. It runs two threads:

**Thread 1: HD scheduler (1 kHz)**
`omni_state_callback()` is called by OpenHaptics at 1000Hz. Each call:
- Reads the transform matrix from the device (4×4 homogeneous matrix with position + rotation)
- Extracts position with axis swap: `position = (X, -Z, Y)` — this reorients from device coords to ROS-friendly coords
- Estimates velocity using a 2nd-order backward difference + low-pass Butterworth filter (cutoff 20Hz) to smooth out noise
- Extracts the quaternion orientation
- Reads button states
- Writes force to the device via `hdSetDoublev(HD_CURRENT_FORCE, feedback)`

**Thread 2: ROS executor**
`publish_omni_state()` runs at the configured rate (1000Hz per the launch file), publishing:
- `/phantom/state` — full OmniState (position in mm, quaternion, velocity, buttons)
- `/phantom/pose` — PoseStamped (position in meters, for tf broadcasting)
- `/phantom/joint_states` — joint angles for rviz visualization
- `/phantom/button` — button events when state changes

The `force_callback` runs when something publishes to `/phantom/force_feedback`. It sets `method_force` which the scheduler callback can apply to the device.

**What I fixed in omni_state.cpp:**

Before my fix, the force logic was:
```cpp
for (int i = 0; i < 3; i++) {
    float lock_force = ...;  // spring toward lock_pos
    if (omni_state->lock[i]) {
        omni_state->force[i] = lock_force + omni_state->method_force[i];
    }
    // if lock is false: force[i] is NEVER updated — stale value persists
}
```

Two problems:
1. `method_force` (from CBF) was only applied if `lock` was toggled on (by pressing both buttons simultaneously). So your CBF force feedback was silently ignored unless you happened to press both buttons.
2. When `lock[i]` is false, `force[i]` is never zeroed — whatever was there before keeps getting sent to the device.

After my fix:
```cpp
omni_state->force = hduVector3Dd(0, 0, 0);  // zero each cycle

for (int i = 0; i < 3; i++)
    omni_state->force[i] = omni_state->method_force[i];  // always apply cbf force

for (int i = 0; i < 3; i++) {
    if (!omni_state->lock[i]) continue;
    double err = omni_state->lock_pos[i] - omni_state->position[i];
    double kp = (i == 2) ? 0.04 : 0.03;  // reduced from 0.5!
    omni_state->force[i] += kp * err;
}

// hard clamp at 0.5N
double fmag = omni_state->force.magnitude();
if (fmag > 0.5) omni_state->force *= (0.5 / fmag);
```

The old lock gain of `0.5` on the Z axis was extreme — if the stylus was 40mm from `lock_pos` (which was stuck at origin), that's `0.5 × 40 = 20N`. The device physically cannot exert more than ~3.3N. Trying to command 20N causes it to slam against its limits and oscillate. I reduced that gain to `0.04`.

---

## haptic_teleop.py — the bridge

This node translates between the physical device and ROS velocity commands. Two callbacks:

**`on_state()`** — reads stylus position, converts to `/cmd_vel_ref`:
```python
cmd.linear.x  = deadzone(x_mm, dz, k_lin)   # forward/back stylus → robot speed
cmd.angular.z = deadzone(y_mm, dz, k_ang)   # left/right stylus → robot turning
```

The dead zone prevents tiny hand tremors from commanding motion. Values outside the dead zone get scaled linearly.

**`on_force()`** — receives `/haptic/force` from the CBF, converts to device force:
```python
rx = wrench.force.x * f_scale    # scale down
ry = wrench.torque.z * f_scale
# clamp magnitude
# low-pass filter (exponential moving average)
self.fx = alpha * rx + (1 - alpha) * self.fx
```

**What was broken:**

The original code used `dead_zone=0.02` and `scale_linear=5.0`, treating position as if it were in meters. But the driver publishes in **millimeters** (the launch file sets `units: "mm"`). So:

- Stylus 50mm forward → `v_ref = (50 - 0.02) × 5.0 = 250 m/s`
- This gets clipped to `v_max=0.3` in the CBF, but the gap `v_safe - v_ref` was always enormous
- With `kf=10`, force = `10 × (v_safe - 0.3) ≈ -4.5N` constantly

My fix:
- `dz=15.0` (15mm dead zone — reasonable for the device's ~160mm workspace)
- `k_lin=0.004` — so 50mm past dead zone → `(50-15) × 0.004 = 0.14 m/s` (within a normal velocity range)
- `f_scale=0.05` + `f_max=0.15N` + low-pass filter → forces are subtle and smooth

---

## cbf.py — the control barrier function math

This is the core of the safety system from Zhang et al. 2020. No ROS here, pure math.

**The idea:** you define a "barrier" function `b(x) = depth - r_safe`. When `b > 0` you're safe (far from obstacle). When `b < 0` you're inside the safety radius. The CBF constraint forces the system to stay safe:

```
ḃ + γ·b ≥ 0
```

This means: if you're getting close to the boundary (`b` is small), you must be decelerating (`ḃ` must be positive or at least not too negative). `γ` controls how aggressively it enforces this.

**`barrier(depth, px)`:**

`b = depth - r_safe`

The bearing angle `α = arctan2(px - cx, fx)` tells you the angle to the obstacle relative to the camera's center axis. `cos(α)` tells you what fraction of your forward velocity is actually moving toward the obstacle:
- Obstacle dead ahead: `α=0`, `cos(α)=1.0` — full constraint
- Obstacle 45° to side: `cos(α)=0.7` — partial constraint
- Obstacle 90° to side: `cos(α)=0` — no constraint on forward speed

The CBF constraint becomes: `cos(α) · v ≤ γ·b`

**`solve(v_ref, w_ref, a, c)`:**

`a = cos(α)`, `c = γ·b`

If `a·v_ref ≤ c` → user command already satisfies the constraint, no modification.

If violated → project onto the constraint boundary: `v_safe = c/a`. This is the closest safe velocity to what the user wanted (minimum modification principle from the QP).

**`force(v_ref, w_ref, v_safe, w_safe)`:**

Zhang 2020 Eq. 6: `F = kf · (v_safe - v_ref)`

The force is zero when you're safe (no resistance). When the CBF has to intervene, the force is proportional to how much it had to change your command — pushing back on you to signal "you're asking too much." The direction of the force guides you toward safe inputs.

**What I changed:**
- `kf`: `10.0 → 0.5` (Zhang uses ~1.0; 10 was generating multi-Newton forces from tiny velocity differences)
- `f_max`: `2.5 → 0.3N` (conservative for testing)
- `r_safe`: `0.4 → 0.3m` (slightly tighter but more realistic for LIMO footprint)
- `gamma`: `1.5 → 0.8` (less aggressive braking onset)

---

## cbf_node.py — the ROS wrapper

This is straightforward: it subscribes to `/cmd_vel_ref` and `/depth/closest`, runs `cbf.step()` at 20Hz, publishes results.

The 20Hz timer is important — it decouples the control loop rate from the sensor rate. Even if the camera only updates at 5Hz (depth estimation is slow), the CBF still outputs at 20Hz using the last known obstacle position. This prevents the robot from drifting dangerously between depth frames.

The only notable thing: `/depth/closest` uses `BEST_EFFORT` QoS (matches how `depth_node` publishes — it drops frames rather than queue them, since stale depth data is worse than no data).

---

## depth_node and camera_node

These handle the perception side. The camera publishes raw frames; `depth_node` runs ZoeDepth (a monocular depth estimation model from Intel, fine-tuned on NYU + KITTI datasets). It finds the minimum-depth pixel in the inner region (ignoring 50px margins where the model is unreliable at edges), and publishes that as `/depth/closest` — a PointStamped where `point.x` = pixel column, `point.z` = depth in meters.

The calibration `real_depth = 0.2407 × model_output + 0.0502` was measured empirically on the Astra camera by comparing ZoeDepth output against known distances.

---

## Why the test command caused maximum shaking

```
ros2 topic pub /depth/closest ... point: {x: 320.0, y: 200.0, z: 0.3}
```

With z=0.3 and r_safe=0.4: `b = 0.3 - 0.4 = -0.1` — already inside the safety zone. The CBF must command `v ≤ γ·b / cos(0) = 0.8 × (-0.1) = -0.08 m/s` (reverse). Meanwhile, the position-unit bug made `v_ref` always saturate at 0.3 m/s. So force was always at max. Combined with the lock force issue and no force smoothing, the result was violent oscillation.

For testing going forward, use z ≥ 0.5 initially, then slowly bring it down toward r_safe=0.3 to feel the force ramp in gradually.