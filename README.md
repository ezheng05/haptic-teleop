# haptic-teleop

Vision-based haptic shared autonomy for an AgileX LIMO ground robot, in ROS 2 Humble.

A single RGB camera feeds ZoeDepth, a monocular depth network. The nearest obstacle goes to a control barrier function (CBF) safety filter that brakes the robot. How much the filter had to change the operator's command is rendered as a force on a Geomagic Touch stylus, so the operator feels the correction.

BU Robotics Lab. Advised by Prof. Roberto Tron and Dr. Dawei Zhang. Presented at the BU UROP Symposium, 16 October 2026 (*An Implementation of Vision-Based Haptic Navigation*).

## Pipeline

```
LIMO camera ──/camera/color/image_raw──> depth_node (ZoeDepth + calibration)
                                              │ /depth/closest
Geomagic Touch ──/phantom/state──> haptic_teleop ──/cmd_vel_ref──> cbf_node (20 Hz)
      ^                                  ^                           │       │
      └──/phantom/force_feedback─────────┴───────/haptic/force───────┘       └──/cmd_vel──> LIMO
```

| node | what it does |
|---|---|
| `depth_node` (`node.py`, `depth.py`) | ZoeDepth on each frame, calibrated to meters, then picks the nearest point. Ignores a 50 px border and the bottom 35% of the frame, where the floor would otherwise be picked as the closest point. |
| `cbf_node` (`cbf.py`, `cbf_node.py`) | Barrier `b = d - r_safe`, constraint `cos(α)·v ≤ γ·b`, solved in closed form (no QP solver). Force `F = kf·(v_safe - v_ref)` (Zhang et al. 2020). Stops the robot if `/cmd_vel_ref` goes quiet for 0.2 s. |
| `haptic_teleop` | Stylus position (mm) to `/cmd_vel_ref` with a dead zone, and CBF force to device force with scaling, clamping and a low-pass filter. Dead-man switch on the stylus button. |
| `patches/omni_state.cpp` | Fixes to the `omni_common` Touch driver: CBF force is always applied (it was dropped unless "lock" was on), force is zeroed each cycle, lock gain reduced, 0.8 N hard clamp. |
| `force_viz`, `overlay`, `validate` | RViz force arrow, camera overlay with depth and direction, and live comparison against the onboard IR depth. |

`System Architecture.md` walks through each piece and the bugs that were fixed.

## Depth calibration

ZoeDepth's raw output is not metric on this camera.

| fit | data | error |
|---|---|---|
| linear, `0.2526·raw + 0.0195` | 19 frames vs onboard IR | 18.7 cm RMSE vs tape measure |
| quadratic, `0.1196·raw² - 0.2886·raw + 0.4771` (default) | 7 tape-measured positions, 0.30 to 1.50 m | 2.3 cm RMSE |

The IR sensor itself is accurate to 0.8 cm on matte surfaces but reads through transparent and thin ones, which is why the IR-based linear fit was off. Below the parabola's vertex the curve is continued with the linear slope, and above 4.5 raw it is held flat, so the mapping stays monotone. Select with `depth_calib:=quadratic|linear`. Scripts: `fit_calib.py`, `validate_gt.py`, `closest_error.py`.

## Results so far

All on recorded bag playback (the camera stream over lab wifi is too slow for live inference on the desktop GPU).

- At the safety boundary the filter brakes 0.20 m/s to -0.01 m/s, at 20 Hz.
- Human in the loop with the Touch (7 Oct 2026): force reaches the hand at exactly the commanded size, direction is always correct, and 0.8 N is clearly felt, while 0.3 to 0.5 N is too weak to be a useful cue. Details and plots in [`docs/haptic_testing`](docs/haptic_testing/README.md).

## Run

Needs ROS 2 Humble, the `omni_common` Touch driver (with `patches/omni_state.cpp`), and PyTorch + `transformers` for ZoeDepth (CUDA, MPS or CPU).

```bash
colcon build --packages-select depth_estimation_pkg && source install/setup.bash

ros2 launch omni_common omni_state.launch.py        # Touch driver, stylus docked
ros2 run depth_estimation_pkg haptic_teleop
ros2 run depth_estimation_pkg depth_node
ros2 run depth_estimation_pkg cbf_node
ros2 bag play <bag> --loop                          # or the live LIMO camera
```

Without the Touch, publish `/cmd_vel_ref` by hand:

```bash
ros2 topic pub /cmd_vel_ref geometry_msgs/Twist '{linear: {x: 0.2}}'
ros2 topic echo /cbf/debug
```

Main parameters: `cbf_node` `r_safe` 0.30 m, `gamma` 0.8, `kf` 0.5; `haptic_teleop` `f_scale` 8, `f_max` 0.8 N, `deadman` grey.

## Next

Live test on the LIMO (dead-man switch and stop-on-exit are written and waiting on a hardware test), handling stale depth if `depth_node` dies, and collision-cone CBFs so the filter can steer around obstacles instead of only braking.

## References

- D. Zhang, G. Yang, R. P. Khurshid. Haptic teleoperation of UAVs through control barrier functions. IEEE Trans. Haptics, 2020.
- A. D. Ames et al. Control barrier functions: theory and applications. ECC, 2019.
- S. F. Bhat et al. ZoeDepth: zero-shot transfer by combining relative and metric depth. arXiv:2302.12288, 2023.

MIT license.
