# ROS2 Circular Trajectory Tracking

This project implements circular trajectory tracking for a unicycle-type mobile robot in ROS2 (Humble) with Gazebo simulation.

A feedforward + feedback controller is designed and evaluated under multiple operating conditions, including gain tuning, curvature variation, noise robustness, and ablation (no feedforward).

The objective is to analyze convergence, steady-state error, and robustness in a structured control experiment.

---

# System Overview

- Platform: Unicycle-type mobile robot
- Simulation: Gazebo + ros_gz_bridge
- Control frequency: 20 Hz
- State feedback: Odometry (x, y, yaw)

---

# Controller Design

The controller consists of a feedforward term for ideal circular motion and a feedback term based on body-frame tracking error.

## 1️⃣ Reference Model (Circular Motion)

\[
x_d = c_x + R \cos(\omega t)
\]
\[
y_d = c_y + R \sin(\omega t)
\]

---

## 2️⃣ Feedforward Control

For ideal circular motion of a unicycle model:

- v_ff = R · ω
- w_ff = ω

This ensures perfect tracking in the absence of error.

---

## 3️⃣ Feedback Control

Tracking error is decomposed in the body frame:

- e_fwd  (longitudinal error)
- e_lat  (lateral error)

Control inputs:

- v = v_ff + k_v · e_fwd
- w = w_ff + k_lat · e_lat

---

## 4️⃣ Control Saturation

To maintain realistic actuator limits:

- v ∈ [0, v_max]
- w ∈ [-w_max, w_max]

Where:

- v_max = 0.15 m/s
- w_max = 1.50 rad/s

Saturation significantly affects convergence under aggressive angular velocities.

---

# Experimental Setup

## Trajectory Parameters

- Radius (R): 2.0 m / 3.0 m
- Angular velocity (ω): 0.07 rad/s / 0.10 rad/s

## Controller Gains

- k_v = 0.35
- k_lat = 2.20
- v_max = 0.15 m/s
- w_max = 1.50 rad/s

## Noise Model

- Baseline:
  - noise_xy_std = 0.01
  - noise_yaw_std = 0.005

- Noise stress test:
  - Increased measurement noise

## Ablation Study

- Feedforward enabled
- Feedforward disabled (pure P-type tracking)

---

# Quantitative Results

| Case | R | ω | RMSE (m) | Mean Error (last 20s) (m) |
|------|---|---|----------|--------------------------|
| Baseline | 2.0 | 0.07 | 0.718 | 0.013 |
| High ω (tuned) | 2.0 | 0.10 | 0.703 | 0.008 |
| High ω (untuned) | 2.0 | 0.10 | 1.281 | 1.355 |
| Large Radius | 3.0 | 0.07 | 1.193 | 0.006 |
| Noise | 2.0 | 0.07 | 0.737 | 0.046 |
| No Feedforward | 2.0 | 0.07 | 0.817 | 0.386 |

---

# Key Observations

- Feedforward significantly reduces steady-state error  
  (0.386 m → 0.013 m).

- Higher angular velocity requires retuning due to actuator saturation limits.

- Increased measurement noise degrades steady-state accuracy but does not destabilize the controller.

- Larger curvature mainly increases transient error while maintaining convergence.

---

# Example Result (Baseline)

![Baseline Trajectory](results/baseline_R2_w007_trajectory.png)

![Baseline Tracking Error](results/baseline_R2_w007_error.png)


---

# Conclusion

This project demonstrates structured control system evaluation in a ROS2 simulation environment, including robustness analysis, ablation study, and saturation-aware design.

The controller achieves stable circular tracking under varying curvature, noise, and actuator constraints.
