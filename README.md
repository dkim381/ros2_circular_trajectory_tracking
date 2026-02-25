# ROS2 Circular Trajectory Tracking

This project implements circular trajectory tracking for a unicycle-type mobile robot using ROS2 (Humble) and Gazebo.  
A feedforward + feedback controller is designed and evaluated under multiple experimental conditions.

---

## Controller Design

The control input is composed of:

- Feedforward term (ideal circular motion)
    - v_ff = R * ω
    - w_ff = ω
- Feedback term
    - Longitudinal correction: kv * e_fwd
    - Lateral correction: kp_lat * e_lat

Body-frame error decomposition is used for stability and convergence.

---

## Experimental Cases

The controller was evaluated under the following conditions:

| Case | R | ω | Description |
|------|---|---|-------------|
| Baseline | 2.0 | 0.07 | Nominal tracking |
| High ω | 2.0 | 0.10 | Increased angular velocity |
| Large Radius | 3.0 | 0.07 | Larger curvature |
| Noise | 2.0 | 0.07 | Increased measurement noise |
| No Feedforward | 2.0 | 0.07 | Ablation study |

---

## Quantitative Results

| Case | RMSE (m) | Mean Error (last 20s) (m) |
|------|----------|--------------------------|
| Baseline | 0.718 | 0.013 |
| High ω (tuned) | 0.703 | 0.008 |
| High ω (untuned) | 1.281 | 1.355 |
| Large Radius | 1.193 | 0.006 |
| Noise | 0.737 | 0.046 |
| No Feedforward | 0.817 | 0.386 |

---

## Key Observations

- Feedforward significantly reduces steady-state error (0.386 m → 0.013 m).
- Higher angular velocity requires gain retuning to maintain stability.
- Increased measurement noise degrades steady-state precision.
- The controller maintains convergence across varying curvature conditions.

---

## Example Tracking Result (Baseline)

![Baseline Trajectory](results/baseline_R2_w007_trajectory.png)
![Baseline Error](results/baseline_R2_w007_error.png)
