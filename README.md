# AUV Adaptive MPC with Regularized Dynamic-Forgetting Gaussian Processes

## Introduction

Autonomous underwater vehicles (AUVs) are increasingly used for tasks like
subsea pipeline inspection, seabed mapping, and infrastructure survey —
work that depends on the vehicle holding an accurate position and heading
even while moving through a hostile, constantly shifting environment.
Achieving that kind of precision requires a control strategy that can plan
ahead rather than just react, and can account for the physical limits of
the vehicle's thrusters as it does so.

## Domain

The underwater domain is particularly unforgiving for model-based control.
An AUV's behavior is governed by nonlinear hydrodynamics — added mass,
drag, Coriolis and centripetal coupling between its surge, sway, heave,
and yaw motions — and on top of that, it is constantly pushed around by
currents, self-induced flow, thruster wake, and, for tethered vehicles,
drag from the tether itself. None of this is fully known in advance, and
much of it changes character over the course of a mission.

## Problem Statement

Model predictive control (MPC) optimizes a sequence of actions over a
future horizon while respecting actuator and state limits, rather than
responding to error moment by moment. But an MPC controller is only as
good as the model it optimizes against, and an AUV's model is never fully
accurate. The unmodeled portion — currents, tether pull, thrust
degradation — behaves like an external disturbance, and if the controller
cannot estimate it, tracking accuracy degrades exactly when precision
matters most.

A Gaussian Process (GP) can learn that disturbance online, but a single
GP with one fixed "forgetting factor" (how much it trusts old data vs.
new) cannot handle disturbances that change character mid-mission — one
setting is never right for both slow, repeating currents and sudden
transients.

## Solution Provided

This project implements a learning-based MPC framework for a reduced,
4-degree-of-freedom (surge, sway, heave, yaw) AUV model that maintains
several Gaussian Processes in parallel, each discounting past data at a
different rate, and blends their disturbance predictions online based on
recent accuracy. Two extensions are added on top of that base idea:

- **Regularized blending across the GP bank** (Novelty 1) — the plain
  linear-program blend collapses to hard-switching between models; a
  quadratic regularization term fixes this into a genuine blend.
- **Uncertainty-aware constraint tightening in the MPC** (Novelty 2) —
  the GP bank's predictive variance, not just its mean, tightens the
  controller's actuator limits when the disturbance estimate is unsure.

---

## System Architecture

![System Architecture](docs/system_architecture.png)

The two red-bordered stages above (5 and 7) are this project's own
contributions — everything else follows the base paper's structure.

---

## Complete Mathematical Formulation

This section walks through every equation actually implemented in the
project, from the vehicle physics up to the two novel contributions.

### 1. Degrees of freedom and state definition

The vehicle is modeled in **4 degrees of freedom (4-DOF)**: surge, sway,
heave, and yaw. Roll and pitch are excluded because the vehicle's
buoyancy/weight distribution self-stabilizes them, so active control of
those two axes is unnecessary — a standard simplification for this class
of AUV.

State vector:
```
x = [x, y, z, u, v, w, psi, r]^T
```
- `x, y, z` — position in the earth-fixed frame
- `u, v, w` — surge/sway/heave velocity in the body-fixed frame
- `psi` — heading (yaw angle)
- `r` — yaw rate

Control input:
```
tau = [X, Y, Z, Mz]^T
```
Forces along surge/sway/heave and a yaw moment.

### 2. Kinematics — body-frame velocity to earth-frame motion

```
x_dot   = cos(psi)*u - sin(psi)*v
y_dot   = sin(psi)*u + cos(psi)*v
z_dot   = w
psi_dot = r
```
This rotates body-frame velocity into the earth frame using the current
heading. (Code: `KinematicsBlock.m` / the kinematics section of
`auv_dynamics.m`.)

### 3. Rigid-body dynamics

For each velocity channel, the general pattern is:
```
(effective mass) * acceleration = applied_force + Coriolis_term + Damping_term + disturbance
```

**Surge:**
```
(m - Xu_dot)*u_dot = X + (m*v + Yv_dot*v)*r + (Xu + Xuc*|u|)*u + Delta_x
```
**Sway:**
```
(m - Yv_dot)*v_dot = Y - (m*u + Xu_dot*u)*r + (Yv + Yvc*|v|)*v + Delta_y
```
**Heave:**
```
(m - Zw_dot)*w_dot = Z + (Zw + Zwc*|w|)*w + (m - Vsub*rho_water)*g + Delta_z
```
**Yaw:**
```
(Izz - Nr_dot)*r_dot = Mz - (m*v - Yv_dot*v)*u - (Xu_dot*u - m*u)*v + (Nr + Nrc*|r|)*r + Delta_Mz
```

Where `Xu_dot, Yv_dot, Zw_dot, Nr_dot` are added-mass coefficients,
`Xu, Yv, Zw, Nr` are linear drag coefficients, `Xuc, Yvc, Zwc, Nrc` are
quadratic drag coefficients, and `Delta = [Delta_x, Delta_y, Delta_z,
Delta_Mz]` is the lumped external disturbance this whole project exists
to estimate. (Code: `auv_dynamics.m`, and as separate
`CoriolisBlock.m`/`DampingBlock.m` in the block-decomposed Simulink
model.)

**Note on the yaw axis specifically:** `(Izz - Nr_dot) = 0.52` is a small
number, meaning a given moment produces a disproportionately large
angular acceleration on this axis. This is what made yaw the numerically
sensitive part of the whole simulation (see the Simulink section below).

### 4. Discretization

The continuous equations above are integrated forward in time using
4th-order Runge-Kutta (RK4):
```
k1 = f(x_k,          u_k, Delta)
k2 = f(x_k + Ts/2*k1, u_k, Delta)
k3 = f(x_k + Ts/2*k2, u_k, Delta)
k4 = f(x_k + Ts*k3,   u_k, Delta)
x_{k+1} = x_k + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4)
```
(Code: `rk4_integrate.m`.) RK4 evaluates the dynamics 4 times per step
and blends the results — far more numerically accurate and stable than
a single-evaluation method like Forward Euler, which is what standard
Simulink `Discrete-Time Integrator` blocks use by default.

### 5. Model Predictive Control formulation

At every control step, over a horizon of `Nc` steps:
```
minimize   sum_{k=0}^{Nc-1} [ (x_k - x_ref_k)^T Q (x_k - x_ref_k) + u_k^T R u_k ]
           + (x_Nc - x_ref_Nc)^T Q_T (x_Nc - x_ref_Nc)

subject to  x_{k+1} = f_d(x_k, u_k, Delta_hat)      (RK4 discretized model)
            u_min <= u_k <= u_max
```
`Q` penalizes tracking error per state, `R` penalizes control effort,
`Q_T` is the terminal cost. Solved with MATLAB's `fmincon` (SQP
algorithm). Only the first control action from the optimal sequence is
applied, then the whole problem is re-solved next step with fresh
information — the defining feature of MPC. (Code: `nmpc_solve.m`.)

### 6. Disturbance measurement (residual estimation)

The disturbance is never measured directly — it is inferred by comparing
what actually happened to the vehicle against what the disturbance-free
model predicted:
```
Delta_meas = M * ( x_dot_measured - x_dot_nominal )
```
where `M = diag(m - Xu_dot, m - Yv_dot, m - Zw_dot, Izz - Nr_dot)` and
`x_dot_measured` is approximated by finite-differencing consecutive
state measurements. (Code: `residual_disturbance.m`.)

### 7. Gaussian Process disturbance prediction

Each GP uses a squared-exponential kernel:
```
k(a, a') = sigma_f^2 * exp( -0.5 * (a - a')^T * L^-2 * (a - a') )
```
For a forgetting factor `lambda`, older training samples are given less
weight (larger effective noise) via:
```
w_i = lambda^(n-i)          (most recent sample gets weight 1)
```
The predictive mean and variance use a standard weighted kernel-ridge
formulation:
```
mu*    = k*^T (K + sigma_eps^2 * diag(1/w))^-1 * y
sigma2* = k** - k*^T (K + sigma_eps^2 * diag(1/w))^-1 * k*
```
Three GPs run in parallel with `lambda = 1.0, 0.8, 0.6`. (Code:
`ForgettingGP.m`.)

### 8. NOVELTY 1 — Regularized dynamic weight blending

The base paper blends the 3 GP predictions by solving:
```
minimize_eta   sum_j eta_j * error_j
subject to     sum(eta) = 1,  eta >= 0
```
This is **linear** in `eta`. A linear objective over a probability
simplex is always minimized at a vertex — meaning the "optimal blend" is
mathematically forced to put 100% weight on one GP and 0% on the others,
a hard switch, not a real blend (proven, not assumed — see
`demo_lp_vs_qp_blend.m`).

This project's fix — add a quadratic regularization term:
```
minimize_eta   sum_j eta_j * error_j  +  rho * ||eta||^2
subject to     sum(eta) = 1,  eta >= 0
```
`rho = 0` reproduces the paper's exact hard-switching behavior. `rho =
0.05` (this project's setting) bends the objective into a bowl shape,
letting the minimum land in the interior of the simplex — a genuine
blend across GPs. Solved as a small quadratic program via `quadprog`.
(Code: `dynamic_weight_qp.m`.) Measured result: mean weight "churn" per
step drops from ~0.049 (plain LP) to ~0.045 (regularized QP).

### 9. NOVELTY 2 — Uncertainty-aware MPC

The blended disturbance estimate and its variance:
```
Delta_hat  = sum_j eta_j * mu_j
Sigma_hat  = sum_j eta_j * sigma2_j
```
`Delta_hat` feeds into the MPC's internal model (Section 5). This
project additionally uses `Sigma_hat` to tighten the actuator bounds:
```
shrink = min(0.3, 0.05 * sqrt(mean(Sigma_hat)))
u_min_tightened = u_min * (1 - shrink)
u_max_tightened = u_max * (1 - shrink)
```
When the disturbance estimate is uncertain, the controller automatically
leaves itself more margin; when the GP is confident, it uses the full
actuator range. (Code: `nmpc_solve.m`.)

### 10. Sensor noise and Monte Carlo validation

To test robustness under realistic conditions, a noisy variant of the
simulation adds Gaussian noise to the velocity/yaw-rate measurements the
controller sees, while the vehicle's true physics stay exact:
```
x_meas = x_true + noise,   noise ~ N(0, noise_std^2)   (on u, v, w, r only)
```
Tracking error is judged against the true state; every controller
decision uses only the noisy measurement. (Code:
`simulate_method_noisy.m`.) The Monte Carlo study reruns this across
many random noise realizations ("seeds") and reports mean ± standard
deviation instead of a single-run number, which is what determines
whether a result is statistically reliable rather than a lucky draw.
(Code: `run_monte_carlo.m`.)

---

## System Modelling

The system is implemented two ways that call the same underlying
functions: a pure-MATLAB simulation, and a Simulink block diagram.

### Top-level closed loop

![Top-level Simulink model](docs/top_level_diagram.png)

A `Clock` drives `Reference`, `Disturbance`, and `Controller`. The
`Controller` block runs the entire GP bank, weight blending, and
uncertainty-aware MPC solve internally, outputting `u` into the `Plant`
subsystem along with the true disturbance `Delta`. `Plant`'s resulting
state `x` feeds back into `Controller` through an explicit `Unit Delay`
block (needed to break the Controller-Plant algebraic loop reliably).

### Inside the Plant subsystem — two versions

Two Simulink models exist, both implementing the exact equations in
Section 3 above:

- **`auv_full_system.slx`** — the physics combined into a single block
  internally calling the same RK4-integrated functions the MATLAB
  simulation uses. This is the version whose results closely match
  `run_simulation.m`.
- **`auv_full_system_decomposed.slx`** — Coriolis, Damping, Restoring,
  and Kinematics as four separate, individually labeled blocks (see
  `docs/plant_subsystem.png`), matching the base paper's diagram
  exactly. This version uses Simulink's `Discrete-Time Integrator`
  blocks (Forward-Euler integration), a cruder method than RK4 — on the
  yaw axis's small effective inertia (Section 3), this required running
  the model at a much finer internal time step to remain numerically
  stable, which is why its results diverge somewhat from the other two.

Both are kept intentionally: one demonstrates the physics as visibly
separated, labeled blocks; the other demonstrates closer numerical
agreement with the validated MATLAB results.

---

## Results

Produced from a 90 s run of all four controller variants (`NoGP`,
`StaticGP`, `DFGP_LP`, `RDFGP_UAMPC`) tracking a rotated figure-eight
reference under a three-regime disturbance profile (single sine →
combined sine → square wave).

### Trajectory tracking

![Trajectory tracking](docs/trajectory_tracking.png)

All four controllers trace a closed figure-eight following the tilted
reference closely for most of the loop. Two extra small loops appear at
the top and bottom tips — this is the MPC's limited lookahead (`Nc = 8`
steps, 1.6 s) not fully anticipating the sharpest direction reversal on
the path, causing a brief overshoot before rejoining the trajectory.

### Position error over time

![Position error](docs/position_error.png)

Two sharp error spikes appear at roughly t = 30 s and t = 60 s — exactly
where the disturbance regime switches. The GP bank has to re-learn the
new pattern from scratch at each switch; error follows the
disturbance-prediction error until it catches up. Outside those two
transients, error stays consistently low across all four methods.

### Disturbance estimation (surge axis)

![Disturbance estimation](docs/disturbance_estimation.png)

The GP-based estimators track the smooth sine and combined-sine sections
well, with visible overshoot right at each abrupt square-wave jump
before settling — expected behavior when adapting to a sudden step
rather than a gradual change.

### Measured metrics

| Method | Prediction RMSE | Tracking RMSE (m) |
|---|---|---|
| StaticGP | 0.4363 | 0.1973 |
| RDFGP_UAMPC (proposed) | 0.5676 | 0.1976 |

## Conclusion

Both the MATLAB simulation and the Simulink models implement the same
learning-based MPC pipeline: a bank of forgetting-factor Gaussian
Processes estimates the AUV's lumped disturbance online, a regularized
quadratic program blends their predictions into a single estimate and
uncertainty, and an uncertainty-aware nonlinear MPC uses both to track a
rotated figure-eight trajectory under a shifting disturbance profile.
The closed-loop system tracks the reference reliably across all three
disturbance regimes, with error concentrated at the two regime-switch
points where the estimator has to adapt. The project demonstrates, with
its own generated evidence, that the base paper's weight-blending
formulation collapses to hard-switching between models, and that a
quadratic regularization term resolves this while keeping the same
overall architecture. A sensor-noise model and Monte Carlo robustness
study further validate the approach under realistic, non-idealized
conditions.
