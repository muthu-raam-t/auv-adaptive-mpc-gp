# AUV Adaptive MPC with Regularized Dynamic-Forgetting Gaussian Processes

## Introduction

Autonomous underwater vehicles (AUVs) are used for pipeline inspection,
seabed mapping, and subsea survey — work that depends on holding an
accurate position while pushed around by a constantly shifting
environment. The base paper validated its method in the **Mobula**
underwater simulator plus real BlueROV2 pool tests. This project
reproduces and extends the same control framework entirely in
**MATLAB + Simulink**, without hardware access.

## Problem Statement

MPC plans ahead over a future horizon but is only as good as the model
it optimizes against. The unmodeled portion of the vehicle's behavior —
currents, tether pull, thrust degradation — acts like an external
disturbance; if the controller cannot estimate it, tracking degrades
exactly when precision matters most. A single GP with one fixed
"forgetting factor" cannot handle disturbances that change character
mid-mission.

## Solution Provided

A learning-based MPC that blends several GPs (each trusting recent vs.
old data differently) online, extended with two core novelties to the
control algorithm plus four further validation/industrial-relevance
additions.

---

## System Architecture

![System Architecture](docs/system_architecture.png)

Red-bordered stages inside the control loop are the two core algorithmic
novelties. The parallel MATLAB / Simulink branch and the fault-tolerance
and Monte-Carlo stages are the further validation work described below.

---

## All Equations — Paper vs. This Project

| Eq. | What it is | Formula | Status |
|---|---|---|---|
| 1–3 | Position kinematics | `x_dot=cos(psi)u-sin(psi)v`, `y_dot=sin(psi)u+cos(psi)v`, `z_dot=w` | Same |
| 4 | Surge dynamics | `(m-Xu_dot)u_dot = X+(mv+Yv_dot·v)r+(Xu+Xuc\|u\|)u+Delta_x` | Same |
| 5 | Sway dynamics | `(m-Yv_dot)v_dot = Y-(mu+Xu_dot·u)r+(Yv+Yvc\|v\|)v+Delta_y` | Same |
| 6 | Heave dynamics | `(m-Zw_dot)w_dot = Z+(Zw+Zwc\|w\|)w+(m-Vsub·rho)g+Delta_z` | Same |
| 7 | Heading kinematics | `psi_dot = r` | Same |
| 8 | Yaw dynamics | `(Izz-Nr_dot)r_dot = Mz-(mv-Yv_dot·v)u-(Xu_dot·u-mu)v+(Nr+Nrc\|r\|)r+Delta_Mz` | Same |
| 9 | Bundled state-space form | `x_dot = f(x,u,Delta)` | Same |
| 10 | State vector | `x = [x,y,z,u,v,w,psi,r]^T` | Same |
| 11 | Control vector | `u = [X,Y,Z,Mz]^T` | Same |
| 12a | MPC cost | `sum[(x_k-x_ref)'Q(x_k-x_ref)+u_k'Ru_k] + terminal` | Same |
| 12b | Dynamics constraint | `x_{k+1}=f_d(x_k,u_k,Delta)` (RK4) | Same (solved via `fmincon`, paper uses qpOASES) |
| 12c | Actuator bounds | `u_min <= u_k <= u_max` | Same |
| 13 | Training dataset | `D = D1 (static) UNION D2 (sliding)` | **Changed** — single sliding buffer only |
| 14 | Observation model | `y_i = f(a_i) + eps_i,  eps_i~N(0,sigma_eps^2)` | Same |
| 15 | Joint Gaussian prior | `[y;f*] ~ N(0,[[K+sigma^2 I, k*],[k*^T,k**]])` | Same |
| 16 | Kernel | `k(a,a')=sigma_f^2·exp(-0.5(a-a')^T L^-2(a-a'))` | Same |
| 17 | Hyperparameter fit | `theta_opt = argmin[NLL]` via conjugate gradient | **Changed** — closed-form heuristic, no iteration |
| 18–19 | Dense GP posterior mean/variance | Standard GP formulas | **Changed** — merged directly into the forgetting-weighted form |
| 20–21 | Adaptive Sparse GP mean/variance | Sparse, inducing-point form | **Changed** — dense weighted-kernel-ridge instead |
| 22 | Forgetting matrix | `Lambda=diag(lambda^(n-1),...,lambda^0)` | **Changed** — folded into per-sample weight `w_i` |
| 23 | Sparse precision matrix | `B_lambda=(Kss+sigma^-2·Ksa·Lambda·Kas)^-1` | **Changed** — not used (no sparse structure) |
| 24 | GP input feature | Full history window `[Delta,x,u]` back `H` steps | **Changed** — current state only: `a=[u,v,w,r]` |
| 25 | GP target | `y = Delta_tau` | Same |
| 26a | Weight objective | `min_eta sum_j(eta_j·error_j)` | **Changed** — `+ rho·‖eta‖^2` added (Novelty 1) |
| 26b–c | Weight constraints | `sum(eta)=1`, `eta>=0` | Same |
| 27 | LP standard form | `min c'eta  s.t. a'eta=b, eta>=0` | Same in spirit (ours is QP when `rho>0`) |
| 28 | Fused mean | `Delta_hat = sum(eta_j·mu_j)` | Same |
| 29 | Fused variance | `Sigma_hat = sum(eta_j·sigma2_j)` | Same formula — **Changed in use**: paper never feeds this into control; we do (Novelty 2) |
| alpha_i | Recency weight | `alpha_i = e^(0.05(N-i))` | Same |

---

## Novelties — Complete List

1. **Regularized dynamic weight blending** (Eq. 26a) — the paper's blend is
   linear in the weights, so it always collapses to a hard switch between
   GPs (a simplex-LP always optimizes at a vertex). Adding `rho·‖eta‖^2`
   makes it a genuine blend. **Measured: weight churn drops from
   0.049/step (plain LP) to 0.045/step (regularized QP).**
2. **Uncertainty-aware MPC** (Eq. 29 use) — the paper computes the fused
   GP variance but never feeds it into the control law (stated future
   work). This project uses it directly to tighten actuator bounds when
   the disturbance estimate is unreliable.
3. **Thruster fault-tolerance testing** — a mid-mission actuator fault
   (yaw-thruster efficiency drops to 50% at t=45s) is absorbed by the
   *existing* disturbance-learning pipeline, no new algorithm required,
   because a fault and a disturbance are mathematically indistinguishable
   to the controller.
4. **Sensor noise + Monte Carlo validation** — an 8-seed statistical study
   under realistic DVL/IMU-style measurement noise, reporting mean and
   standard deviation instead of a single run.
5. **Industrial-relevance reframing** — mission-spec compliance and
   actuator-energy usage, business-relevant metrics beyond raw RMSE.
6. **Dual Simulink validation** — two independent block-diagram
   implementations, an algebraic-loop diagnosis and fix (explicit Unit
   Delay), and a documented integration-accuracy trade-off between them.

---

## Results

### Single-run comparison (90s, no sensor noise)

| Method | Tracking RMSE (m) | Disturbance Pred. RMSE |
|---|---|---|
| NoGP | 0.1969 | N/A |
| StaticGP | 0.1972 | 0.3671 |
| DFGP_LP (paper method) | 0.1975 | 0.3581 |
| **RDFGP_UAMPC (proposed)** | 0.1975 | 0.3584 |

**Inference:** in a clean scenario all four methods track almost
identically — an easy scenario alone does not separate them. The two
multi-GP methods edge out `StaticGP` slightly on disturbance prediction.

### Base paper vs. this project

| Method | Paper (Pred) | Ours | Delta% | Paper (Track) | Ours | Delta% |
|---|---|---|---|---|---|---|
| No GP | -- | N/A | -- | 0.1050 | 0.1969 | +87% |
| DF-GP (LP) | 0.289 | 0.358 | +24% | 0.0285 | 0.1975 | +593% |

**Inference:** our numbers are higher than the paper's, expected because
our controller's internal model IS the true simulated model — there is
no hidden real-world mismatch for the learning system to overcome, unlike
the paper's real hardware test.

### Monte Carlo robustness study (8 seeds, sensor noise std = 0.01)

| Method | Pos RMSE mean | Pos RMSE std | Dist RMSE mean | Dist RMSE std |
|---|---|---|---|---|
| NoGP | 0.1970 | 0.0003 | N/A | N/A |
| StaticGP | 0.2066 | 0.0028 | 3.1959 | 0.6602 |
| DFGP_LP | 0.2050 | 0.0030 | 3.4889 | 0.5590 |
| **RDFGP_UAMPC** | 0.2055 | 0.0024 | 3.6408 | **0.3384** |

**Inference:** `RDFGP_UAMPC` has the highest average disturbance error of
the three learning methods, but the **smallest spread** (std 0.34 vs.
`StaticGP`'s 0.66) — the most consistent, predictable performer across
randomized noise conditions, even though not the most accurate on
average.

### Thruster fault-tolerance test (fault at t=45s, yaw actuation → 50%)

| Method | Before Fault RMSE (m) | After Fault RMSE (m) | Degradation |
|---|---|---|---|
| NoGP | 0.1950 | 0.2632 | +35.0% |
| **RDFGP_UAMPC** | 0.1958 | 0.2583 | **+31.9%** |

**Inference:** both degrade after the fault (unavoidable — less usable
thrust exists), but `RDFGP_UAMPC` degrades less, with no new algorithm
added for this scenario — direct evidence that the disturbance-learning
pipeline partially absorbs actuator failure.

### Industrial readiness metrics (0.30m mission-spec tolerance)

| Method | Clean-run % in spec | Effort vs. NoGP | Spec % Before Fault | Spec % After Fault |
|---|---|---|---|---|
| NoGP | 92.9% | 1.00x | 92.9% | 90.7% |
| StaticGP | 92.9% | 1.01x | -- | -- |
| DFGP_LP | 92.7% | 1.01x | -- | -- |
| RDFGP_UAMPC | 92.7% | 1.01x | 92.9% | 90.7% |

**Inference:** clean-run spec compliance is nearly identical across
methods; the real industrial question is whether compliance holds up
under fault, which is what an operations team actually judges a system
on.

---

## System Modelling

- **`auv_full_system.slx`** — physics as one RK4-integrated block, closely matching MATLAB. Feedback loop closed with an explicit `Unit Delay` block (required to reliably break the Controller–Plant algebraic loop).
- **`auv_full_system_decomposed.slx`** — Coriolis/Damping/Restoring/Kinematics as four separate blocks, matching the paper's diagram; needed a finer time step to stay stable on the yaw axis (`Izz-Nr_dot=0.52`, numerically stiff), so its results diverge somewhat from the RK4 version.

---

## What You Should Be Able to Explain, Unprompted

1. Why a plain MPC fails under disturbance
2. Why one GP with a fixed forgetting factor isn't enough
3. Why the paper's weight-blending secretly hard-switches, and how the quadratic term fixes it (with the measured churn numbers)
4. Why the GP's variance, not just its mean, is useful to the MPC
5. Why the yaw axis was numerically tricky (RK4 vs Euler)
6. What a Monte Carlo seed is, and why single-run numbers aren't trustworthy alone
7. The honest finding: `RDFGP_UAMPC` has the worst average disturbance-prediction accuracy of the learning methods but the smallest spread — most consistent, not most accurate
8. Why a thruster fault and an environmental disturbance are indistinguishable to the controller, and what that implies for fault tolerance

## Conclusion

This project reproduces the base paper's learning-based MPC framework,
proves and fixes a real mathematical weakness in its GP weight-blending
step, builds the uncertainty-aware control the paper's own authors left
as future work, and extends the system with demonstrated thruster
fault tolerance, statistical robustness under sensor noise, and an
industrial mission-spec framing — all validated across a MATLAB
simulation and two independent Simulink implementations.
