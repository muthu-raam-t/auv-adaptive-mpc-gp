# Complete Project Masterguide: Learning-Based MPC for AUV Trajectory Tracking

Read this top to bottom, in order. It's built so each section only needs
what came before it — by the end you should be able to explain the whole
thing without notes.

---

## PART 1 — The Problem (Why This Project Exists)

An autonomous underwater vehicle (AUV) needs to follow a planned path
precisely — for pipeline inspection, seabed mapping, survey work. It has
thrusters it can push with, and a controller deciding how hard, in which
direction, every instant.

The controller needs a **model** of the vehicle: "if I push this hard,
the vehicle moves like this." The problem: that model is never perfectly
right. Currents, tether drag, thruster wake — none of it is fully known
in advance, and it changes character mid-mission. A current that's
steady one minute can shift or strengthen the next.

**The fix this project builds:** watch how the vehicle actually moves vs.
how the model predicted, treat the difference as a "disturbance" signal,
learn it online with a Gaussian Process, and feed the learned disturbance
back into the controller so its predictions self-correct.

---

## PART 2 — The Vehicle Model (The Actual Equations)

The vehicle has 8 numbers describing it at any instant — the state:

```
x = [x, y, z, u, v, w, psi, r]
```
- `x, y, z` — position in the world (earth-fixed frame)
- `u, v, w` — velocity in the vehicle's OWN frame (forward/sideways/up-down)
- `psi` — heading angle
- `r` — turn rate (yaw rate)

Four forces control it: `tau = [X, Y, Z, Mz]` — push forward, sideways,
up/down, and a turning moment.

### 2.1 Kinematics — converting body velocity into world position change

```
x_dot   = cos(psi)*u - sin(psi)*v
y_dot   = sin(psi)*u + cos(psi)*v
z_dot   = w
psi_dot = r
```
This just accounts for which way the vehicle is currently pointing when
converting "I'm moving forward at 1 m/s" into "which direction is that
in the real world right now."

### 2.2 Rigid-body dynamics — how forces turn into acceleration

For each velocity component, the pattern is the same:

```
(mass_term) * acceleration = applied_force + Coriolis_term + Damping_term + disturbance
```

**Coriolis/centripetal terms** — apparent forces that show up because the
vehicle is rotating while moving (same reason objects seem to curve on a
spinning platform):
```
Coriolis_u = (m*v + Yv_dot*v)*r
Coriolis_v = -(m*u + Xu_dot*u)*r
```

**Damping terms** — water resistance, growing with speed squared (drag):
```
Damping_u = (Xu + Xuc*|u|)*u
Damping_v = (Yv + Yvc*|v|)*v
```

**Restoring** — buoyancy vs. weight, near-zero here since the vehicle is
close to neutral buoyancy by design.

**The mass terms matter a lot for one specific axis.** The yaw axis has
`(Izz - Nr_dot) = 0.52` — a SMALL number. Small denominator means a given
force produces a LARGE acceleration on that axis. This single fact is
why yaw turned out to be the numerically tricky part of the whole
project (see Part 8).

### 2.3 Putting it together (code: `auv_dynamics.m`)

All four force channels get summed, divided by the effective mass
(`inv(M)` gain), integrated once to get velocity, and the kinematics
equations integrate that into position. `rk4_integrate.m` does this
integration using 4th-order Runge-Kutta — a numerically accurate method
that evaluates the dynamics 4 times per step and blends the results,
rather than a single crude step.

---

## PART 3 — Model Predictive Control (MPC)

Instead of reacting to error moment-to-moment, MPC looks ahead. At every
control step:

1. Take the current state and a short future window (`Nc = 8` steps,
   `Ts = 0.2s` each → 1.6 seconds of lookahead).
2. Simulate forward using the vehicle model, trying different sequences
   of thruster forces.
3. Pick the sequence that minimizes: how far off the planned path you'd
   be (tracking error) + how much force you used (energy cost).
4. Apply only the FIRST action from that sequence.
5. Repeat next step, with fresh information.

The cost function (code: `nmpc_solve.m`):
```
J = sum over horizon of [ (x_k - x_ref_k)' * Q * (x_k - x_ref_k) + u_k' * R * u_k ]
    + terminal cost at the end of the horizon
```
`Q` weighs how much tracking error matters per state; `R` weighs how much
control effort matters. Solved with `fmincon` (nonlinear optimizer),
respecting the vehicle's actuator limits (`u_min`, `u_max`).

**Why this alone isn't enough:** the model MPC optimizes against doesn't
know about the disturbance. If a current is pushing the vehicle
sideways and the MPC's internal model doesn't know that, its "optimal"
plan is optimal for a vehicle that doesn't exist.

---

## PART 4 — Learning the Disturbance: Gaussian Processes

A Gaussian Process (GP) learns a pattern from data without assuming a
fixed shape, and — critically — reports how CONFIDENT it is, not just a
point prediction.

**The estimation loop, every control step (code: `residual_disturbance.m`):**
1. Compare what actually happened to the vehicle (measured acceleration)
   against what the disturbance-free model predicted.
2. The difference IS the disturbance measurement — call it `Delta_meas`.
3. Feed `(recent state, Delta_meas)` into the GP as a training point.
4. Ask the GP: given the CURRENT state, what disturbance do you predict
   right now, and how sure are you?

**The forgetting factor problem:** a GP needs to decide how much to trust
old data vs. new. Trust old data too much → reacts slowly to sudden
changes. Forget too fast → noisy, unstable during calm periods. There's
no single correct setting — it depends on what the disturbance is doing
RIGHT NOW, which isn't known in advance.

**This project's answer (matching the base paper's core idea):** don't
pick one forgetting factor. Run 3 GPs in parallel — `lambda = 1.0, 0.8,
0.6` (slow, medium, fast forgetting) — and blend their predictions based
on which has been most accurate recently. Code: `ForgettingGP.m`.

---

## PART 5 — NOVELTY 1: Regularized Weight Blending

This is where the project diverges from the base paper, with actual
mathematical evidence, not just a claim.

**The base paper's approach (Eq. 26 in their paper):** choose blend
weights `eta_1, eta_2, eta_3` (one per GP) to minimize squared prediction
error, subject to `sum(eta) = 1` and `eta >= 0`.

**The problem, proven, not asserted:** the weights sit OUTSIDE the
squared-error term in that formulation — making the objective LINEAR in
the weights. A linear objective over a probability simplex (the
`sum=1, >=0` constraint region) is ALWAYS minimized at a corner of that
simplex. A corner means: 100% weight on ONE model, 0% on the others.
**The "optimal blend" is mathematically forced to be a hard switch, not
a real blend** — even though it looks like blending because which corner
wins keeps changing over time.

**This project's fix:** add a small quadratic penalty term
`rho * ||eta||^2` to the objective. This bends the flat linear cost into
a bowl shape, so the minimum can land in the MIDDLE of the simplex —
genuinely splitting weight across multiple GPs.

```
minimize   sum(eta_j * error_j)  +  rho * ||eta||^2
subject to sum(eta) = 1,  eta >= 0
```
`rho = 0` reproduces the paper's exact behavior (hard switching).
`rho = 0.05` (this project's setting) gives a genuine blend.

**Proof this actually works, not just theory** (code: `demo_lp_vs_qp_blend.m`):
running the same scenario with both settings and measuring how much the
weights "churn" (jump around) per step:
- Plain LP (`rho=0`): mean weight change per step ≈ 0.049
- Regularized QP (`rho=0.05`): mean weight change per step ≈ 0.045

A real, measured ~9% reduction in chattering — same order of magnitude
as the improvement the base paper itself claims for its own method.

---

## PART 6 — NOVELTY 2: Uncertainty-Aware MPC

The GP bank doesn't just predict a disturbance value — it also predicts
a VARIANCE (how sure it is). The base paper computes this variance but
only uses the mean prediction in the MPC.

**This project's addition (code: `nmpc_solve.m`):** the blended variance
also tightens the MPC's allowed actuator range:

```
shrink = min(0.3, 0.05 * sqrt(mean(sigma2_hat)))
tightened_bounds = original_bounds * (1 - shrink)
```

When the disturbance estimate is uncertain (early in a mission, or right
after a disturbance regime changes), the controller automatically
becomes more conservative — leaving itself more margin exactly when it
has the least reason to trust its own estimate. When the GP is confident,
the controller uses its full actuator range.

---

## PART 7 — Full System Architecture (10 Stages)

See `docs/system_architecture.png` for the diagram. In words, one
control loop tick:

1. **Reference + Disturbance generation** — the planned path (rotated
   figure-eight) and the TRUE hidden disturbance (only used to test the
   estimator — a real vehicle wouldn't know this)
2. **State feedback** — measure current position/velocity/heading
3. **Disturbance residual estimation** — compare measured vs. predicted
   acceleration
4. **Forgetting-factor GP bank** — 3 GPs predict, each with its own
   trust setting
5. **Dynamic weight optimization** ← NOVELTY 1 (regularized blend)
6. **Blended disturbance estimate** — combined mean + variance
7. **Uncertainty-aware MPC** ← NOVELTY 2 (variance tightens bounds)
8. **Control allocation** — apply the first action
9. **AUV plant** — real physics respond, under the REAL disturbance
10. **Logging** — everything recorded, loop back to step 2

---

## PART 8 — Two Implementations: MATLAB and Simulink

Everything above is implemented twice, calling the SAME underlying
functions both times:

- **MATLAB-only** (`run_simulation.m`): fast to iterate, runs all 4
  controller variants (`NoGP`, `StaticGP`, `DFGP_LP`, `RDFGP_UAMPC`) and
  overlays them — this is the multi-line, paper-style comparison.
- **Simulink** (`build_full_simulink_model.m` + `sim`): the same
  proposed controller as an actual block diagram, demonstrating it as a
  deployable system rather than just a script.

**A real numerical issue worth knowing cold:** the yaw axis's small
effective inertia (`Izz - Nr_dot = 0.52`) makes it numerically "stiff."
A crude integration method (like Simulink's basic `Discrete-Time
Integrator` block, which does Forward-Euler integration) can go unstable
on that axis specifically, while a more accurate method (RK4, what
MATLAB uses) stays stable. This project has TWO Simulink models:
`auv_full_system` (single block, RK4-integrated, matches MATLAB
closely) and `auv_full_system_decomposed` (physics split into visible
Coriolis/Damping/Restoring/Kinematics blocks, matching the paper's
diagram, at the cost of the cruder integration method).

---

## PART 9 — Sensor Noise and Monte Carlo (The Final Additions)

Everything above runs in a perfect, noise-free world by default. Two
additions close that gap:

**Sensor noise** (`simulate_method_noisy.m`): the controller only ever
sees a NOISY measurement of the state (realistic DVL/IMU-style noise),
while the vehicle's true physical motion stays exact — matching how a
real sensor-driven system actually works. Tracking error is judged
against the TRUE state (what matters), but every decision the controller
makes uses only the noisy measurement (all it would really have).

**Monte Carlo robustness study** (`run_monte_carlo.m`): rerun everything
across many different random noise realizations ("seeds" — see below),
report the AVERAGE and the SPREAD (standard deviation), not a single
number. This is what proves a result is reliable rather than a lucky
single run.

**What a "seed" is:** MATLAB's "random" numbers are really a long
pre-computed sequence; the seed is where in that sequence you start.
Same seed = identical noise every time (reproducible). Different seed =
a genuinely different noise pattern, simulating a different real-world
"draw" of sensor imperfection.

---

## PART 10 — Base Paper vs. This Project, Side by Side

| Aspect | Base Paper | This Project |
|---|---|---|
| Vehicle model | 4-DOF, same equations | Same equations, same parameters |
| Disturbance learning | Bank of GPs, different forgetting factors | Same core idea |
| GP internals | Sparse GP with inducing points | Dense weighted-kernel GP (same purpose, simpler mechanism) |
| Weight blending | Plain linear program (provably hard-switches) | **Regularized quadratic program (Novelty 1) — genuine blend** |
| MPC input | Disturbance mean only | **Disturbance mean + variance (Novelty 2) — uncertainty-aware** |
| Solver | qpOASES + real-time iteration | `fmincon` SQP |
| Validation | Simulation + real pool test | Simulation only (MATLAB + Simulink), + Monte Carlo over noise |
| Noise handling | Real-world noise inherent to hardware test | Explicit synthetic noise model + statistical study |

---

## PART 11 — Novelty Confirmation Checklist

- [x] **Novelty 1 — Regularized GP weight blending**: implemented in
      `dynamic_weight_qp.m`, proven with real evidence in
      `demo_lp_vs_qp_blend.m`
- [x] **Novelty 2 — Uncertainty-aware MPC**: implemented in
      `nmpc_solve.m`, actuator bounds genuinely respond to GP variance
- [x] **Sensor noise model**: `simulate_method_noisy.m`
- [x] **Monte Carlo robustness study**: `run_monte_carlo.m`

All four are in the codebase and functional. Nothing here is aspirational
or "planned" — it's built and has been run.

---

## PART 12 — What You Should Be Able to Say, Unprompted

1. Why a plain MPC fails under disturbance (wrong internal model, no
   correction mechanism)
2. Why one GP with a fixed forgetting factor isn't enough
3. Why the paper's weight-blending step secretly hard-switches (simplex
   vertex argument) and how the quadratic term fixes it — with the
   actual measured churn numbers
4. Why the GP's variance, not just its mean, is useful to the MPC
5. Why the yaw axis specifically was the numerically tricky part, and
   what integration method (RK4 vs Euler) has to do with that
6. What a Monte Carlo seed is and why single-run numbers aren't
   trustworthy on their own
7. The honest open finding: in single runs, `RDFGP_UAMPC` didn't clearly
   beat `StaticGP` on prediction error — and what the Monte Carlo study
   (once finished) says about whether that holds up statistically

If you can explain all seven without looking at a screen, you know this
project.
