# AUV Adaptive MPC with Regularized Dynamic-Forgetting Gaussian Processes

A MATLAB implementation of a learning-based model predictive control (MPC)
framework for autonomous underwater vehicle (AUV) trajectory tracking under
unknown, time-varying environmental disturbances such as currents, tether
drag, and unmodeled hydrodynamic effects.

## Background

Model predictive control is a natural fit for AUV navigation because it
handles actuator and state constraints directly, but its tracking
performance is only as good as the underlying vehicle model. In practice,
AUV models are never exact: added-mass terms, drag coefficients, tether
dynamics, and current-induced forces all introduce errors that a purely
model-based controller cannot compensate for.

A common fix is to estimate the *lumped* disturbance — everything the
model gets wrong, bundled into one term — using a Gaussian Process (GP).
GPs are attractive here because they are non-parametric and provide a
variance estimate alongside every prediction, which is useful for
building safer, more conservative control laws. The catch is that a
single GP with fixed hyperparameters struggles once the disturbance
statistics change: a model tuned to slowly varying currents will lag
behind a sudden squall, and a model tuned for fast transients will be
noisy during calm periods.

This project follows a *forgetting-factor* approach to that problem:
instead of one GP, maintain several GPs that each discount older
training samples at a different rate, and combine their predictions
based on how well each has performed recently. Models with a forgetting
factor close to 1 behave like a standard GP and excel at repeating,
slowly-varying disturbances; models with a lower forgetting factor react
faster to abrupt changes at the cost of more noise. Weighting them
online — rather than committing to one forgetting factor offline —
removes the need to re-tune the estimator every time the operating
conditions shift.

## What this project adds

Two extensions are layered on top of the base dynamic-forgetting scheme:

1. **Regularized weight blending.** A naive linear-programming weight
   optimization across the candidate GPs is a linear objective over a
   simplex constraint, which mathematically collapses to picking a
   single "best" model at each step — a hard switch that can chatter
   between models when their errors are close. Adding a quadratic
   regularization term turns the optimization into a small quadratic
   program and yields a smoother blend across models instead of a hard
   selection.

2. **Uncertainty-aware constraint tightening.** Rather than feeding only
   the GP's mean disturbance estimate into the MPC's internal model, the
   predicted variance is also used to shrink the actuator bounds when the
   estimator is unsure. This gives the controller some built-in margin
   during periods of high disturbance uncertainty instead of treating
   every estimate as equally trustworthy.

## Repository layout

```
config.m                 Physical, control, and simulation parameters
auv_dynamics.m            Nonlinear AUV plant model (surge/sway/heave/yaw)
rk4_integrate.m           4th-order Runge-Kutta discretization
disturbance_profile.m     Ground-truth disturbance generator (sim only)
reference_trajectory.m    Lemniscate reference trajectory generator
residual_disturbance.m    Back-calculates the measured disturbance for GP training
ForgettingGP.m             Weighted-GP class, one instance per disturbance axis
dynamic_weight_qp.m       Online weight blending across forgetting-factor GPs
nmpc_solve.m               Nonlinear MPC (fmincon), with uncertainty tightening
simulate_method.m          Closed-loop simulation loop for one controller variant
plot_results.m             Comparison plots and summary metrics
run_simulation.m           Main script - run this one
results/                   Saved simulation output (created automatically)
```

## How to run

1. Open MATLAB and `cd` into this folder (or add it to the path).
2. Requires the **Optimization Toolbox** (`fmincon`, `quadprog`).
3. Run:

   ```matlab
   run_simulation
   ```

That single script simulates all four controller variants back to back,
saves the results to `results/simulation_results.mat`, and produces three
comparison figures: trajectory tracking, position error over time, and
disturbance-prediction accuracy, followed by a printed summary table of
RMSE tracking error and disturbance-prediction error for each method.

No other file needs to be run directly — everything else is called from
`run_simulation.m` / `simulate_method.m`.

## Tuning knobs

- `config.m` — sample time, horizon length, actuator limits, MPC weights.
- `simulate_method.m` — GP buffer size `H`, weight-optimization window
  `Nwin`, forgetting-factor bank `lambdas`, and the regularization
  strength `rho` used by the proposed RDF-GP variant.
- `disturbance_profile.m` — the disturbance scenario used to stress-test
  each estimator.
