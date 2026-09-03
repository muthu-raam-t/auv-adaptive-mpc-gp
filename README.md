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
much of it changes character over the course of a mission: a current that
was steady a minute ago can shift direction, strengthen, or turn choppy
with no warning.

## Problem Statement

Model predictive control (MPC) is a natural fit for this kind of vehicle:
it optimizes a sequence of actions over a future horizon while respecting
actuator and state limits, rather than responding to error moment by
moment. But an MPC controller is only as good as the model it optimizes
against, and an AUV's model is never fully accurate. The unmodeled portion
— currents, tether pull, thrust degradation, wall effects — behaves like
an external disturbance acting on the vehicle, and if the controller has
no way to estimate it, tracking accuracy degrades exactly when precision
matters most.

A natural fix is to learn that disturbance online using a Gaussian Process
(GP): GPs are non-parametric, so they don't force a particular structure
onto the disturbance, and they report an uncertainty alongside every
prediction. The difficulty is that a single GP with hyperparameters tuned
once, offline, cannot keep up if the disturbance's behavior changes
mid-mission — a model tuned for a slow, repeating current lags behind a
sudden squall, and a model tuned for fast transients is noisy the rest of
the time. Introducing a forgetting factor that discounts older samples
helps, but a single fixed value is again only right for one timescale,
and there is no principled rule for choosing it online.

## Solution Provided

This project implements a learning-based MPC framework for a reduced,
4-degree-of-freedom AUV model (surge, sway, heave, yaw) that resolves the
above by maintaining several Gaussian Processes in parallel, each
discounting past data at a different rate, and blending their disturbance
predictions online based on which one has been tracking the true
disturbance best over a recent window. This removes the need to commit to
a single forgetting factor or to retune anything offline once the mission
starts.

Two extensions are added on top of that base idea:

- **Regularized blending across the GP bank.** Combining several
  candidate GPs by minimizing a plain squared-error objective over a
  weight simplex is a linear program, and a linear program over a simplex
  is always optimized at a vertex — meaning the "optimal blend" collapses
  to hard-switching between models rather than genuinely combining them.
  Adding a quadratic regularization term turns this into a small
  quadratic program instead, producing a smooth blend across the GP bank
  and avoiding that switching behavior.

- **Uncertainty-aware constraint tightening in the MPC.** Rather than
  handing the controller only the GP bank's mean disturbance estimate,
  the blended predictive variance is also fed back in, tightening the
  actuator limits during periods where the disturbance estimate is
  unreliable. This gives the controller some built-in margin instead of
  treating every disturbance estimate as equally trustworthy.

The full pipeline — vehicle dynamics, disturbance and reference
generation, the GP-based estimator, and the uncertainty-aware MPC — is
implemented both as a MATLAB simulation and as a closed-loop Simulink
model, with the underlying vehicle physics (Coriolis, damping, restoring
force, and kinematics) built as separate, clearly labeled blocks rather
than folded into one opaque function, so the governing equations remain
visible and traceable through the block diagram.
