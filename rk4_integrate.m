function x_next = rk4_integrate(x, u, d, p, Ts)
%RK4_INTEGRATE One control-step advance of auv_dynamics, internally
%   subdivided into smaller RK4 sub-steps for numerical stability.
%
%   The yaw axis has a small effective inertia (Izz - Nr_dot = 0.52),
%   which makes its dynamics numerically stiff. A single large RK4 step
%   at the full control-loop Ts can go numerically unstable once the yaw
%   rate grows large (it stops tracking the real physics and amplifies
%   its own error each step, eventually reaching NaN/Inf). Taking several
%   smaller sub-steps instead keeps the same external interface (one
%   call = one Ts advance) while keeping the integration stable across
%   the full range of states the controller might explore, including
%   inside the MPC's own internal multi-step horizon rollout.

N_SUB = 4;
h = Ts / N_SUB;

x_next = x;
for i = 1:N_SUB
    k1 = auv_dynamics(x_next,          u, d, p);
    k2 = auv_dynamics(x_next + h/2*k1, u, d, p);
    k3 = auv_dynamics(x_next + h/2*k2, u, d, p);
    k4 = auv_dynamics(x_next + h*k3,   u, d, p);
    x_next = x_next + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

end
