function d_hat = residual_disturbance(x_prev, x_curr, u_prev, p, Ts)
%RESIDUAL_DISTURBANCE Estimate the lumped disturbance that actually acted
%   on the vehicle between x_prev and x_curr, given the applied control
%   u_prev. This is the "measurement" used to train the GP models: it
%   compares the observed acceleration against the disturbance-free
%   nominal model evaluated at the same state/input.

xdot_meas = (x_curr - x_prev) / Ts;
xdot_nom  = auv_dynamics(x_prev, u_prev, [0;0;0;0], p);

M = diag([p.m - p.Xu_dot, p.m - p.Yv_dot, p.m - p.Zw_dot, p.Izz - p.Nr_dot]);

accel_error = xdot_meas([4 5 6 8]) - xdot_nom([4 5 6 8]);
d_hat = M * accel_error;

end
