function out = simulate_method_pid(p)
%SIMULATE_METHOD_PID Independent per-axis PID baseline - no prediction,
%   no disturbance learning, reacts only to the current tracking error.
%   Represents the industry-standard controller most real ROVs/AUVs
%   actually run today, as a comparison point independent of the base
%   paper's MPC lineage.
%
%   Returns the same output structure as simulate_method.m, so it plugs
%   directly into your existing plotting and comparison functions.

t_vec = 0:p.Ts:p.Tf;
N = numel(t_vec);
nx = 8; nd = 4;

x = [0; 0; -1; 0; 0; 0; 0; 0];
X_log = zeros(nx, N);
U_log = zeros(4, N-1);
Dtrue_log = zeros(nd, N-1);

% PID gains, one set per axis: [Kp Ki Kd]
gains.x   = [8.0, 0.15, 4.0];   % surge  (body X force)
gains.y   = [8.0, 0.15, 4.0];   % sway   (body Y force)
gains.z   = [10.0, 0.20, 5.0];  % heave  (Z force)
gains.psi = [6.0, 0.10, 2.5];   % yaw    (Mz moment)

I_x = 0; I_y = 0; I_z = 0; I_psi = 0;
I_MAX = 5;   % anti-windup clamp on each integral term

X_log(:, 1) = x;

for k = 1:N-1
    t = t_vec(k);
    d_true = disturbance_profile(t);
    Dtrue_log(:, k) = d_true;

    xref = reference_trajectory(t, p);

    % Earth-frame position/heading error
    ex = xref(1) - x(1);
    ey = xref(2) - x(2);
    ez = xref(3) - x(3);
    epsi = wrapToPi(xref(7) - x(7));

    % Rotate surge/sway error into the body frame (heading-relative)
    psi = x(7);
    ex_body =  cos(psi)*ex + sin(psi)*ey;
    ey_body = -sin(psi)*ex + cos(psi)*ey;

    % Integrate with anti-windup clamping
    I_x   = max(min(I_x   + ex_body*p.Ts, I_MAX), -I_MAX);
    I_y   = max(min(I_y   + ey_body*p.Ts, I_MAX), -I_MAX);
    I_z   = max(min(I_z   + ez*p.Ts,      I_MAX), -I_MAX);
    I_psi = max(min(I_psi + epsi*p.Ts,    I_MAX), -I_MAX);

    % Derivative term approximated directly from the measured body rates
    % (standard PID practice - avoids amplifying noise from differencing
    % the error signal itself)
    u_b = x(4); v_b = x(5); w_b = x(6); r_b = x(8);

    X  = gains.x(1)*ex_body   + gains.x(2)*I_x   - gains.x(3)*u_b;
    Y  = gains.y(1)*ey_body   + gains.y(2)*I_y   - gains.y(3)*v_b;
    Z  = gains.z(1)*ez        + gains.z(2)*I_z   - gains.z(3)*w_b;
    Mz = gains.psi(1)*epsi    + gains.psi(2)*I_psi - gains.psi(3)*r_b;

    u_cmd = [X; Y; Z; Mz];
    u_cmd = max(min(u_cmd, p.u_max), p.u_min);   % same actuator limits as the MPC methods

    x = rk4_integrate(x, u_cmd, d_true, p, p.Ts);

    U_log(:, k) = u_cmd;
    X_log(:, k+1) = x;
end

out.t = t_vec;
out.X = X_log;
out.U = U_log;
out.Dtrue = Dtrue_log;
out.Dhat = NaN(nd, N-1);     % PID has no disturbance estimate at all
out.Sig = NaN(nd, N-1);
out.Xref = reference_trajectory(t_vec, p);

out.pos_error = sqrt(sum((X_log(1:2, :) - out.Xref(1:2, :)).^2, 1));
out.rmse_pos = sqrt(mean(out.pos_error.^2));
out.dist_pred_error = NaN;   % no learning system to score

end
