function out = simulate_method_pid_kf(p)
%SIMULATE_METHOD_PID_KF PID controller paired with an Augmented Kalman Filter
%   (KF Disturbance Observer).
%
%   How the Kalman Filter acts like the GP here:
%     - Plain PID (simulate_method_pid.m) has NO disturbance estimator
%       (Dhat = NaN) and reacts only after position error builds up.
%     - Here, an 8-state Augmented Extended Kalman Filter (4 body velocities
%       nu = [ub; vb; wb; r] + 4 unknown disturbance wrenches
%       d = [dx; dy; dz; dMz]) tracks both the filtered velocity state and
%       the unknown external disturbance online at every Ts = 0.2 s step.
%     - The KF disturbance estimate d_hat_kf is subtracted directly as a
%       feedforward compensation term inside the PID control law:
%           u_cmd = u_PID - d_hat_kf
%       while its innovation covariance diag(P_d) provides the uncertainty
%       estimate Sig.
%
%   Returns the exact same struct fields as simulate_method.m and
%   simulate_method_pid.m so it plugs into all plotting and table scripts.

t_vec = 0:p.Ts:p.Tf;
N  = numel(t_vec);
nx = 8; nd = 4;

x = [0; 0; -1; 0; 0; 0; 0; 0];
X_log     = zeros(nx, N);
U_log     = zeros(4, N-1);
Dtrue_log = zeros(nd, N-1);
Dhat_log  = zeros(nd, N-1);
Sig_log   = zeros(nd, N-1);

% --- PID gains (identical to simulate_method_pid.m for a fair test) ------
gains.x   = [8.0,  0.15, 4.0];   % surge  (body X force)
gains.y   = [8.0,  0.15, 4.0];   % sway   (body Y force)
gains.z   = [10.0, 0.20, 5.0];   % heave  (Z force)
gains.psi = [6.0,  0.10, 2.5];   % yaw    (Mz moment)

I_x = 0; I_y = 0; I_z = 0; I_psi = 0;
I_MAX = 5;   % anti-windup clamp on each integral term

% --- Augmented Kalman Filter (KF Disturbance Observer) setup -------------
% Augmented state z_kf = [ub; vb; wb; r; dx; dy; dz; dMz] (8x1)
% Dynamics:
%   nu_{k+1} = nu_k + Ts * M^{-1} * (u_k + C(nu_k)*nu_k + D(nu_k)*nu_k + g + d_k)
%   d_{k+1}  = d_k   (random-walk disturbance model driven by process noise Q_d)
% Measurement:
%   y_k = [ub; vb; wb; r]  (4x1 measured body velocities from DVL/IMU)
M_vec  = [p.m - p.Xu_dot; p.m - p.Yv_dot; p.m - p.Zw_dot; p.Izz - p.Nr_dot];
invM   = diag(1 ./ M_vec);

z_kf   = zeros(8, 1);                  % initial [nu_hat; d_hat]
P_kf   = diag([0.1*ones(1,4), 2.0*ones(1,4)]);

% Process noise covariance Q_kf: small on velocity model, larger random-walk
% variance on disturbance channels so KF tracks time-varying ocean currents
Q_kf   = diag([1e-4, 1e-4, 1e-4, 1e-4,  0.35, 0.35, 0.10, 0.05]);

% Measurement noise covariance R_kf on [ub, vb, wb, r]
R_kf   = diag([2e-3, 2e-3, 2e-3, 2e-3]);

H_kf   = [eye(4), zeros(4, 4)];        % measures velocity states [ub; vb; wb; r]

u_prev = zeros(4, 1);
X_log(:, 1) = x;

for k = 1:N-1
    t = t_vec(k);
    d_true = real_disturbance_profile(t, p);
    Dtrue_log(:, k) = d_true;

    % =====================================================================
    % 1) KALMAN FILTER PREDICT & UPDATE (estimates nu_hat and d_hat_kf)
    % =====================================================================
    if k > 1
        nu_est = z_kf(1:4);
        d_est  = z_kf(5:8);

        % Nominal hydrodynamic forces at current velocity estimate
        xdot_nom = auv_dynamics([0;0;-1;nu_est(1);nu_est(2);nu_est(3);x(7);nu_est(4)], ...
                                u_prev, zeros(4,1), p);
        nu_dot_nom = xdot_nom([4 5 6 8]);

        % Predict augmented state forward by Ts
        nu_pred = nu_est + p.Ts * (nu_dot_nom + invM * d_est);
        d_pred  = d_est;
        z_pred  = [nu_pred; d_pred];

        % State-transition Jacobian F_kf (linearized damping + coupling)
        dD_du = [p.Xu + 2*p.Xuc*abs(nu_est(1));
                 p.Yv + 2*p.Yvc*abs(nu_est(2));
                 p.Zw + 2*p.Zwc*abs(nu_est(3));
                 p.Nr + 2*p.Nrc*abs(nu_est(4))];
        F_nu  = eye(4) + p.Ts * diag(dD_du ./ M_vec);
        F_kf  = [F_nu,          p.Ts * invM;
                 zeros(4, 4),   eye(4)];

        P_pred = F_kf * P_kf * F_kf' + Q_kf;

        % Measurement update with current velocity measurement y_meas
        y_meas = x([4 5 6 8]);
        innov  = y_meas - H_kf * z_pred;
        S_kf   = H_kf * P_pred * H_kf' + R_kf;
        K_kf   = (P_pred * H_kf') / S_kf;

        z_kf   = z_pred + K_kf * innov;
        P_kf   = (eye(8) - K_kf * H_kf) * P_pred;
    else
        z_kf(1:4) = x([4 5 6 8]);
    end

    nu_hat   = z_kf(1:4);
    d_hat_kf = z_kf(5:8);
    sig2_kf  = diag(P_kf(5:8, 5:8));

    % =====================================================================
    % 2) PID CONTROL LAW + KALMAN DISTURBANCE FEEDFORWARD CANCELLATION
    % =====================================================================
    xref = reference_trajectory(t, p);

    % Earth-frame position/heading error
    ex   = xref(1) - x(1);
    ey   = xref(2) - x(2);
    ez   = xref(3) - x(3);
    epsi = atan2(sin(xref(7) - x(7)), cos(xref(7) - x(7)));

    % Rotate surge/sway error into the body frame (heading-relative)
    psi = x(7);
    ex_body =  cos(psi)*ex + sin(psi)*ey;
    ey_body = -sin(psi)*ex + cos(psi)*ey;

    % Integrate with anti-windup clamping
    I_x   = max(min(I_x   + ex_body*p.Ts, I_MAX), -I_MAX);
    I_y   = max(min(I_y   + ey_body*p.Ts, I_MAX), -I_MAX);
    I_z   = max(min(I_z   + ez*p.Ts,      I_MAX), -I_MAX);
    I_psi = max(min(I_psi + epsi*p.Ts,    I_MAX), -I_MAX);

    % Use Kalman-filtered body velocities for derivative damping AND
    % subtract the Kalman disturbance estimate d_hat_kf as feedforward
    u_b = nu_hat(1); v_b = nu_hat(2); w_b = nu_hat(3); r_b = nu_hat(4);

    X_pid  = gains.x(1)*ex_body   + gains.x(2)*I_x   - gains.x(3)*u_b;
    Y_pid  = gains.y(1)*ey_body   + gains.y(2)*I_y   - gains.y(3)*v_b;
    Z_pid  = gains.z(1)*ez        + gains.z(2)*I_z   - gains.z(3)*w_b;
    Mz_pid = gains.psi(1)*epsi    + gains.psi(2)*I_psi - gains.psi(3)*r_b;

    u_cmd = [X_pid; Y_pid; Z_pid; Mz_pid] - d_hat_kf;
    u_cmd = max(min(u_cmd, p.u_max), p.u_min);   % enforce same actuator limits

    % Advance true plant state
    x = rk4_integrate(x, u_cmd, d_true, p, p.Ts);
    u_prev = u_cmd;

    U_log(:, k)    = u_cmd;
    Dhat_log(:, k) = d_hat_kf;
    Sig_log(:, k)  = sig2_kf;
    X_log(:, k+1)  = x;
end

out.t     = t_vec;
out.X     = X_log;
out.U     = U_log;
out.Dtrue = Dtrue_log;
out.Dhat  = Dhat_log;
out.Sig   = Sig_log;
out.Xref  = reference_trajectory(t_vec, p);

out.pos_error       = sqrt(sum((X_log(1:2, :) - out.Xref(1:2, :)).^2, 1));
out.rmse_pos        = sqrt(mean(out.pos_error.^2));
out.dist_pred_error = sqrt(mean(sum((Dhat_log - Dtrue_log).^2, 1)));

end
