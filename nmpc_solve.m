function [u_opt, X_pred] = nmpc_solve(x0, xref_seq, d_hat, sigma2_hat, p, u_prev_guess)
%NMPC_SOLVE Single-shooting nonlinear MPC over the AUV model.
%
%   x0           : 8x1 current state
%   xref_seq     : 8 x Nc reference sequence over the horizon
%   d_hat        : 4x1 disturbance estimate fed into the MPC's internal model
%   sigma2_hat   : 4x1 disturbance-estimate variance (zeros if not used)
%   p            : config() struct
%   u_prev_guess : Nc x 4 warm-start control sequence
%
%   Returns the first control action u_opt (4x1) and the predicted state
%   trajectory X_pred (8 x Nc+1).

Nc = p.Nc; nu = 4;
u0vec = reshape(u_prev_guess', [], 1);

lb = repmat(p.u_min, Nc, 1);
ub = repmat(p.u_max, Nc, 1);

% --- Uncertainty-aware constraint tightening (novel contribution) ------
% The more uncertain the disturbance estimate, the more conservative the
% allowed actuator range, giving the controller headroom to react to
% model mismatch instead of saturating.
shrink = min(0.3, 0.05*sqrt(mean(sigma2_hat)));
lb = lb * (1 - shrink);
ub = ub * (1 - shrink);

costfun = @(uvec) mpc_cost(uvec, x0, xref_seq, d_hat, p);
options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'MaxIterations', 60);

% Evaluate the cost directly at the initial guess before handing off to
% fmincon. fmincon's own error message for a bad initial point ("Objective
% function is undefined at initial point") swallows the real underlying
% cause - this makes it surface directly, either as the true MATLAB error
% (if mpc_cost/rk4_integrate actually throws) or as a clear report of
% exactly which input was non-finite.
J0 = mpc_cost(u0vec, x0, xref_seq, d_hat, p);
if ~isfinite(J0)
    error('nmpc_solve:badInitialCost', ...
        ['Initial MPC cost is non-finite (J0 = %g).\n' ...
         'x0        = %s\n' ...
         'd_hat     = %s\n' ...
         'sigma2_hat= %s\n' ...
         'u0vec any non-finite: %d'], ...
        J0, mat2str(x0'), mat2str(d_hat'), mat2str(sigma2_hat'), any(~isfinite(u0vec)));
end

uvec_opt = fmincon(costfun, u0vec, [], [], [], [], lb, ub, [], options);

U = reshape(uvec_opt, nu, Nc)';
u_opt = U(1, :)';
X_pred = simulate_horizon(x0, U, d_hat, p);

end

% ------------------------------------------------------------------------
function J = mpc_cost(uvec, x0, xref_seq, d_hat, p)
Nc = p.Nc; nu = 4;
U = reshape(uvec, nu, Nc)';

x = x0;
J = 0;
for k = 1:Nc
    uk = U(k, :)';
    xref_k = xref_seq(:, min(k, size(xref_seq, 2)));
    e = x - xref_k;
    if k < Nc
        J = J + e'*p.Q*e + uk'*p.R*uk;
    else
        J = J + e'*p.QT*e + uk'*p.R*uk;
    end
    x = rk4_integrate(x, uk, d_hat, p, p.Ts);
end
end

% ------------------------------------------------------------------------
function X_pred = simulate_horizon(x0, U, d_hat, p)
Nc = size(U, 1);
X_pred = zeros(numel(x0), Nc+1);
X_pred(:, 1) = x0;
x = x0;
for k = 1:Nc
    x = rk4_integrate(x, U(k, :)', d_hat, p, p.Ts);
    X_pred(:, k+1) = x;
end
end
