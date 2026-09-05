function out = simulate_method_noisy(method, p, noise_std, seed)
%SIMULATE_METHOD_NOISY Same closed-loop simulation as simulate_method.m,
%   but the controller only ever sees a NOISY sensor measurement of the
%   state (Gaussian noise added to the velocity/yaw-rate channels,
%   matching a realistic DVL/IMU noise floor), while the vehicle's TRUE
%   physical motion stays exact. Tracking error is still computed
%   against the true state (that is what actually matters), but every
%   decision the controller makes - the disturbance estimate, the GP
%   update, the MPC solve - is computed from the noisy measurement only,
%   since that is genuinely all a real controller would have access to.
%
%   This directly answers the gap flagged in the project's own planning:
%   a noise-free simulation cannot show whether a result is realistic,
%   only how the algorithm behaves in an idealised world.
%
%   noise_std : std of the added measurement noise on [ub vb wb r]
%               (default 0.01, a modest DVL-like noise level)
%   seed      : RNG seed, for reproducible Monte Carlo runs (default 0)

if nargin < 3, noise_std = 0.01; end
if nargin < 4, seed = 0; end
rng(seed);

t_vec = 0:p.Ts:p.Tf;
N  = numel(t_vec);
nx = 8; nd = 4;

x = [0; 0; -1; 0; 0; 0; 0; 0];   % true physical state
X_log    = zeros(nx, N);
U_log    = zeros(4, N-1);
Dtrue_log = zeros(nd, N-1);
Dhat_log  = zeros(nd, N-1);
Sig_log   = zeros(nd, N-1);

useGP = ~strcmp(method, 'NoGP');

if useGP
    dimA = 4; H = 25; Nwin = 20;
    if strcmp(method, 'StaticGP')
        lambdas = 0.9;
    else
        lambdas = [1.0, 0.8, 0.6];
    end
    rho = 0.0;
    if strcmp(method, 'RDFGP_UAMPC')
        rho = 0.05;
    end
    gpBank  = cell(nd, 1);
    lastMus = cell(nd, 1);
    errBuf  = cell(nd, 1);
    for i = 1:nd
        gpBank{i}  = ForgettingGP(lambdas, H, dimA);
        lastMus{i} = zeros(numel(lambdas), 1);
        errBuf{i}  = [];
    end
end

u_guess     = zeros(p.Nc, 4);
x_meas_prev = x;
u_prev      = zeros(4, 1);
d_hat       = zeros(nd, 1);
sig2_hat    = zeros(nd, 1);

X_log(:, 1) = x;

for k = 1:N-1
    t = t_vec(k);

    d_true = disturbance_profile(t);
    Dtrue_log(:, k) = d_true;

    % Noisy sensor measurement of the current TRUE state - all the
    % controller actually gets to see.
    x_meas = x;
    x_meas([4 5 6 8]) = x_meas([4 5 6 8]) + noise_std*randn(4,1);

    if useGP
        if k > 1
            d_meas = residual_disturbance(x_meas_prev, x_meas, u_prev, p, p.Ts);
            featA  = x_meas_prev([4 5 6 8])';
            for i = 1:nd
                gpBank{i}.addPoint(featA, d_meas(i));
                if mod(k, 10) == 0
                    gpBank{i}.fitHyperparameters();
                end
                if ~strcmp(method, 'StaticGP')
                    errRow = (lastMus{i}' - d_meas(i)).^2;
                    errBuf{i} = [errBuf{i}; errRow];
                    if size(errBuf{i}, 1) > Nwin
                        errBuf{i} = errBuf{i}(end-Nwin+1:end, :);
                    end
                end
            end
        end

        featStar = x_meas([4 5 6 8])';
        for i = 1:nd
            [mus, sig2s] = gpBank{i}.predictAll(featStar);
            if strcmp(method, 'StaticGP')
                d_hat(i)    = mus(1);
                sig2_hat(i) = sig2s(1);
            else
                Kn = numel(lambdas);
                if size(errBuf{i}, 1) >= 3
                    Nrows = size(errBuf{i}, 1);
                    alpha = exp(0.05*((1:Nrows) - 1))';
                    weightedErr = errBuf{i} .* alpha;
                    eta = dynamic_weight_qp(weightedErr, rho);
                else
                    eta = zeros(Kn, 1); eta(1) = 1;
                end
                d_hat(i)    = eta' * mus;
                sig2_hat(i) = eta' * sig2s;
            end
            lastMus{i} = mus;
        end
    end

    t_horizon = t + (1:p.Nc)*p.Ts;
    xref_seq  = reference_trajectory(t_horizon, p);

    % MPC solves using the NOISY measured state - genuinely all a real
    % controller would have.
    [u_opt, ~] = nmpc_solve(x_meas, xref_seq, d_hat, sig2_hat, p, u_guess);
    u_guess = [u_guess(2:end, :); u_opt'];

    x_meas_prev = x_meas;
    u_prev = u_opt;

    % TRUE physics propagate from the TRUE previous state - the
    % vehicle's real motion is unaffected by sensor noise, only the
    % controller's knowledge of it is.
    x = rk4_integrate(x, u_opt, d_true, p, p.Ts);

    U_log(:, k)     = u_opt;
    Dhat_log(:, k)  = d_hat;
    Sig_log(:, k)   = sig2_hat;
    X_log(:, k+1)   = x;
end

out.t      = t_vec;
out.X      = X_log;
out.U      = U_log;
out.Dtrue  = Dtrue_log;
out.Dhat   = Dhat_log;
out.Sig    = Sig_log;
out.Xref   = reference_trajectory(t_vec, p);

out.pos_error = sqrt(sum((X_log(1:2, :) - out.Xref(1:2, :)).^2, 1));
out.rmse_pos  = sqrt(mean(out.pos_error.^2));

if useGP
    out.dist_pred_error = sqrt(mean(sum((Dhat_log - Dtrue_log).^2, 1)));
else
    out.dist_pred_error = NaN;
end

end
