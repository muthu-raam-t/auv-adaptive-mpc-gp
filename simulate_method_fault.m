function out = simulate_method_fault(method, p, fault_time, fault_scale)
%SIMULATE_METHOD_FAULT Same closed-loop simulation as simulate_method.m,
%   but from fault_time onward, the vehicle's ACTUAL delivered force is
%   scaled by fault_scale (e.g. [1 1 1 0.5] means the yaw-moment channel
%   only delivers 50% of the commanded force from that point on),
%   simulating a degraded or partially fouled thruster.
%
%   The controller is NOT told about the fault - it keeps commanding
%   based on full actuator capability. This tests whether the existing
%   disturbance-learning pipeline provides inherent resilience: a
%   thruster fault and an external disturbance have the same signature
%   (commanded force does not produce the expected motion), so the same
%   machinery built to reject currents should also partly absorb
%   actuator faults, without any new algorithm being added.
%
%   fault_time  : simulation time [s] at which the fault begins (default 45)
%   fault_scale : 4x1 multiplier on delivered [X Y Z Mz] after fault_time
%                 (default [1 1 1 0.5] - 50% yaw actuation loss)

if nargin < 3, fault_time = 45; end
if nargin < 4, fault_scale = [1 1 1 0.5]; end
fault_scale = fault_scale(:);

t_vec = 0:p.Ts:p.Tf;
N  = numel(t_vec);
nx = 8; nd = 4;

x = [0; 0; -1; 0; 0; 0; 0; 0];
X_log     = zeros(nx, N);
U_log     = zeros(4, N-1);
Dtrue_log = zeros(nd, N-1);
Dhat_log  = zeros(nd, N-1);
Sig_log   = zeros(nd, N-1);
FaultActive_log = false(1, N-1);

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

u_guess = zeros(p.Nc, 4);
x_prev  = x;
u_prev  = zeros(4, 1);
d_hat    = zeros(nd, 1);
sig2_hat = zeros(nd, 1);

X_log(:, 1) = x;

for k = 1:N-1
    t = t_vec(k);
    d_true = disturbance_profile(t);
    Dtrue_log(:, k) = d_true;

    if useGP
        if k > 1
            d_meas = residual_disturbance(x_prev, x, u_prev, p, p.Ts);
            featA  = x_prev([4 5 6 8])';
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

        featStar = x([4 5 6 8])';
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
    [u_opt, ~] = nmpc_solve(x, xref_seq, d_hat, sig2_hat, p, u_guess);
    u_guess = [u_guess(2:end, :); u_opt'];

    x_prev = x;
    u_prev = u_opt;

    % Apply the actuator fault to the TRUE delivered force only - the
    % controller above computed u_opt believing full capability.
    if t >= fault_time
        u_delivered = u_opt .* fault_scale;
        FaultActive_log(k) = true;
    else
        u_delivered = u_opt;
    end

    x = rk4_integrate(x, u_delivered, d_true, p, p.Ts);

    U_log(:, k)    = u_opt;
    Dhat_log(:, k) = d_hat;
    Sig_log(:, k)  = sig2_hat;
    X_log(:, k+1)  = x;
end

out.t      = t_vec;
out.X      = X_log;
out.U      = U_log;
out.Dtrue  = Dtrue_log;
out.Dhat   = Dhat_log;
out.Sig    = Sig_log;
out.FaultActive = FaultActive_log;
out.fault_time  = fault_time;
out.fault_scale = fault_scale;
out.Xref   = reference_trajectory(t_vec, p);

out.pos_error = sqrt(sum((X_log(1:2, :) - out.Xref(1:2, :)).^2, 1));
out.rmse_pos  = sqrt(mean(out.pos_error.^2));

if useGP
    out.dist_pred_error = sqrt(mean(sum((Dhat_log - Dtrue_log).^2, 1)));
else
    out.dist_pred_error = NaN;
end

end
