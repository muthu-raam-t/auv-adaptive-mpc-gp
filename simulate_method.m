function out = simulate_method(method, p)
%SIMULATE_METHOD Run the closed-loop AUV simulation for one controller
%   variant and return logged signals + summary metrics.
%
%   method: one of
%     'NoGP'         - nominal MPC, no disturbance estimation
%     'StaticGP'     - single fixed-forgetting-factor GP-MPC (paper baseline)
%     'DFGP_LP'      - multi-GP dynamic forgetting, plain LP weights (paper method)
%     'RDFGP_UAMPC'  - regularized dynamic forgetting GP + uncertainty-aware MPC (proposed)

t_vec = 0:p.Ts:p.Tf;
N  = numel(t_vec);
nx = 8; nd = 4;

x = [0; 0; -1; 0; 0; 0; 0; 0];
X_log    = zeros(nx, N);
U_log    = zeros(4, N-1);
Dtrue_log = zeros(nd, N-1);
Dhat_log  = zeros(nd, N-1);
Sig_log   = zeros(nd, N-1);

useGP = ~strcmp(method, 'NoGP');

if useGP
    dimA = 4;   % GP input feature = [ub vb wb r]
    H    = 25;  % training buffer size
    Nwin = 20;  % dynamic weight optimization window (matches paper's N)

    if strcmp(method, 'StaticGP')
        lambdas = 0.9;
    else
        lambdas = [1.0, 0.8, 0.6];
    end

    rho = 0.0;
    if strcmp(method, 'RDFGP_UAMPC')
        rho = 0.05;   % regularization strength (proposed smoothing)
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
d_hat   = zeros(nd, 1);
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
                    errRow = (lastMus{i}' - d_meas(i)).^2;   % 1 x K
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
                    alpha = exp(0.05*((1:Nrows) - 1))';  % recent rows weighted highest
                    weightedErr = errBuf{i} .* alpha;
                    eta = dynamic_weight_qp(weightedErr, rho);
                else
                    eta = zeros(Kn, 1); eta(1) = 1;      % trust lambda=1 model early on
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
