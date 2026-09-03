function demo_lp_vs_qp_blend()
%DEMO_LP_VS_QP_BLEND Demonstrates the "linear program is a hard switch,
%   not a blend" finding directly from this project's own code. Runs the
%   same disturbance-estimation scenario twice - once with rho = 0 (the
%   plain linear program, as in the base paper's Eq. 26) and once with
%   rho = 0.05 (this project's regularized quadratic program) - and plots
%   how the blend weights eta evolve over time for each, plus reports the
%   mean weight "churn" per step for both.

p = config();
lambdas = [1.0, 0.8, 0.6];
H = 25; Nwin = 20; dimA = 4;

rhos   = [0, 0.05];
labels = {'LP (\rho=0, printed formulation)', 'QP (\rho=0.05, proposed)'};

figure('Name', 'LP vs QP blend weights');

for r = 1:numel(rhos)
    rho = rhos(r);

    gp = ForgettingGP(lambdas, H, dimA);
    lastMus = zeros(numel(lambdas), 1);
    errBuf  = [];
    etaHist = [];

    x = [0; 0; -1; 0; 0; 0; 0; 0];
    x_prev = x;
    u_prev = zeros(4, 1);

    t_vec = 0:p.Ts:90;
    for k = 1:numel(t_vec)-1
        t = t_vec(k);
        d_true = disturbance_profile(t);

        % Open-loop rollout under the true disturbance, just to generate a
        % realistic feature/measurement stream for this standalone demo -
        % no controller is involved, this isolates the weight-blending
        % behaviour on its own.
        x = rk4_integrate(x, zeros(4,1), d_true, p, p.Ts);

        if k > 1
            d_meas = residual_disturbance(x_prev, x, u_prev, p, p.Ts);
            featA  = x_prev([4 5 6 8])';
            gp.addPoint(featA, d_meas(1));
            errRow = (lastMus' - d_meas(1)).^2;
            errBuf = [errBuf; errRow];
            if size(errBuf, 1) > Nwin
                errBuf = errBuf(end-Nwin+1:end, :);
            end
        end

        featStar = x([4 5 6 8])';
        [mus, ~] = gp.predictAll(featStar);

        if size(errBuf, 1) >= 3
            Nrows = size(errBuf, 1);
            alpha = exp(0.05*((1:Nrows) - 1))';
            weightedErr = errBuf .* alpha;
            eta = dynamic_weight_qp(weightedErr, rho);
        else
            eta = zeros(numel(lambdas), 1); eta(1) = 1;
        end

        etaHist = [etaHist; eta'];
        lastMus = mus;
        x_prev  = x;
    end

    subplot(1, 2, r);
    plot(t_vec(1:end-1), etaHist(:,1), t_vec(1:end-1), etaHist(:,2), t_vec(1:end-1), etaHist(:,3));
    xlabel('Time [s]'); ylabel('\eta');
    title(labels{r});
    legend('\lambda=1.0', '\lambda=0.8', '\lambda=0.6');
    ylim([0 1]); grid on;

    churn = mean(sum(abs(diff(etaHist, 1, 1)), 2));
    fprintf('%s: mean |delta eta| per step = %.4f\n', labels{r}, churn);
end

end
