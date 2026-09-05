function results_mc = run_monte_carlo(n_seeds, noise_std)
%RUN_MONTE_CARLO Repeats each controller variant over multiple random
%   sensor-noise realizations and reports mean +/- std RMSE instead of a
%   single-run number. This is what turns "the proposed method got this
%   one number" into a real statistical claim, and directly answers the
%   robustness study this project's own planning listed as outstanding.
%
%   Usage:
%       results_mc = run_monte_carlo();        % defaults: 10 seeds, std 0.01
%       results_mc = run_monte_carlo(20, 0.02); % 20 seeds, larger noise
%
%   NOTE ON RUNTIME: each seed runs all 4 controller variants through a
%   full 90s simulation (real optimisation at every step). 10 seeds
%   means 40 full runs - expect this to take a while. Start with a
%   small n_seeds to confirm it works, then increase overnight if you
%   want the full 20-seed study.

if nargin < 1, n_seeds = 10; end
if nargin < 2, noise_std = 0.01; end

fprintf('Running %d seeds x 4 methods = %d full simulations.\n', n_seeds, n_seeds*4);
fprintf('This will take a while - each simulation solves a real optimisation\n');
fprintf('problem at every one of ~450 steps.\n\n');

p = config();
methods = {'NoGP', 'StaticGP', 'DFGP_LP', 'RDFGP_UAMPC'};

rmse_pos_all  = zeros(n_seeds, numel(methods));
rmse_dist_all = zeros(n_seeds, numel(methods));

for s = 1:n_seeds
    fprintf('--- Seed %d/%d ---\n', s, n_seeds);
    for m = 1:numel(methods)
        out = simulate_method_noisy(methods{m}, p, noise_std, s);
        rmse_pos_all(s, m)  = out.rmse_pos;
        rmse_dist_all(s, m) = out.dist_pred_error;
        fprintf('    %-14s pos RMSE = %.4f m\n', methods{m}, out.rmse_pos);
    end
end

fprintf('\n=== Monte Carlo Summary (%d seeds, noise std = %.3f) ===\n\n', n_seeds, noise_std);
fprintf('%-14s %14s %14s %14s %14s\n', 'Method', 'PosRMSE mean', 'PosRMSE std', 'DistRMSE mean', 'DistRMSE std');
fprintf('%s\n', repmat('-', 1, 74));
for m = 1:numel(methods)
    fprintf('%-14s %14.4f %14.4f %14.4f %14.4f\n', methods{m}, ...
        mean(rmse_pos_all(:,m)), std(rmse_pos_all(:,m)), ...
        mean(rmse_dist_all(:,m), 'omitnan'), std(rmse_dist_all(:,m), 'omitnan'));
end

% ---- Plot: mean +/- std error bars across methods --------------------------
figure('Name', 'Monte Carlo Robustness Study', 'Color', 'w');

subplot(1,2,1);
bar(mean(rmse_pos_all,1)); hold on;
errorbar(1:numel(methods), mean(rmse_pos_all,1), std(rmse_pos_all,1), 'k.', 'LineWidth', 1.2);
set(gca, 'XTickLabel', methods); xtickangle(30);
ylabel('Tracking RMSE [m]');
title(sprintf('Position Tracking (mean \\pm std, %d seeds)', n_seeds));
grid on;

subplot(1,2,2);
bar(mean(rmse_dist_all,1,'omitnan')); hold on;
errorbar(1:numel(methods), mean(rmse_dist_all,1,'omitnan'), std(rmse_dist_all,1,'omitnan'), 'k.', 'LineWidth', 1.2);
set(gca, 'XTickLabel', methods); xtickangle(30);
ylabel('Disturbance Prediction RMSE');
title(sprintf('Disturbance Prediction (mean \\pm std, %d seeds)', n_seeds));
grid on;

results_mc.methods       = methods;
results_mc.rmse_pos_all  = rmse_pos_all;
results_mc.rmse_dist_all = rmse_dist_all;
results_mc.noise_std     = noise_std;
results_mc.n_seeds       = n_seeds;

if ~exist('results', 'dir'); mkdir('results'); end
save('results/monte_carlo_results.mat', 'results_mc');
fprintf('\nSaved results/monte_carlo_results.mat\n');

end
