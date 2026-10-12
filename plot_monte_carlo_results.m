function fig = plot_monte_carlo_results(results_mc)
%PLOT_MONTE_CARLO_RESULTS Draws the Monte Carlo robustness figure (mean +/- std
%   across seeds) from a saved results_mc struct, on a white background.
%   Lets you redraw the figure without re-running the 40 simulations:
%
%       S = load('results/monte_carlo_results.mat');
%       plot_monte_carlo_results(S.results_mc);

methods = results_mc.methods;
rp      = results_mc.rmse_pos_all;
rd      = results_mc.rmse_dist_all;
n_seeds = results_mc.n_seeds;
x       = 1:numel(methods);

fig = figure('Name', 'Monte Carlo Robustness Study', 'Color', 'white', ...
             'Position', [100 100 1000 420]);

subplot(1,2,1);
bar(mean(rp,1)); hold on;
errorbar(x, mean(rp,1), std(rp,0,1), 'k.', 'LineWidth', 1.2);
set(gca, 'XTick', x, 'XTickLabel', methods, 'TickLabelInterpreter', 'none');
xtickangle(30);
ylabel('Tracking RMSE [m]');
title(sprintf('Position Tracking (mean \\pm std, %d seeds)', n_seeds));
grid on;

subplot(1,2,2);
bar(mean(rd,1,'omitnan')); hold on;
errorbar(x, mean(rd,1,'omitnan'), std(rd,0,1,'omitnan'), 'k.', 'LineWidth', 1.2);
set(gca, 'XTick', x, 'XTickLabel', methods, 'TickLabelInterpreter', 'none');
xtickangle(30);
ylabel('Disturbance Prediction RMSE');
title(sprintf('Disturbance Prediction (mean \\pm std, %d seeds)', n_seeds));
grid on;

force_white_figure(fig);
end
