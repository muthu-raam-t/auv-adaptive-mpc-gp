function industrial_metrics_report()
%INDUSTRIAL_METRICS_REPORT Reframes the already-generated simulation
%   results as business-relevant metrics an operations team would
%   actually judge a system on, instead of abstract RMSE numbers:
%
%   1) Mission-spec compliance - what fraction of the mission stayed
%      within a stated tracking tolerance (e.g. the accuracy needed for
%      usable inspection camera footage)
%   2) Control effort - total actuator energy used, a proxy for battery
%      drain and mission range
%
%   Reads the already-saved results/simulation_results.mat from
%   run_simulation.m - does not run any new simulation, so there is no
%   risk of introducing a new bug this close to a deadline.
%
%   Usage: industrial_metrics_report()   (run run_simulation.m first)

if ~exist('results/simulation_results.mat', 'file')
    error('industrial_metrics_report:missingResults', ...
        'results/simulation_results.mat not found. Run run_simulation first.');
end
load('results/simulation_results.mat', 'results');

spec_tolerance = 0.30;   % [m] - example inspection-quality tracking tolerance
methods = {'NoGP', 'StaticGP', 'DFGP_LP', 'RDFGP_UAMPC'};

fprintf('\n=== Industrial Readiness Report ===\n');
fprintf('Mission-spec tolerance: %.2f m (example: usable inspection camera footage)\n\n', spec_tolerance);
fprintf('%-14s %20s %18s %16s\n', 'Method', '%% time in spec', 'Total effort', 'Effort vs NoGP');
fprintf('%s\n', repmat('-', 1, 70));

compliance = zeros(1, numel(methods));
effort     = zeros(1, numel(methods));

for m = 1:numel(methods)
    r = results.(methods{m});
    compliance(m) = 100 * mean(r.pos_error <= spec_tolerance);
    effort(m)     = sum(sqrt(sum(r.U.^2, 1)) * (r.t(2)-r.t(1)));
end

effort_ratio = effort / effort(1);

for m = 1:numel(methods)
    fprintf('%-14s %19.1f%% %18.1f %15.2fx\n', methods{m}, compliance(m), effort(m), effort_ratio(m));
end

fprintf('\nReading this: higher %% time in spec = more of the mission produced\n');
fprintf('usable inspection data. Effort ratio > 1.0 = uses more actuator\n');
fprintf('energy than the no-learning baseline, a proxy for reduced range.\n');

figure('Name', 'Industrial Readiness Metrics', 'Color', 'white', 'Position', [100 100 1000 450]);

subplot(1,2,1);
b1 = bar(compliance, 'FaceColor', [0.20 0.45 0.70]);
ax1 = gca;
set(ax1, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
    'GridColor', [0.75 0.75 0.75], 'FontSize', 11);
set(ax1, 'XTickLabel', methods);
xtickangle(30);
ylabel('% of mission within spec', 'Color', 'black', 'FontWeight', 'bold');
title(sprintf('Mission-Spec Compliance (tolerance = %.2fm)', spec_tolerance), ...
    'Color', 'black', 'FontWeight', 'bold');
grid on; box on;

subplot(1,2,2);
b2 = bar(effort_ratio, 'FaceColor', [0.70 0.30 0.20]);
ax2 = gca;
set(ax2, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
    'GridColor', [0.75 0.75 0.75], 'FontSize', 11);
set(ax2, 'XTickLabel', methods);
xtickangle(30);
ylabel('Control effort relative to No-GP', 'Color', 'black', 'FontWeight', 'bold');
title('Actuator Energy Usage', 'Color', 'black', 'FontWeight', 'bold');
grid on; box on;

end
