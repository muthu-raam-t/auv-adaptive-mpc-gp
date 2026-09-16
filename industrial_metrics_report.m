function industrial_metrics_report()
%INDUSTRIAL_METRICS_REPORT Reframes existing simulation results as
%   business-relevant metrics an operations team would actually judge a
%   system on, instead of abstract RMSE numbers:
%
%   1) Mission-spec compliance in a clean run - what fraction of the
%      mission stayed within a stated tracking tolerance
%   2) Control effort - total actuator energy used, a proxy for battery
%      drain and mission range
%   3) Mission-spec compliance BEFORE vs. AFTER a mid-mission thruster
%      fault - this is where methods actually diverge, since a clean
%      run alone does not stress-test the system enough to separate them
%
%   Usage: industrial_metrics_report()   (run run_simulation.m first)

if ~exist('results/simulation_results.mat', 'file')
    error('industrial_metrics_report:missingResults', ...
        'results/simulation_results.mat not found. Run run_simulation first.');
end
load('results/simulation_results.mat', 'results');

spec_tolerance = 0.30;   % [m] - example inspection-quality tracking tolerance
methods = {'NoGP', 'StaticGP', 'DFGP_LP', 'RDFGP_UAMPC'};

% ---- Part 1: clean-run compliance + effort --------------------------------
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

fprintf('\nNote: in a clean, undisturbed run all methods perform similarly -\n');
fprintf('the real industrial question is whether spec compliance HOLDS UP\n');
fprintf('when something goes wrong. See Part 2 below.\n');

% ---- Part 2: fault-scenario compliance, before vs. after ------------------
fprintf('\n=== Spec Compliance Under Thruster Fault ===\n');
fprintf('Same %.2fm tolerance, before vs. after a mid-mission thruster fault\n\n', spec_tolerance);

p = config();
fault_time = 45;
fault_scale = [1 1 1 0.5];

out_nogp     = simulate_method_fault('NoGP', p, fault_time, fault_scale);
out_proposed = simulate_method_fault('RDFGP_UAMPC', p, fault_time, fault_scale);

before_idx = out_nogp.t < fault_time;
after_idx  = ~before_idx;

comp_nogp     = [100*mean(out_nogp.pos_error(before_idx)     <= spec_tolerance), ...
                 100*mean(out_nogp.pos_error(after_idx)      <= spec_tolerance)];
comp_proposed = [100*mean(out_proposed.pos_error(before_idx) <= spec_tolerance), ...
                 100*mean(out_proposed.pos_error(after_idx)  <= spec_tolerance)];

fprintf('%-14s %18s %18s\n', 'Method', 'Before fault', 'After fault');
fprintf('%s\n', repmat('-', 1, 52));
fprintf('%-14s %17.1f%% %17.1f%%\n', 'NoGP', comp_nogp(1), comp_nogp(2));
fprintf('%-14s %17.1f%% %17.1f%%\n', 'RDFGP_UAMPC', comp_proposed(1), comp_proposed(2));

% ---- Figures ----------------------------------------------------------------
figure('Name', 'Industrial Readiness Metrics', 'Color', 'white', 'Position', [80 80 1000 450]);

subplot(1,2,1);
bar(compliance, 'FaceColor', [0.20 0.45 0.70]);
ax1 = gca;
set(ax1, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
    'GridColor', [0.75 0.75 0.75], 'FontSize', 11, 'XTickLabel', methods);
xtickangle(30);
ylabel('% of mission within spec', 'Color', 'black', 'FontWeight', 'bold');
title(sprintf('Clean-Run Spec Compliance (tol = %.2fm)', spec_tolerance), 'Color', 'black', 'FontWeight', 'bold');
grid on; box on;

subplot(1,2,2);
bar(effort_ratio, 'FaceColor', [0.70 0.30 0.20]);
ax2 = gca;
set(ax2, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
    'GridColor', [0.75 0.75 0.75], 'FontSize', 11, 'XTickLabel', methods);
xtickangle(30);
ylabel('Control effort relative to No-GP', 'Color', 'black', 'FontWeight', 'bold');
title('Actuator Energy Usage', 'Color', 'black', 'FontWeight', 'bold');
grid on; box on;

figure('Name', 'Spec Compliance Under Fault', 'Color', 'white', 'Position', [80 560 700 450]);
bardata = [comp_nogp; comp_proposed];
bh = bar(bardata, 'grouped');
bh(1).FaceColor = [0.55 0.55 0.55];
bh(2).FaceColor = [0.20 0.55 0.30];
ax3 = gca;
set(ax3, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
    'GridColor', [0.75 0.75 0.75], 'FontSize', 11, ...
    'XTickLabel', {'NoGP', 'RDFGP\_UAMPC'});
ylabel('% of time within spec', 'Color', 'black', 'FontWeight', 'bold');
title(sprintf('Spec Compliance Before vs. After Thruster Fault (t=%ds)', fault_time), ...
    'Color', 'black', 'FontWeight', 'bold');
legend({'Before fault', 'After fault'}, 'TextColor', 'black', 'Location', 'best');
grid on; box on;

end
