function plot_simulink_results()
%PLOT_SIMULINK_RESULTS Plots the logged signals from a Simulink run of
%   auv_full_system, styled to resemble the base paper's figures. Run
%   sim('auv_full_system') first, then call this function.
%
%   NOTE: this shows only the proposed method (RDF-GP + uncertainty-aware
%   MPC) - the multi-method comparison that looks like the paper's Figs.
%   3/5/6 (No GP vs StaticGP vs DFGP_LP vs RDFGP_UAMPC all overlaid) comes
%   from run_simulation.m / plot_results.m / generate_comparison_table.m,
%   which already reproduce that side-by-side comparison. Simulink
%   demonstrates one method's full closed loop as a block diagram, not
%   the multi-method study.

vars = {'t_log', 'x_log', 'xref_log', 'dtrue_log', 'dhat_log', 'u_log'};
for i = 1:numel(vars)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', vars{i}))
        error('plot_simulink_results:missingVar', ...
            ['Variable "%s" not found in the base workspace.\n' ...
             'Run the Simulink model with sim(''auv_full_system'') first ' ...
             '(not "out = sim(...)" - that bundles everything into an ' ...
             'object instead of writing it to the workspace).'], vars{i});
    end
end

t      = evalin('base', 't_log');
X      = evalin('base', 'x_log');
Xref   = evalin('base', 'xref_log');
Dtrue  = evalin('base', 'dtrue_log');
Dhat   = evalin('base', 'dhat_log');

t = t(:)';
X     = squeeze(X);
Xref  = squeeze(Xref);
Dtrue = squeeze(Dtrue);
Dhat  = squeeze(Dhat);

% Keep only the actual control-decision points (ControllerBlock holds
% its estimate for SUBSTEPS fine ticks at a time to keep the plant's
% integration stable) - this removes the staircase artifact and matches
% the MATLAB-only simulation's time resolution.
SUBSTEPS = 16;   % must match ControllerBlock.SUBSTEPS
idx = 1:SUBSTEPS:numel(t);
t     = t(idx);
X     = X(:, idx);
Xref  = Xref(:, idx);
Dtrue = Dtrue(:, idx);
Dhat  = Dhat(:, idx);

pos_error = sqrt(sum((X(1:2, :) - Xref(1:2, :)).^2, 1));

% ---- Figure 1: trajectory, paper-style (dashed reference + start marker) --
figure('Name', 'Simulink Trajectory Tracking', 'Color', 'w');
plot(Xref(1,:), Xref(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(X(1,:), X(2,:), 'Color', [0.6 0 0.2], 'LineWidth', 1.6);
plot(X(1,1), X(2,1), 'bx', 'MarkerSize', 12, 'LineWidth', 2.5);
text(X(1,1)+0.05, X(2,1)+0.05, 'Start of trajectory', 'Color', 'b', 'FontSize', 9);
xlabel('X [m]'); ylabel('Y [m]');
title('Obtained Trajectory - RDF-GP + UA-MPC (Simulink)', 'FontWeight', 'bold');
legend('Reference', 'RDF-GP + UA-MPC', 'Location', 'best');
axis equal; grid on; box on;

% ---- Figure 2: |Error| over time, paper-style ------------------------------
figure('Name', 'Simulink Position Error', 'Color', 'w');
plot(t, pos_error, 'Color', [0.6 0 0.2], 'LineWidth', 1.6);
xlabel('Time [s]'); ylabel('|Error| [m]');
title('Trajectory Tracking Error (Simulink)', 'FontWeight', 'bold');
legend(sprintf('RDF-GP + UA-MPC (mean: %.4f)', mean(pos_error)), 'Location', 'best');
grid on; box on;

% ---- Figure 3: disturbance prediction, paper-style -------------------------
figure('Name', 'Simulink Disturbance Estimation (surge axis)', 'Color', 'w');
plot(t, Dtrue(1,:), 'k', 'LineWidth', 1.8); hold on;
plot(t, Dhat(1,:), 'Color', [0.6 0 0.2], 'LineWidth', 1.4);
xlabel('Time [s]'); ylabel('Disturbance [N]');
title('GP Disturbance Prediction - Surge Axis (Simulink)', 'FontWeight', 'bold');
legend('Ground Truth', 'RDF-GP + UA-MPC', 'Location', 'best');
grid on; box on;

fprintf('\n=== Simulink Run Summary ===\n');
fprintf('RMSE position error  : %.4f m\n', sqrt(mean(pos_error.^2)));
fprintf('RMSE disturbance err : %.4f\n', sqrt(mean(sum((Dhat - Dtrue).^2, 1))));
fprintf('\nFor the multi-method paper-style comparison (No GP / StaticGP /\n');
fprintf('DFGP_LP / RDFGP_UAMPC overlaid), run run_simulation.m instead.\n');

end
