function plot_simulink_decomposed_results()
%PLOT_SIMULINK_DECOMPOSED_RESULTS Plots the logged signals from a run of
%   auv_full_system_decomposed (the block-decomposed Plant, matching the
%   base paper's diagram). Run sim('auv_full_system_decomposed') first.

vars = {'t_log_v2', 'x_log_v2', 'xref_log_v2', 'dtrue_log_v2', 'dhat_log_v2'};
for i = 1:numel(vars)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', vars{i}))
        error('plot_simulink_decomposed_results:missingVar', ...
            'Variable "%s" not found. Run sim(''auv_full_system_decomposed'') first.', vars{i});
    end
end

t      = evalin('base', 't_log_v2');
X      = evalin('base', 'x_log_v2');
Xref   = evalin('base', 'xref_log_v2');
Dtrue  = evalin('base', 'dtrue_log_v2');
Dhat   = evalin('base', 'dhat_log_v2');

t = t(:)';
X     = squeeze(X);
Xref  = squeeze(Xref);
Dtrue = squeeze(Dtrue);
Dhat  = squeeze(Dhat);

% Keep only the actual control-decision points (this controller holds
% its estimate for 16 fine ticks at a time to keep the Euler-integrated
% plant stable) - removes the staircase artifact for a cleaner plot.
SUBSTEPS = 16;
idx = 1:SUBSTEPS:numel(t);
t     = t(idx);
X     = X(:, idx);
Xref  = Xref(:, idx);
Dtrue = Dtrue(:, idx);
Dhat  = Dhat(:, idx);

pos_error = sqrt(sum((X(1:2, :) - Xref(1:2, :)).^2, 1));

figure('Name', 'Simulink (Decomposed) Trajectory Tracking', 'Color', 'w');
plot(Xref(1,:), Xref(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(X(1,:), X(2,:), 'Color', [0 0.3 0.6], 'LineWidth', 1.6);
plot(X(1,1), X(2,1), 'bx', 'MarkerSize', 12, 'LineWidth', 2.5);
xlabel('X [m]'); ylabel('Y [m]');
title('Obtained Trajectory - Block-Decomposed Plant (Simulink)', 'FontWeight', 'bold');
legend('Reference', 'RDF-GP + UA-MPC'); axis equal; grid on; box on;

figure('Name', 'Simulink (Decomposed) Position Error', 'Color', 'w');
plot(t, pos_error, 'Color', [0 0.3 0.6], 'LineWidth', 1.6);
xlabel('Time [s]'); ylabel('|Error| [m]');
title('Trajectory Tracking Error - Block-Decomposed Plant (Simulink)', 'FontWeight', 'bold');
grid on; box on;

figure('Name', 'Simulink (Decomposed) Disturbance Estimation', 'Color', 'w');
plot(t, Dtrue(1,:), 'k', 'LineWidth', 1.8); hold on;
plot(t, Dhat(1,:), 'Color', [0 0.3 0.6], 'LineWidth', 1.4);
xlabel('Time [s]'); ylabel('Disturbance [N]');
title('Disturbance Prediction - Block-Decomposed Plant (Simulink)', 'FontWeight', 'bold');
legend('Ground Truth', 'RDF-GP + UA-MPC'); grid on; box on;

fprintf('\n=== Decomposed-Plant Simulink Run Summary ===\n');
fprintf('RMSE position error  : %.4f m\n', sqrt(mean(pos_error.^2)));
fprintf('RMSE disturbance err : %.4f\n', sqrt(mean(sum((Dhat - Dtrue).^2, 1))));

end
