%% run_pid_kf_vs_mpc_gp.m
%
% Standalone benchmark script comparing:
%   1) PID          - Purely reactive 4-axis PID (no estimator)
%   2) PID_KF       - 4-axis PID + Augmented Kalman Filter Disturbance Observer
%                     (KF acts like the GP by estimating unknown disturbance
%                      d_hat_kf online and cancelling it via feedforward)
%   3) RDFGP_UAMPC  - Proposed Regularized Dynamic-Forgetting GP +
%                     Uncertainty-Aware Nonlinear MPC
%
% Generates dedicated white-background comparison plots and prints a summary
% table comparing both Position Tracking RMSE and Disturbance Prediction RMSE.

clear; clc; close all;
addpath(genpath(pwd));

p = config();

fprintf('--- Running 1/3: Plain PID (no estimator) ---\n');
res_pid = simulate_method_pid(p);

fprintf('--- Running 2/3: PID + Kalman Filter (PID_KF) ---\n');
res_pid_kf = simulate_method_pid_kf(p);

fprintf('--- Running 3/3: Proposed MPC + RDF-GP (RDFGP_UAMPC) ---\n');
res_mpc_gp = simulate_method('RDFGP_UAMPC', p);

% --- Print Quantitative Comparison Table ---------------------------------
fprintf('\n=====================================================================\n');
fprintf('   PID vs. PID + Kalman Filter vs. Proposed MPC + RDF-GP Benchmark\n');
fprintf('=====================================================================\n');
fprintf('%-22s | %18s | %22s\n', 'Controller Method', 'Tracking RMSE [m]', 'Disturbance Pred RMSE');
fprintf('%s\n', repmat('-', 1, 69));
fprintf('%-22s | %18.4f | %22s\n', 'PID (Reactive only)',  res_pid.rmse_pos,    'N/A (no estimator)');
fprintf('%-22s | %18.4f | %22.4f\n', 'PID + Kalman (PID_KF)', res_pid_kf.rmse_pos, res_pid_kf.dist_pred_error);
fprintf('%-22s | %18.4f | %22.4f\n', 'MPC + RDF-GP (Ours)',   res_mpc_gp.rmse_pos, res_mpc_gp.dist_pred_error);
fprintf('=====================================================================\n\n');

% --- Combined 3-Panel Comparison Figure ----------------------------------
fig = figure('Name', 'PID_KF_vs_MPC_GP_Comparison', 'Color', 'white', ...
             'Position', [60 60 1380 430]);

% 1) Figure-8 Trajectory Tracking
subplot(1, 3, 1);
plot(res_pid.Xref(1,:), res_pid.Xref(2,:), 'k--', 'LineWidth', 1.6); hold on;
plot(res_pid.X(1,:),    res_pid.X(2,:),    'Color', [0.47 0.67 0.19], 'LineWidth', 1.3);
plot(res_pid_kf.X(1,:), res_pid_kf.X(2,:), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
plot(res_mpc_gp.X(1,:), res_mpc_gp.X(2,:), 'Color', [0.49 0.18 0.56], 'LineWidth', 1.7);
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory Tracking Comparison', 'FontWeight', 'bold');
legend({'Reference', 'PID (No Estimator)', 'PID + Kalman Filter', 'MPC + RDF-GP (Ours)'}, ...
       'Location', 'northeast', 'FontSize', 8.5);
axis equal; grid on; box on;

% 2) Position Tracking Error Over Time
subplot(1, 3, 2);
plot(res_pid.t,    res_pid.pos_error,    'Color', [0.47 0.67 0.19], 'LineWidth', 1.3); hold on;
plot(res_pid_kf.t, res_pid_kf.pos_error, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
plot(res_mpc_gp.t, res_mpc_gp.pos_error, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.7);
xlabel('Time [s]'); ylabel('Position error [m]');
title('Position Tracking Error Over Time', 'FontWeight', 'bold');
legend({sprintf('PID (RMSE: %.4f m)', res_pid.rmse_pos), ...
        sprintf('PID + KF (RMSE: %.4f m)', res_pid_kf.rmse_pos), ...
        sprintf('MPC + RDF-GP (RMSE: %.4f m)', res_mpc_gp.rmse_pos)}, ...
       'Location', 'northeast', 'FontSize', 8.5);
grid on; box on;

% 3) Surge Disturbance Estimation (Kalman Filter vs. RDF-GP)
subplot(1, 3, 3);
t_d = res_mpc_gp.t(1:end-1);
plot(t_d, res_mpc_gp.Dtrue(1,:), 'k', 'LineWidth', 1.8); hold on;
plot(t_d, res_pid_kf.Dhat(1,:),  'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
plot(t_d, res_mpc_gp.Dhat(1,:),  'Color', [0.49 0.18 0.56], 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Surge Disturbance [N]');
title('Disturbance Estimation: KF vs. RDF-GP', 'FontWeight', 'bold');
legend({'Ground Truth (NorKyst)', ...
        sprintf('Kalman Filter (RMSE: %.4f)', res_pid_kf.dist_pred_error), ...
        sprintf('RDF-GP Bank (RMSE: %.4f)', res_mpc_gp.dist_pred_error)}, ...
       'Location', 'northeast', 'FontSize', 8.5);
grid on; box on;

if exist('force_white_figure', 'file')
    force_white_figure(fig);
end
if ~exist('slide_graphs', 'dir'), mkdir('slide_graphs'); end
exportgraphics(fig, fullfile('slide_graphs', 'PID_KF_vs_MPC_GP_Comparison.png'), ...
    'Resolution', 200, 'BackgroundColor', 'white');
fprintf('Saved slide_graphs/PID_KF_vs_MPC_GP_Comparison.png\n');
