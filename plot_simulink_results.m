function plot_simulink_results()
%PLOT_SIMULINK_RESULTS Plots the logged signals from a Simulink run of
%   auv_full_system. Run sim('auv_full_system') first (or press Run in
%   the model window), then call this function.

vars = {'t_log', 'x_log', 'xref_log', 'dtrue_log', 'dhat_log', 'u_log'};
for i = 1:numel(vars)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', vars{i}))
        error('plot_simulink_results:missingVar', ...
            'Variable "%s" not found in the base workspace. Run the Simulink model first.', vars{i});
    end
end

t      = evalin('base', 't_log');
X      = evalin('base', 'x_log');
Xref   = evalin('base', 'xref_log');
Dtrue  = evalin('base', 'dtrue_log');
Dhat   = evalin('base', 'dhat_log');

t = t(:)';

% To Workspace with 'Array' format logs N x M (time along rows) -
% transpose so downstream indexing matches the rest of the project
% (columns = time samples).
X     = X';
Xref  = Xref';
Dtrue = Dtrue';
Dhat  = Dhat';

pos_error = sqrt(sum((X(1:2, :) - Xref(1:2, :)).^2, 1));

figure('Name', 'Simulink Trajectory Tracking');
plot(Xref(1,:), Xref(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(X(1,:), X(2,:), 'r', 'LineWidth', 1.2);
xlabel('x [m]'); ylabel('y [m]');
title('Simulink Model - Trajectory Tracking');
legend('Reference', 'RDF-GP + UA-MPC'); axis equal; grid on;

figure('Name', 'Simulink Position Error');
plot(t, pos_error, 'r', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Position error [m]');
title('Simulink Model - Tracking Error Over Time'); grid on;

figure('Name', 'Simulink Disturbance Estimation (surge axis)');
plot(t, Dtrue(1,:), 'k', 'LineWidth', 1.5); hold on;
plot(t, Dhat(1,:), 'r', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Disturbance [N]');
title('Simulink Model - Disturbance Prediction (Surge Axis)');
legend('Ground truth', 'RDF-GP estimate'); grid on;

fprintf('\nRMSE position error : %.4f m\n', sqrt(mean(pos_error.^2)));
fprintf('RMSE disturbance err : %.4f\n', sqrt(mean(sum((Dhat - Dtrue).^2, 1))));

end
