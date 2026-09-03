function plot_results(results, p) %#ok<INUSD>
%PLOT_RESULTS Comparison plots + printed summary table for all methods.

methods = fieldnames(results);
colors  = lines(numel(methods));

% --- Trajectory tracking -------------------------------------------------
figure('Name', 'Trajectory Tracking');
hold on;
ref = results.(methods{1}).Xref;
plot(ref(1, :), ref(2, :), 'k--', 'LineWidth', 1.5);
legendEntries = {'Reference'};
for m = 1:numel(methods)
    r = results.(methods{m});
    plot(r.X(1, :), r.X(2, :), 'Color', colors(m, :), 'LineWidth', 1.2);
    legendEntries{end+1} = methods{m}; %#ok<AGROW>
end
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory Tracking Comparison');
legend(legendEntries, 'Interpreter', 'none');
axis equal; grid on; hold off;

% --- Position tracking error over time -----------------------------------
figure('Name', 'Position Tracking Error');
hold on;
for m = 1:numel(methods)
    r = results.(methods{m});
    plot(r.t, r.pos_error, 'Color', colors(m, :), 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('Position error [m]');
title('Tracking Error Over Time');
legend(methods, 'Interpreter', 'none'); grid on; hold off;

% --- Disturbance prediction on the surge axis -----------------------------
figure('Name', 'Disturbance Estimation (surge axis)');
hold on;
r0 = results.(methods{1});
plot(r0.t(1:end-1), r0.Dtrue(1, :), 'k', 'LineWidth', 1.5);
legendEntries2 = {'Ground truth'};
for m = 1:numel(methods)
    r = results.(methods{m});
    plot(r.t(1:end-1), r.Dhat(1, :), 'Color', colors(m, :));
    legendEntries2{end+1} = methods{m}; %#ok<AGROW>
end
xlabel('Time [s]'); ylabel('Disturbance [N]');
title('Disturbance Prediction - Surge Axis');
legend(legendEntries2, 'Interpreter', 'none'); grid on; hold off;

% --- Summary metrics -------------------------------------------------------
fprintf('\n=== Summary Metrics ===\n');
for m = 1:numel(methods)
    r = results.(methods{m});
    fprintf('%-14s | RMSE pos: %.4f m | Dist. pred. RMSE: %.4f\n', ...
        methods{m}, r.rmse_pos, r.dist_pred_error);
end

end
