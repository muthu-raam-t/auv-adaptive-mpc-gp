function plot_results(results, p) %#ok<INUSD>
%PLOT_RESULTS Comparison plots + printed summary table for all methods.

methods = fieldnames(results);
colors  = lines(numel(methods));

% --- Trajectory tracking -------------------------------------------------
fig1 = figure('Name', 'Trajectory Tracking', 'Color', 'white');
ax1 = axes(fig1, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'FontSize', 10);
hold(ax1, 'on');
ref = results.(methods{1}).Xref;
plot(ax1, ref(1, :), ref(2, :), '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.5);
legendEntries = {'Reference'};
for m = 1:numel(methods)
    r = results.(methods{m});
    plot(ax1, r.X(1, :), r.X(2, :), 'Color', colors(m, :), 'LineWidth', 1.2);
    legendEntries{end+1} = methods{m}; %#ok<AGROW>
end
xlabel(ax1, 'x [m]', 'Color', 'black'); ylabel(ax1, 'y [m]', 'Color', 'black');
title(ax1, 'Trajectory Tracking Comparison', 'Color', 'black');
legend(ax1, legendEntries, 'Interpreter', 'none', 'TextColor', 'black');
axis(ax1, 'equal'); grid(ax1, 'on'); box(ax1, 'on'); hold(ax1, 'off');

% --- Position tracking error over time -----------------------------------
fig2 = figure('Name', 'Position Tracking Error', 'Color', 'white');
ax2 = axes(fig2, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'FontSize', 10);
hold(ax2, 'on');
for m = 1:numel(methods)
    r = results.(methods{m});
    plot(ax2, r.t, r.pos_error, 'Color', colors(m, :), 'LineWidth', 1.2);
end
xlabel(ax2, 'Time [s]', 'Color', 'black'); ylabel(ax2, 'Position error [m]', 'Color', 'black');
title(ax2, 'Tracking Error Over Time', 'Color', 'black');
legend(ax2, methods, 'Interpreter', 'none', 'TextColor', 'black');
grid(ax2, 'on'); box(ax2, 'on'); hold(ax2, 'off');

% --- Disturbance prediction on the surge axis -----------------------------
% Methods with no disturbance estimator at all (e.g. PID) are skipped here
% - an all-NaN line would just clutter the legend with nothing to show.
fig3 = figure('Name', 'Disturbance Estimation (surge axis)', 'Color', 'white');
ax3 = axes(fig3, 'Color', 'white', 'XColor', 'black', 'YColor', 'black', 'FontSize', 10);
hold(ax3, 'on');
r0 = results.(methods{1});
plot(ax3, r0.t(1:end-1), r0.Dtrue(1, :), 'Color', [0.15 0.15 0.15], 'LineWidth', 1.8);
legendEntries2 = {'Ground truth'};
for m = 1:numel(methods)
    r = results.(methods{m});
    if all(isnan(r.Dhat(1, :)))
        continue;   % no estimator for this method (e.g. PID) - nothing to plot
    end
    plot(ax3, r.t(1:end-1), r.Dhat(1, :), 'Color', colors(m, :));
    legendEntries2{end+1} = methods{m}; %#ok<AGROW>
end
xlabel(ax3, 'Time [s]', 'Color', 'black'); ylabel(ax3, 'Disturbance [N]', 'Color', 'black');
title(ax3, 'Disturbance Prediction - Surge Axis', 'Color', 'black');
legend(ax3, legendEntries2, 'Interpreter', 'none', 'TextColor', 'black');
grid(ax3, 'on'); box(ax3, 'on'); hold(ax3, 'off');

% --- Summary metrics -------------------------------------------------------
fprintf('\n=== Summary Metrics ===\n');
for m = 1:numel(methods)
    r = results.(methods{m});
    fprintf('%-14s | RMSE pos: %.4f m | Dist. pred. RMSE: %.4f\n', ...
        methods{m}, r.rmse_pos, r.dist_pred_error);
end

end
