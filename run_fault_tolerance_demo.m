function run_fault_tolerance_demo()
%RUN_FAULT_TOLERANCE_DEMO Simulates a mid-mission actuator (thruster)
%   fault - the yaw-moment channel drops to 50% effectiveness partway
%   through the run, as if fouled or partially damaged - and compares
%   tracking performance with and without the learning-based
%   disturbance-estimation pipeline. The point: a thruster fault and an
%   environmental disturbance look identical to the controller
%   (commanded force does not produce the expected motion), so a system
%   built to reject currents should also partly absorb actuator faults.

p = config();
fault_time  = 45;
fault_scale = [1 1 1 0.5];

fprintf('Fault scenario: yaw actuation drops to %.0f%% effectiveness at t = %.0f s.\n\n', ...
    100*fault_scale(4), fault_time);

out_nogp     = simulate_method_fault('NoGP', p, fault_time, fault_scale);
out_proposed = simulate_method_fault('RDFGP_UAMPC', p, fault_time, fault_scale);

before_idx = out_nogp.t < fault_time;
after_idx  = ~before_idx;

fprintf('=== Before fault (t < %d s) ===\n', fault_time);
fprintf('  NoGP          RMSE: %.4f m\n', sqrt(mean(out_nogp.pos_error(before_idx).^2)));
fprintf('  RDFGP_UAMPC   RMSE: %.4f m\n', sqrt(mean(out_proposed.pos_error(before_idx).^2)));

fprintf('\n=== After fault (t >= %d s) ===\n', fault_time);
fprintf('  NoGP          RMSE: %.4f m\n', sqrt(mean(out_nogp.pos_error(after_idx).^2)));
fprintf('  RDFGP_UAMPC   RMSE: %.4f m\n', sqrt(mean(out_proposed.pos_error(after_idx).^2)));

figure('Name', 'Fault Tolerance - Trajectory', 'Color', 'w');
plot(out_nogp.Xref(1,:), out_nogp.Xref(2,:), 'k--', 'LineWidth', 1.3); hold on;
plot(out_nogp.X(1,:), out_nogp.X(2,:), 'r', 'LineWidth', 1.4);
plot(out_proposed.X(1,:), out_proposed.X(2,:), 'b', 'LineWidth', 1.4);
legend('Reference', 'No GP', 'RDF-GP + UA-MPC');
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory Under Mid-Mission Thruster Fault');
axis equal; grid on;

figure('Name', 'Fault Tolerance - Tracking Error', 'Color', 'w');
plot(out_nogp.t, out_nogp.pos_error, 'r', 'LineWidth', 1.3); hold on;
plot(out_proposed.t, out_proposed.pos_error, 'b', 'LineWidth', 1.3);
xline(fault_time, 'k--', 'Fault onset', 'LineWidth', 1.5);
legend('No GP', 'RDF-GP + UA-MPC');
xlabel('Time [s]'); ylabel('Position error [m]');
title('Tracking Error Before / After Actuator Fault');
grid on;

end
