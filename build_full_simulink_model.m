function build_full_simulink_model()
%BUILD_FULL_SIMULINK_MODEL Builds the complete closed-loop AUV system in
%   Simulink, matching the MATLAB-only simulation (run_simulation.m /
%   simulate_method.m) as closely as possible: the same fixed step (Ts),
%   the same auv_dynamics.m / rk4_integrate.m functions called at the
%   same rate, and the same controller logic (ControllerBlock mirrors
%   simulate_method.m's per-step body exactly). This is what makes the
%   Simulink run's results match the MATLAB run's results, rather than
%   drifting apart from two different numerical schemes.
%
%   RUN THIS ONCE from MATLAB. It creates auv_full_system.slx in the
%   current folder and opens it.
%
%   After building, run it with:
%       sim('auv_full_system');
%       plot_simulink_results();

modelName = 'auv_full_system';
p = config();
Ts = p.Ts;

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(Ts));
set_param(modelName, 'StopTime', num2str(p.Tf));
set_param(modelName, 'ReturnWorkspaceOutputs', 'off');

% =============================================================================
% TOP LEVEL: Clock, Reference, Disturbance, Controller, Plant, Loggers
% =============================================================================

add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [40 260 70 280]);

add_ms_block(modelName, 'Reference',  'ReferenceBlock',  [160 40  300 90],  'Interpreted Execution');
add_ms_block(modelName, 'Disturbance','DisturbanceBlock',[160 160 300 210], 'Interpreted Execution');
add_ms_block(modelName, 'Controller', 'ControllerBlock', [160 300 340 400], 'Interpreted Execution');
add_ms_block(modelName, 'Plant',      'PlantBlock',      [500 260 660 360], 'Interpreted Execution');

% Explicit Unit Delay on the Plant -> Controller feedback wire ONLY. This
% is what actually breaks the Controller <-> Plant algebraic loop - a
% native Simulink delay block is always recognized reliably by the
% solver, unlike relying on a custom MATLAB System object's internal
% state timing (which repeatedly was not honored in practice). The
% logging path (Plant/1 -> x_log) stays undelayed so plots show the true
% state, not the one-step-delayed copy the controller sees.
add_block('simulink/Discrete/Unit Delay', [modelName '/UnitDelay_x'], ...
    'Position', [700 320 740 360]);
add_to_workspace(modelName, 't_log',     [820 30]);
add_to_workspace(modelName, 'x_log',     [820 90]);
add_to_workspace(modelName, 'xref_log',  [820 150]);
add_to_workspace(modelName, 'dtrue_log', [820 210]);
add_to_workspace(modelName, 'dhat_log',  [820 270]);
add_to_workspace(modelName, 'u_log',     [820 330]);
add_to_workspace(modelName, 'nudot_log', [820 390]);

% ---- Wiring -----------------------------------------------------------------
add_line(modelName, 'Clock/1', 'Reference/1');
add_line(modelName, 'Clock/1', 'Disturbance/1');
add_line(modelName, 'Clock/1', 'Controller/1');
add_line(modelName, 'Clock/1', 't_log/1');

add_line(modelName, 'Controller/1',  'Plant/1');   % u -> tau
add_line(modelName, 'Disturbance/1', 'Plant/2');   % d_true -> Delta

add_line(modelName, 'Plant/1', 'UnitDelay_x/1');
add_line(modelName, 'UnitDelay_x/1', 'Controller/2');    % delayed x feedback
add_line(modelName, 'Plant/1', 'x_log/1');               % undelayed for logging
add_line(modelName, 'Plant/2', 'nudot_log/1');

add_line(modelName, 'Reference/1',   'xref_log/1');
add_line(modelName, 'Disturbance/1', 'dtrue_log/1');
add_line(modelName, 'Controller/2',  'dhat_log/1');
add_line(modelName, 'Controller/1',  'u_log/1');

Simulink.BlockDiagram.arrangeSystem(modelName);

save_system(modelName);
fprintf('Saved %s.slx in %s\n', modelName, pwd);
fprintf('Fixed step = Ts = %.2f s, matching run_simulation.m exactly.\n', Ts);
fprintf('Run it with:  sim(''%s'');  then  plot_simulink_results();\n', modelName);

end

% =============================================================================
function blk = add_ms_block(parentPath, name, className, pos, simMode)
%ADD_MS_BLOCK Add a MATLAB System block referencing a classdef file.
blk = add_block('simulink/User-Defined Functions/MATLAB System', ...
    [parentPath '/' name], 'Position', pos);
set_param(blk, 'System', className);
if nargin >= 5 && ~isempty(simMode)
    set_param(blk, 'SimulateUsing', simMode);
end
end

% =============================================================================
function blk = add_to_workspace(modelName, varName, posTopLeft)
%ADD_TO_WORKSPACE Add a "To Workspace" logger block.
pos = [posTopLeft(1), posTopLeft(2), posTopLeft(1)+90, posTopLeft(2)+30];
blk = add_block('simulink/Sinks/To Workspace', [modelName '/' varName], ...
    'VariableName', varName, 'SaveFormat', 'Array', ...
    'MaxDataPoints', 'inf', 'Position', pos);
end
