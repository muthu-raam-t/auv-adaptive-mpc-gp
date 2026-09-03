function build_full_simulink_model()
%BUILD_FULL_SIMULINK_MODEL Builds the complete closed-loop AUV system in
%   Simulink: reference generator, disturbance generator, plant (Coriolis
%   / Damping / Restoring / Kinematics as separate visible blocks), and
%   the proposed RDF-GP + uncertainty-aware MPC controller - all wired
%   together, plus logging (To Workspace) blocks for post-run plotting.
%
%   RUN THIS ONCE from MATLAB. It creates auv_full_system.slx in the
%   current folder and opens it. Everything runs at one fixed discrete
%   sample rate (config().Ts), so no continuous/discrete rate-transition
%   issues arise.
%
%   After building, run it with:
%       sim('auv_full_system');
%       plot_simulink_results();
%
%   or just click Run inside the Simulink window.

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
set_param(modelName, 'StopTime', '20');   % short first run; raise once confirmed working

% ---- Clock ---------------------------------------------------------------
add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [30 30 60 50]);

% ---- Reference, Disturbance, Controller (MATLAB System blocks) -----------
add_ms_block(modelName, 'Reference',  'ReferenceBlock',  [140 20  270 60]);
add_ms_block(modelName, 'Disturbance','DisturbanceBlock',[140 100 270 140]);
add_ms_block(modelName, 'Controller', 'ControllerBlock', [140 250 300 340]);

% ---- Plant: Coriolis / Damping / Restoring / Kinematics -------------------
add_ms_block(modelName, 'Coriolis',   'CoriolisBlock',   [420 40  540 110]);
add_ms_block(modelName, 'Damping',    'DampingBlock',    [420 150 540 220]);
add_block('simulink/Sources/Constant', [modelName '/Restoring_g'], ...
    'Value', '[0; 0; -0.981; 0]', 'Position', [420 260 540 300]);

add_block('simulink/Math Operations/Sum', [modelName '/Sum_of_forces'], ...
    'Inputs', '+++++', 'Position', [590 40 620 340]);

add_block('simulink/Math Operations/Gain', [modelName '/invM'], ...
    'Gain', 'diag([1/14.00, 1/29.90, 1/24.70, 1/0.52])', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [660 150 720 220]);

add_block('simulink/Discrete/Discrete-Time Integrator', [modelName '/Integrator_nu'], ...
    'SampleTime', num2str(Ts), 'Position', [760 150 800 220]);

add_ms_block(modelName, 'Kinematics', 'KinematicsBlock', [760 380 900 450]);

add_block('simulink/Discrete/Discrete-Time Integrator', [modelName '/Integrator_eta'], ...
    'SampleTime', num2str(Ts), 'Position', [940 380 980 450]);

add_ms_block(modelName, 'StateAssembly', 'StateAssemblyBlock', [1020 60 1160 140]);

% ---- Logging (To Workspace) -----------------------------------------------
add_to_workspace(modelName, 't_log',     [1220 30]);
add_to_workspace(modelName, 'x_log',     [1220 90]);
add_to_workspace(modelName, 'xref_log',  [1220 150]);
add_to_workspace(modelName, 'dtrue_log', [1220 210]);
add_to_workspace(modelName, 'dhat_log',  [1220 270]);
add_to_workspace(modelName, 'u_log',     [1220 330]);

% ---- Wiring ----------------------------------------------------------------
add_line(modelName, 'Clock/1', 'Reference/1');
add_line(modelName, 'Clock/1', 'Disturbance/1');
add_line(modelName, 'Clock/1', 'Controller/1');
add_line(modelName, 'Clock/1', 't_log/1');

add_line(modelName, 'Controller/1',   'Sum_of_forces/1');   % u -> tau
add_line(modelName, 'Coriolis/1',     'Sum_of_forces/2');
add_line(modelName, 'Damping/1',      'Sum_of_forces/3');
add_line(modelName, 'Restoring_g/1',  'Sum_of_forces/4');
add_line(modelName, 'Disturbance/1',  'Sum_of_forces/5');

add_line(modelName, 'Sum_of_forces/1', 'invM/1');
add_line(modelName, 'invM/1',           'Integrator_nu/1');

add_line(modelName, 'Integrator_nu/1', 'Coriolis/1');
add_line(modelName, 'Integrator_nu/1', 'Damping/1');
add_line(modelName, 'Integrator_nu/1', 'Kinematics/2');
add_line(modelName, 'Integrator_nu/1', 'StateAssembly/2');

add_line(modelName, 'Kinematics/1',     'Integrator_eta/1');
add_line(modelName, 'Integrator_eta/1', 'Kinematics/1');
add_line(modelName, 'Integrator_eta/1', 'StateAssembly/1');

add_line(modelName, 'StateAssembly/1', 'Controller/2');   % x feedback
add_line(modelName, 'StateAssembly/1', 'x_log/1');

add_line(modelName, 'Reference/1',    'xref_log/1');
add_line(modelName, 'Disturbance/1',  'dtrue_log/1');
add_line(modelName, 'Controller/2',   'dhat_log/1');
add_line(modelName, 'Controller/1',   'u_log/1');

save_system(modelName);
fprintf('Saved %s.slx in %s\n', modelName, pwd);
fprintf('Run it with:  sim(''%s'');  then  plot_simulink_results();\n', modelName);

end

% =============================================================================
function blk = add_ms_block(modelName, name, className, pos)
%ADD_MS_BLOCK Add a MATLAB System block referencing a classdef file.
blk = add_block('simulink/User-Defined Functions/MATLAB System', ...
    [modelName '/' name], 'Position', pos);
set_param(blk, 'System', className);
end

% =============================================================================
function blk = add_to_workspace(modelName, varName, posTopLeft)
%ADD_TO_WORKSPACE Add a "To Workspace" logger block.
pos = [posTopLeft(1), posTopLeft(2), posTopLeft(1)+90, posTopLeft(2)+30];
blk = add_block('simulink/Sinks/To Workspace', [modelName '/' varName], ...
    'VariableName', varName, 'SaveFormat', 'Array', ...
    'MaxDataPoints', 'inf', 'Position', pos);
end
