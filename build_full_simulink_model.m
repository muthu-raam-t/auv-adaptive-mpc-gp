function build_full_simulink_model()
%BUILD_FULL_SIMULINK_MODEL Builds the complete closed-loop AUV system in
%   Simulink with a clean two-level structure, matching the base paper's
%   layout: a tidy top-level diagram (reference, disturbance, controller,
%   plant, logging), with the vehicle physics (Coriolis, Damping,
%   Restoring, Kinematics - still separate, clearly labeled blocks)
%   encapsulated inside a single "Plant" Subsystem block instead of
%   scattered loose at the top level.
%
%   RUN THIS ONCE from MATLAB. It creates auv_full_system.slx in the
%   current folder and opens it. Everything runs at one fixed discrete
%   sample rate (config().Ts).
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

% =============================================================================
% TOP LEVEL: Clock, Reference, Disturbance, Controller, Plant, Loggers
% =============================================================================

add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [40 260 70 280]);

add_ms_block(modelName, 'Reference',  'ReferenceBlock',  [160 40  300 90]);
add_ms_block(modelName, 'Disturbance','DisturbanceBlock',[160 160 300 210]);
add_ms_block(modelName, 'Controller', 'ControllerBlock', [160 300 340 400], 'Interpreted Execution');

% ---- Plant subsystem (built-in Subsystem block) ---------------------------
plantPath = [modelName '/Plant'];
add_block('built-in/Subsystem', plantPath, 'Position', [500 260 660 420]);
build_plant_subsystem(plantPath, Ts);

% ---- Loggers ---------------------------------------------------------------
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

add_line(modelName, 'Plant/1', 'Controller/2');    % x feedback
add_line(modelName, 'Plant/1', 'x_log/1');
add_line(modelName, 'Plant/2', 'nudot_log/1');

add_line(modelName, 'Reference/1',   'xref_log/1');
add_line(modelName, 'Disturbance/1', 'dtrue_log/1');
add_line(modelName, 'Controller/2',  'dhat_log/1');
add_line(modelName, 'Controller/1',  'u_log/1');

% ---- Auto-arrange for a clean, non-overlapping layout ----------------------
Simulink.BlockDiagram.arrangeSystem(modelName);
Simulink.BlockDiagram.arrangeSystem(plantPath);

save_system(modelName);
fprintf('Saved %s.slx in %s\n', modelName, pwd);
fprintf('Run it with:  sim(''%s'');  then  plot_simulink_results();\n', modelName);

end

% =============================================================================
function build_plant_subsystem(plantPath, Ts)
%BUILD_PLANT_SUBSYSTEM Populate the Plant subsystem with the AUV physics:
%   Coriolis / Damping / Restoring / Kinematics as separate blocks, summed
%   and scaled by inv(M), then integrated - the exact structure from the
%   base paper's plant diagram, just nested one level down instead of
%   sitting loose at the top of the model.

add_block('simulink/Sources/In1',  [plantPath '/tau'],   'Port', '1', 'Position', [30 40 60 60]);
add_block('simulink/Sources/In1',  [plantPath '/Delta'], 'Port', '2', 'Position', [30 200 60 220]);
add_block('simulink/Sinks/Out1',   [plantPath '/x'],     'Port', '1', 'Position', [820 40  850 60]);
add_block('simulink/Sinks/Out1',   [plantPath '/nudot'], 'Port', '2', 'Position', [820 300 850 320]);

add_ms_block(plantPath, 'Coriolis', 'CoriolisBlock', [140 20  260 90]);
add_ms_block(plantPath, 'Damping',  'DampingBlock',  [140 130 260 200]);
add_block('simulink/Sources/Constant', [plantPath '/Restoring_g'], ...
    'Value', '[0; 0; -0.981; 0]', 'Position', [140 240 260 280]);

add_block('simulink/Math Operations/Sum', [plantPath '/Sum_of_forces'], ...
    'Inputs', '+++++', 'Position', [310 30 340 290]);

add_block('simulink/Math Operations/Gain', [plantPath '/invM'], ...
    'Gain', 'diag([1/14.00, 1/29.90, 1/24.70, 1/0.52])', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [380 130 440 200]);

add_block('simulink/Discrete/Discrete-Time Integrator', [plantPath '/Integrator_nu'], ...
    'SampleTime', num2str(Ts), 'Position', [480 130 520 200]);

add_ms_block(plantPath, 'Kinematics', 'KinematicsBlock', [480 340 620 410]);

add_block('simulink/Discrete/Discrete-Time Integrator', [plantPath '/Integrator_eta'], ...
    'SampleTime', num2str(Ts), 'Position', [660 340 700 410]);

add_ms_block(plantPath, 'StateAssembly', 'StateAssemblyBlock', [700 30 830 100]);

add_line(plantPath, 'tau/1',           'Sum_of_forces/1');
add_line(plantPath, 'Coriolis/1',      'Sum_of_forces/2');
add_line(plantPath, 'Damping/1',       'Sum_of_forces/3');
add_line(plantPath, 'Restoring_g/1',   'Sum_of_forces/4');
add_line(plantPath, 'Delta/1',         'Sum_of_forces/5');

add_line(plantPath, 'Sum_of_forces/1', 'invM/1');
add_line(plantPath, 'invM/1',           'Integrator_nu/1');
add_line(plantPath, 'Integrator_nu/1', 'nudot/1');

add_line(plantPath, 'Integrator_nu/1', 'Coriolis/1');
add_line(plantPath, 'Integrator_nu/1', 'Damping/1');
add_line(plantPath, 'Integrator_nu/1', 'Kinematics/2');
add_line(plantPath, 'Integrator_nu/1', 'StateAssembly/2');

add_line(plantPath, 'Kinematics/1',     'Integrator_eta/1');
add_line(plantPath, 'Integrator_eta/1', 'Kinematics/1');
add_line(plantPath, 'Integrator_eta/1', 'StateAssembly/1');

add_line(plantPath, 'StateAssembly/1', 'x/1');

end

% =============================================================================
function blk = add_ms_block(parentPath, name, className, pos, simMode)
%ADD_MS_BLOCK Add a MATLAB System block referencing a classdef file.
%   simMode (optional): 'Code Generation' (default) or 'Interpreted Execution'.
%   Use 'Interpreted Execution' for blocks whose code isn't code-generation
%   compatible (e.g. anything calling fmincon/quadprog, or using dynamically
%   growing arrays) - it runs the exact same MATLAB code path as the
%   MATLAB-only simulation, just without compiling to C first.
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
