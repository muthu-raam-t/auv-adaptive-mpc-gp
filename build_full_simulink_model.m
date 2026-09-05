function build_full_simulink_model()
%BUILD_FULL_SIMULINK_MODEL Builds the complete closed-loop AUV system in
%   Simulink with a clean two-level structure: a tidy top-level diagram
%   (reference, disturbance, controller, plant, logging), with the
%   vehicle physics (Coriolis, Damping, Restoring, Kinematics - separate,
%   clearly labeled blocks, matching the base paper's plant diagram)
%   encapsulated inside a single "Plant" Subsystem block.
%
%   IMPORTANT ON TIMING: the model's fixed step is set much finer
%   (Ts/PLANT_SUBSTEPS) than the control decision rate (Ts). The yaw axis
%   has a small effective inertia (Izz - Nr_dot = 0.52), making its
%   dynamics numerically stiff - the Discrete-Time Integrator blocks
%   inside Plant use Forward-Euler integration, which is only stable at
%   a small enough step. Running the whole model at the finer step makes
%   that integration stable; ControllerBlock internally only re-solves
%   the expensive GP+MPC logic once every PLANT_SUBSTEPS ticks (holding
%   its output in between), so the actual control rate - and the
%   simulation's wall-clock cost - stays the same as before.
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

PLANT_SUBSTEPS = 16;   % must match ControllerBlock.SUBSTEPS
model_step = Ts / PLANT_SUBSTEPS;

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(model_step));
set_param(modelName, 'StopTime', '20');   % short first run; raise once confirmed working
set_param(modelName, 'ReturnWorkspaceOutputs', 'off');   % write To Workspace logs
                                                           % straight to the base
                                                           % workspace, not bundled
                                                           % into a SimulationOutput
                                                           % object

% =============================================================================
% TOP LEVEL: Clock, Reference, Disturbance, Controller, Plant, Loggers
% =============================================================================

add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [40 260 70 280]);

add_ms_block(modelName, 'Reference',  'ReferenceBlock',  [160 40  300 90],  'Interpreted Execution');
add_ms_block(modelName, 'Disturbance','DisturbanceBlock',[160 160 300 210], 'Interpreted Execution');
add_ms_block(modelName, 'Controller', 'ControllerBlock', [160 300 340 400], 'Interpreted Execution');

% ---- Plant subsystem (built-in Subsystem block) ---------------------------
plantPath = [modelName '/Plant'];
add_block('built-in/Subsystem', plantPath, 'Position', [500 260 660 420]);
build_plant_subsystem(plantPath);

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
fprintf('Model fixed step = %.4f s (control rate = %.2f s, %d sub-steps)\n', model_step, Ts, PLANT_SUBSTEPS);
fprintf('Run it with:  sim(''%s'');  then  plot_simulink_results();\n', modelName);

end

% =============================================================================
function build_plant_subsystem(plantPath)
%BUILD_PLANT_SUBSYSTEM Populate the Plant subsystem with the AUV physics:
%   Coriolis / Damping / Restoring / Kinematics as separate blocks, summed
%   and scaled by inv(M), then integrated. Integrator sample times are
%   left as "inherited" (-1), so they automatically tick at the model's
%   fine fixed step set in build_full_simulink_model.m, which is what
%   keeps this Forward-Euler-based integration numerically stable.

add_block('simulink/Sources/In1',  [plantPath '/tau'],   'Port', '1', 'Position', [30 40 60 60]);
add_block('simulink/Sources/In1',  [plantPath '/Delta'], 'Port', '2', 'Position', [30 200 60 220]);
add_block('simulink/Sinks/Out1',   [plantPath '/x'],     'Port', '1', 'Position', [820 40  850 60]);
add_block('simulink/Sinks/Out1',   [plantPath '/nudot'], 'Port', '2', 'Position', [820 300 850 320]);

add_ms_block(plantPath, 'Coriolis', 'CoriolisBlock', [140 20  260 90],  'Interpreted Execution');
add_ms_block(plantPath, 'Damping',  'DampingBlock',  [140 130 260 200], 'Interpreted Execution');
add_block('simulink/Sources/Constant', [plantPath '/Restoring_g'], ...
    'Value', '[0; 0; -0.981; 0]', 'Position', [140 240 260 280]);

add_block('simulink/Math Operations/Sum', [plantPath '/Sum_of_forces'], ...
    'Inputs', '+++++', 'Position', [310 30 340 290]);

add_block('simulink/Math Operations/Gain', [plantPath '/invM'], ...
    'Gain', 'diag([1/14.00, 1/29.90, 1/24.70, 1/0.52])', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [380 130 440 200]);

add_block('simulink/Discrete/Discrete-Time Integrator', [plantPath '/Integrator_nu'], ...
    'Position', [480 130 520 200]);   % SampleTime left as inherited (-1)

add_ms_block(plantPath, 'Kinematics', 'KinematicsBlock', [480 340 620 410], 'Interpreted Execution');

add_block('simulink/Discrete/Discrete-Time Integrator', [plantPath '/Integrator_eta'], ...
    'Position', [660 340 700 410]);   % SampleTime left as inherited (-1)

add_ms_block(plantPath, 'StateAssembly', 'StateAssemblyBlock', [700 30 830 100], 'Interpreted Execution');

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
