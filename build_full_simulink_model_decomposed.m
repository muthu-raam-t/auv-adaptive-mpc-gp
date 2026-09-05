function build_full_simulink_model_decomposed()
%BUILD_FULL_SIMULINK_MODEL_DECOMPOSED Builds a SEPARATE Simulink model,
%   auv_full_system_decomposed.slx, alongside auv_full_system.slx (built
%   by build_full_simulink_model.m). This one shows the vehicle physics
%   as four separate, individually labeled blocks (Coriolis, Damping,
%   Restoring, Kinematics) matching the base paper's plant diagram,
%   instead of one combined block.
%
%   This model runs at a much finer internal time step than the control
%   decision rate, because the Discrete-Time Integrator blocks it uses
%   (Euler integration) need that to stay numerically stable on this
%   vehicle's stiff yaw axis. ControllerBlockDecomposed only actually
%   re-solves the GP+MPC logic once every 16 ticks and holds its output
%   in between, so the real control rate still matches Ts.
%
%   Because of the cruder (Euler vs. RK4) integration, this model's
%   numbers will not match auv_full_system.slx / run_simulation.m as
%   closely - that is the known, expected trade-off for having the
%   physics visible as separate blocks. Both models remain available;
%   running one does not affect or overwrite the other, since they use
%   different model names and different logged variable names.
%
%   Run it with:
%       sim('auv_full_system_decomposed');
%       plot_simulink_decomposed_results();

modelName = 'auv_full_system_decomposed';
p = config();
Ts = p.Ts;

PLANT_SUBSTEPS = 16;   % must match ControllerBlockDecomposed.SUBSTEPS
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
set_param(modelName, 'StopTime', num2str(p.Tf));
set_param(modelName, 'ReturnWorkspaceOutputs', 'off');

% =============================================================================
% TOP LEVEL
% =============================================================================

add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [40 260 70 280]);

add_ms_block(modelName, 'Reference',  'ReferenceBlock',  [160 40  300 90],  'Interpreted Execution');
add_ms_block(modelName, 'Disturbance','DisturbanceBlock',[160 160 300 210], 'Interpreted Execution');
add_ms_block(modelName, 'Controller', 'ControllerBlockDecomposed', [160 300 340 400], 'Interpreted Execution');

plantPath = [modelName '/Plant'];
add_block('built-in/Subsystem', plantPath, 'Position', [500 260 660 420]);
build_plant_subsystem(plantPath);

% ---- Loggers (suffixed _v2 so they never collide with auv_full_system's) --
add_to_workspace(modelName, 't_log_v2',     [820 30]);
add_to_workspace(modelName, 'x_log_v2',     [820 90]);
add_to_workspace(modelName, 'xref_log_v2',  [820 150]);
add_to_workspace(modelName, 'dtrue_log_v2', [820 210]);
add_to_workspace(modelName, 'dhat_log_v2',  [820 270]);
add_to_workspace(modelName, 'u_log_v2',     [820 330]);
add_to_workspace(modelName, 'nudot_log_v2', [820 390]);

% ---- Wiring -----------------------------------------------------------------
add_line(modelName, 'Clock/1', 'Reference/1');
add_line(modelName, 'Clock/1', 'Disturbance/1');
add_line(modelName, 'Clock/1', 'Controller/1');
add_line(modelName, 'Clock/1', 't_log_v2/1');

add_line(modelName, 'Controller/1',  'Plant/1');
add_line(modelName, 'Disturbance/1', 'Plant/2');

add_line(modelName, 'Plant/1', 'Controller/2');
add_line(modelName, 'Plant/1', 'x_log_v2/1');
add_line(modelName, 'Plant/2', 'nudot_log_v2/1');

add_line(modelName, 'Reference/1',   'xref_log_v2/1');
add_line(modelName, 'Disturbance/1', 'dtrue_log_v2/1');
add_line(modelName, 'Controller/2',  'dhat_log_v2/1');
add_line(modelName, 'Controller/1',  'u_log_v2/1');

Simulink.BlockDiagram.arrangeSystem(modelName);
Simulink.BlockDiagram.arrangeSystem(plantPath);

save_system(modelName);
fprintf('Saved %s.slx in %s\n', modelName, pwd);
fprintf('Run it with:  sim(''%s'');  then  plot_simulink_decomposed_results();\n', modelName);

end

% =============================================================================
function build_plant_subsystem(plantPath)
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
    'Position', [480 130 520 200]);

add_ms_block(plantPath, 'Kinematics', 'KinematicsBlock', [480 340 620 410], 'Interpreted Execution');

add_block('simulink/Discrete/Discrete-Time Integrator', [plantPath '/Integrator_eta'], ...
    'Position', [660 340 700 410]);

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
blk = add_block('simulink/User-Defined Functions/MATLAB System', ...
    [parentPath '/' name], 'Position', pos);
set_param(blk, 'System', className);
if nargin >= 5 && ~isempty(simMode)
    set_param(blk, 'SimulateUsing', simMode);
end
end

% =============================================================================
function blk = add_to_workspace(modelName, varName, posTopLeft)
pos = [posTopLeft(1), posTopLeft(2), posTopLeft(1)+90, posTopLeft(2)+30];
blk = add_block('simulink/Sinks/To Workspace', [modelName '/' varName], ...
    'VariableName', varName, 'SaveFormat', 'Array', ...
    'MaxDataPoints', 'inf', 'Position', pos);
end
