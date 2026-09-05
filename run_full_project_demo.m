%% run_full_project_demo.m
%
% Runs the ENTIRE project end to end, in the order that tells the full
% story for a review: the paper-style multi-method comparison first,
% then both Simulink deployments (matching MATLAB, and matching the
% paper's block-decomposed diagram).
%
% RECOMMENDATION: run this BEFORE the review, not live in front of your
% professor - the full 90s runs with real MPC solves take real time.
% Have the figures already open and ready, then just narrate them.

fprintf('\n=================================================\n');
fprintf(' PART 1: Paper-style multi-method comparison (MATLAB)\n');
fprintf('=================================================\n');
run_simulation

fprintf('\n=================================================\n');
fprintf(' PART 2: Simulink - matches MATLAB exactly (single Plant block)\n');
fprintf('=================================================\n');
bdclose('auv_full_system')
if exist('auv_full_system.slx', 'file'); delete('auv_full_system.slx'); end
build_full_simulink_model
sim('auv_full_system')
plot_simulink_results

fprintf('\n=================================================\n');
fprintf(' PART 3: Simulink - block-decomposed physics (matches paper diagram)\n');
fprintf('=================================================\n');
bdclose('auv_full_system_decomposed')
if exist('auv_full_system_decomposed.slx', 'file'); delete('auv_full_system_decomposed.slx'); end
build_full_simulink_model_decomposed
sim('auv_full_system_decomposed')
plot_simulink_decomposed_results

fprintf('\n=================================================\n');
fprintf(' DONE. All figures and both .slx models are ready.\n');
fprintf('=================================================\n');
