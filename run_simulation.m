%% run_simulation.m
%
% MAIN ENTRY POINT - run this script.
%
% Compares five AUV trajectory-tracking controllers under an unknown,
% time-varying disturbance profile:
%
%   1) NoGP         - nominal nonlinear MPC, no disturbance estimation
%   2) StaticGP     - single fixed-forgetting-factor GP-MPC (baseline)
%   3) DFGP_LP      - multi-GP dynamic forgetting with LP weight blending
%   4) RDFGP_UAMPC  - regularized dynamic forgetting GP + uncertainty-aware
%                     MPC (proposed extension)
%   5) PID          - independent per-axis PID, no prediction or learning;
%                     the industry-standard baseline most real ROVs/AUVs
%                     actually run today, included as a reference point
%                     independent of the base paper's MPC lineage
%
% Requires: MATLAB Optimization Toolbox (fmincon, quadprog).

clear; clc; close all;
addpath(genpath(pwd));

p = config();

methods = {'NoGP', 'StaticGP', 'DFGP_LP', 'RDFGP_UAMPC'};
results = struct();

for m = 1:numel(methods)
    method = methods{m};
    fprintf('--- Running method: %s ---\n', method);
    results.(method) = simulate_method(method, p);
end

fprintf('--- Running method: PID ---\n');
results.PID = simulate_method_pid(p);

if ~exist('results_dir', 'var')
    results_dir = 'results';
end
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
save(fullfile(results_dir, 'simulation_results.mat'), 'results');

plot_results(results, p);
generate_comparison_table(results);
