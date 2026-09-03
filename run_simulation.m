%% run_simulation.m
%
% MAIN ENTRY POINT - run this script.
%
% Compares four AUV trajectory-tracking controllers under an unknown,
% time-varying disturbance profile:
%
%   1) NoGP         - nominal nonlinear MPC, no disturbance estimation
%   2) StaticGP     - single fixed-forgetting-factor GP-MPC (baseline)
%   3) DFGP_LP      - multi-GP dynamic forgetting with LP weight blending
%   4) RDFGP_UAMPC  - regularized dynamic forgetting GP + uncertainty-aware
%                     MPC (proposed extension)
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

if ~exist('results_dir', 'var')
    results_dir = 'results';
end
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
save(fullfile(results_dir, 'simulation_results.mat'), 'results');

plot_results(results, p);
