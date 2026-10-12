%% make_white_graphs.m
%
% Opens EVERY figure used in the review - all on a white background - and
% saves each one as a PNG in slide_graphs/.
%
%   Usage (MATLAB, from the project folder):
%       make_white_graphs
%
% This is a SCRIPT (not a function) on purpose: the Simulink logs must land
% in the base workspace, same as in run_full_project_demo.m.
%
% First run is slow (it runs whatever has no saved result yet). Everything
% that gets computed is cached in results/, so later runs only redraw.

close all; clc;
addpath(genpath(pwd));
if ~exist('results',      'dir'), mkdir('results');      end
if ~exist('slide_graphs', 'dir'), mkdir('slide_graphs'); end

% Light graphics theme for this MATLAB session (R2025a+; ignored on older).
try
    s_ = settings;
    s_.matlab.appearance.figure.GraphicsTheme.TemporaryValue = 'light';
catch
end
clear s_

% ---- switches --------------------------------------------------------------
RUN_MC_IF_MISSING = false;  % true = run the slow Monte Carlo (40 sims) if no saved file
RUN_INDUSTRIAL    = false;  % true = also draw the 2 industrial-metrics figures (re-runs 2 fault sims)

p = config();

%% 1) Multi-method comparison (Trajectory, Tracking Error, Disturbance Estimation)
f_res = fullfile('results', 'simulation_results.mat');
if exist(f_res, 'file')
    S_ = load(f_res, 'results');  results = S_.results;  clear S_
else
    mlist = {'NoGP', 'StaticGP', 'DFGP_LP', 'RDFGP_UAMPC'};
    results = struct();
    for m_ = 1:numel(mlist)
        fprintf('--- Running method: %s ---\n', mlist{m_});
        results.(mlist{m_}) = simulate_method(mlist{m_}, p);
    end
    clear m_ mlist
end
if ~isfield(results, 'PID')
    fprintf('--- Running method: PID ---\n');
    results.PID = simulate_method_pid(p);
end
save(f_res, 'results');
plot_results(results, p);

%% 2) LP vs QP blend weights
demo_lp_vs_qp_blend;

%% 3) Fault tolerance (trajectory + tracking error before/after fault)
run_fault_tolerance_demo;

%% 4) Monte Carlo robustness (redrawn from saved results)
f_mc = fullfile('results', 'monte_carlo_results.mat');
if exist(f_mc, 'file')
    S_ = load(f_mc, 'results_mc');  plot_monte_carlo_results(S_.results_mc);  clear S_
elseif RUN_MC_IF_MISSING
    run_monte_carlo(10);
else
    warning('make_white_graphs:noMC', ...
        ['No results/monte_carlo_results.mat - Monte Carlo figure skipped. ' ...
         'Set RUN_MC_IF_MISSING = true to run it (slow).']);
end

%% 5) Simulink - original model
vars1 = {'t_log', 'x_log', 'xref_log', 'dtrue_log', 'dhat_log', 'u_log'};
f_sl1 = fullfile('results', 'simulink_logs.mat');
if exist(f_sl1, 'file')
    load(f_sl1);                                   % logs -> base workspace
else
    if ~exist('auv_full_system.slx', 'file'), build_full_simulink_model; end
    sim('auv_full_system');
    save(f_sl1, vars1{:});
end
plot_simulink_results;

%% 6) Simulink - block-decomposed model
vars2 = {'t_log_v2', 'x_log_v2', 'xref_log_v2', 'dtrue_log_v2', 'dhat_log_v2'};
f_sl2 = fullfile('results', 'simulink_logs_decomposed.mat');
if exist(f_sl2, 'file')
    load(f_sl2);
else
    if ~exist('auv_full_system_decomposed.slx', 'file'), build_full_simulink_model_decomposed; end
    sim('auv_full_system_decomposed');
    save(f_sl2, vars2{:});
end
plot_simulink_decomposed_results;

%% 7) Optional industrial-readiness figures
if RUN_INDUSTRIAL
    industrial_metrics_report;
end

%% 8) Force white on everything open, export PNGs, bring figures to front
figs_ = findall(0, 'Type', 'figure');
[~, ord_] = sort([figs_.Number]);
figs_ = figs_(ord_);
fprintf('\nExporting %d figures to slide_graphs/ ...\n', numel(figs_));
for f_ = figs_'
    force_white_figure(f_);
    nm_ = regexprep(regexprep(f_.Name, '[^A-Za-z0-9]+', '_'), '^_|_$', '');
    if isempty(nm_), nm_ = sprintf('figure_%d', f_.Number); end
    exportgraphics(f_, fullfile('slide_graphs', [nm_ '.png']), ...
        'Resolution', 200, 'BackgroundColor', 'white');
    fprintf('  slide_graphs/%s.png\n', nm_);
    figure(f_);
end

%% 9) Open the Simulink diagrams too
try, open_system('auv_full_system');            catch, end
try, open_system('auv_full_system_decomposed'); catch, end

fprintf('\nDONE. All figures are open (white) and saved in slide_graphs/.\n');
clear figs_ ord_ f_ nm_
