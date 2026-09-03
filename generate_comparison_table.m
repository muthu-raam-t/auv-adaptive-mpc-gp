function generate_comparison_table(results)
%GENERATE_COMPARISON_TABLE Prints a paper-vs-ours comparison table for the
%   rows that have a direct counterpart in this codebase.
%
%   IMPORTANT: the base paper's "GP-MPC [14]" and "Fast-AGP [19]" rows are
%   separate published baselines this project does NOT re-implement.
%   Only "No GP" and the dynamic-forgetting GP with plain LP weighting
%   have a direct equation-level match here. StaticGP and RDFGP_UAMPC are
%   this project's own additions, shown for context but not claimed to
%   equal any specific published row.
%
%   Usage:
%       load('results/simulation_results.mat');   % produces "results"
%       generate_comparison_table(results);
%   (run_simulation.m calls this automatically at the end.)

paper_NoGP_pred    = NaN;    paper_NoGP_track    = 0.1050;
paper_DFGP_pred    = 0.289;  paper_DFGP_track    = 0.0285;

fprintf('\n=== Paper vs. Ours: Comparison Table ===\n\n');
fprintf('%-14s %16s %10s %10s | %16s %10s %10s\n', ...
    'Method', 'PredErr(paper)', 'ours', 'delta%', 'TrackErr(paper)', 'ours', 'delta%');
fprintf('%s\n', repmat('-', 1, 100));

print_row('No GP',     paper_NoGP_pred, results.NoGP.dist_pred_error, ...
                       paper_NoGP_track, results.NoGP.rmse_pos);
print_row('DF-GP(LP)', paper_DFGP_pred, results.DFGP_LP.dist_pred_error, ...
                       paper_DFGP_track, results.DFGP_LP.rmse_pos);

fprintf('\nNote: RDFGP_UAMPC (this project''s proposed regularized +\n');
fprintf('uncertainty-aware variant) has no published row to compare against -\n');
fprintf('it is this project''s own addition. Its metrics:\n');
fprintf('    prediction RMSE = %.4f,  tracking RMSE = %.4f m\n', ...
    results.RDFGP_UAMPC.dist_pred_error, results.RDFGP_UAMPC.rmse_pos);

fprintf('\nStaticGP here is a single fixed-forgetting-factor GP-MPC baseline\n');
fprintf('for context; it is NOT the paper''s GP-MPC[14] or Fast-AGP[19]\n');
fprintf('(those are different published methods, not reproduced in this\n');
fprintf('codebase). Its metrics:\n');
fprintf('    prediction RMSE = %.4f,  tracking RMSE = %.4f m\n', ...
    results.StaticGP.dist_pred_error, results.StaticGP.rmse_pos);

end

function print_row(name, paperPred, oursPred, paperTrack, oursTrack)
if isnan(paperPred)
    predStr = '--';
    deltaPredStr = '--';
else
    predStr = sprintf('%.3f', paperPred);
    deltaPredStr = sprintf('%+.0f%%', 100*(oursPred - paperPred)/paperPred);
end
deltaTrackStr = sprintf('%+.0f%%', 100*(oursTrack - paperTrack)/paperTrack);
fprintf('%-14s %16s %10.3f %10s | %16.4f %10.4f %10s\n', ...
    name, predStr, oursPred, deltaPredStr, paperTrack, oursTrack, deltaTrackStr);
end
