function generate_comparison_table(results)
%GENERATE_COMPARISON_TABLE Prints a comparison table of prediction and
%   tracking error across all four controller variants, alongside the
%   base paper's published numbers where a direct match exists.
%
%   Usage:
%       load('results/simulation_results.mat');   % produces "results"
%       generate_comparison_table(results);
%   (run_simulation.m calls this automatically at the end.)

paper_NoGP_pred    = NaN;    paper_NoGP_track    = 0.1050;
paper_DFGP_pred    = 0.289;  paper_DFGP_track    = 0.0285;

fprintf('\n=== Comparison Table ===\n\n');
fprintf('%-14s %16s %10s %10s | %16s %10s %10s\n', ...
    'Method', 'PredErr(paper)', 'ours', 'delta%', 'TrackErr(paper)', 'ours', 'delta%');
fprintf('%s\n', repmat('-', 1, 100));

print_row('No GP',     paper_NoGP_pred, results.NoGP.dist_pred_error, ...
                       paper_NoGP_track, results.NoGP.rmse_pos);
print_row('DF-GP(LP)', paper_DFGP_pred, results.DFGP_LP.dist_pred_error, ...
                       paper_DFGP_track, results.DFGP_LP.rmse_pos);

fprintf('\nRegularized dynamic-forgetting GP with uncertainty-aware MPC:\n');
fprintf('    prediction RMSE = %.4f,  tracking RMSE = %.4f m\n', ...
    results.RDFGP_UAMPC.dist_pred_error, results.RDFGP_UAMPC.rmse_pos);

fprintf('\nSingle fixed-forgetting-factor GP-MPC baseline:\n');
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
