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
fprintf('%-14s %18s %14s %10s | %18s %14s %10s\n', ...
    'Method', 'Base Paper (Pred)', 'Proposed', 'delta%', 'Base Paper (Track)', 'Proposed', 'delta%');
fprintf('%s\n', repmat('-', 1, 108));

print_row('No GP',     paper_NoGP_pred, results.NoGP.dist_pred_error, ...
                       paper_NoGP_track, results.NoGP.rmse_pos, 'N/A (no disturbance estimation)');
print_row('DF-GP(LP)', paper_DFGP_pred, results.DFGP_LP.dist_pred_error, ...
                       paper_DFGP_track, results.DFGP_LP.rmse_pos, '');

fprintf('\nRegularized dynamic-forgetting GP with uncertainty-aware MPC:\n');
fprintf('    prediction RMSE = %.4f,  tracking RMSE = %.4f m\n', ...
    results.RDFGP_UAMPC.dist_pred_error, results.RDFGP_UAMPC.rmse_pos);

fprintf('\nSingle fixed-forgetting-factor GP-MPC baseline:\n');
fprintf('    prediction RMSE = %.4f,  tracking RMSE = %.4f m\n', ...
    results.StaticGP.dist_pred_error, results.StaticGP.rmse_pos);

end

function print_row(name, paperPred, oursPred, paperTrack, oursTrack, naNote)
if isnan(paperPred)
    predStr = '--';
else
    predStr = sprintf('%.3f', paperPred);
end
if isnan(oursPred)
    oursPredStr = naNote;
    deltaPredStr = '--';
else
    oursPredStr = sprintf('%.3f', oursPred);
    if isnan(paperPred)
        deltaPredStr = '--';
    else
        deltaPredStr = sprintf('%+.0f%%', 100*(oursPred - paperPred)/paperPred);
    end
end
deltaTrackStr = sprintf('%+.0f%%', 100*(oursTrack - paperTrack)/paperTrack);
fprintf('%-14s %18s %14s %10s | %18.4f %14.4f %10s\n', ...
    name, predStr, oursPredStr, deltaPredStr, paperTrack, oursTrack, deltaTrackStr);
end
