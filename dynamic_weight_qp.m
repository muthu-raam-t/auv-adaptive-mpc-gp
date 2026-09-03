function eta = dynamic_weight_qp(weightedErrHist, rho)
%DYNAMIC_WEIGHT_QP Solve for the blending weights eta across K GP models.
%
%   weightedErrHist : Nwin x K matrix, each entry is an (already
%                      recency-weighted) squared prediction error for
%                      model j over the historical window.
%   rho             : regularization strength.
%                      rho = 0   -> reduces to the plain linear program
%                                   used in the base paper (picks a single
%                                   best model at each step).
%                      rho > 0   -> quadratic penalty on ||eta||^2, which
%                                   smooths the blend instead of hard
%                                   switching (our proposed RDF-GP).
%
%   Solves:  min_eta  sum_j eta_j * c_j  + rho * ||eta||^2
%            s.t.      sum(eta) = 1,  eta >= 0

K = size(weightedErrHist, 2);
c = sum(weightedErrHist, 1)';   % linear cost coefficient per model

H  = 2*rho*eye(K);
Aeq = ones(1, K); beq = 1;
lb  = zeros(K, 1); ub = ones(K, 1);

opts = optimoptions('quadprog', 'Display', 'off');
eta = quadprog(H, c, [], [], Aeq, beq, lb, ub, [], opts);

if isempty(eta)
    eta = ones(K, 1) / K;   % fallback if the solver fails
end

end
