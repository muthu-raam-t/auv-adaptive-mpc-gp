classdef ForgettingGP < handle
    %FORGETTINGGP A bank of K weighted Gaussian Process regressors that
    %   share the same training buffer but apply different exponential
    %   forgetting factors (lambda) when weighting past samples. Each
    %   forgetting factor yields its own mean/variance prediction; the
    %   caller is responsible for combining them (see
    %   dynamic_weight_qp.m).

    properties
        lambda   % 1 x K vector of forgetting factors
        K        % number of forgetting factors
        H        % max buffer size
        dim      % input feature dimension
        sf2      % kernel signal variance
        ell      % kernel length scale
        sn2      % observation noise variance
        A        % H x dim input buffer
        Y        % H x 1 target buffer
        n        % current number of stored points
    end

    methods
        function obj = ForgettingGP(lambdas, H, dim)
            obj.lambda = lambdas(:)';
            obj.K      = numel(lambdas);
            obj.H      = H;
            obj.dim    = dim;
            obj.sf2    = 1.0;
            obj.ell    = ones(dim,1);
            obj.sn2    = 1e-3;
            obj.A      = zeros(H, dim);
            obj.Y      = zeros(H, 1);
            obj.n      = 0;
        end

        function addPoint(obj, a, y)
            a = a(:)';
            if obj.n < obj.H
                obj.n = obj.n + 1;
                obj.A(obj.n, :) = a;
                obj.Y(obj.n)    = y;
            else
                obj.A = [obj.A(2:end, :); a];
                obj.Y = [obj.Y(2:end); y];
            end
        end

        function fitHyperparameters(obj)
            % Cheap, data-driven hyperparameter update (median-distance
            % length scale + empirical variance) so we avoid an expensive
            % offline log-likelihood optimization at every call.
            if obj.n < 4
                return;
            end
            Aused = obj.A(1:obj.n, :);
            Yused = obj.Y(1:obj.n);

            obj.sf2 = max(var(Yused), 1e-3);

            dists = [];
            for i = 1:obj.n-1
                for j = i+1:obj.n
                    dists(end+1) = norm(Aused(i,:) - Aused(j,:)); %#ok<AGROW>
                end
            end
            if isempty(dists) || median(dists) < 1e-6
                obj.ell = ones(obj.dim, 1);
            else
                obj.ell = median(dists) * ones(obj.dim, 1);
            end
            obj.sn2 = max(0.05*obj.sf2, 1e-4);
        end

        function Kmat = kernel(obj, A1, A2)
            ellv = obj.ell(:)';
            n1 = size(A1,1);
            Kmat = zeros(n1, size(A2,1));
            for i = 1:n1
                diffm    = (A2 - A1(i,:)) ./ ellv;
                Kmat(i,:) = obj.sf2 * exp(-0.5*sum(diffm.^2, 2))';
            end
        end

        function [mu, sigma2] = predictSingle(obj, astar, kIdx)
            if obj.n < 2
                mu = 0; sigma2 = obj.sf2;
                return;
            end
            Aused = obj.A(1:obj.n, :);
            Yused = obj.Y(1:obj.n);

            lam = obj.lambda(kIdx);
            w = lam .^ ((obj.n-1):-1:0)';   % most recent sample gets weight 1
            w = max(w, 1e-6);

            Kaa   = obj.kernel(Aused, Aused);
            kstar = obj.kernel(astar, Aused);
            kss   = obj.kernel(astar, astar);

            Gmat = Kaa + obj.sn2*diag(1./w);   % weighted GP regression:
                                                % older points behave as if
                                                % noisier -> "forgotten"
            mu = kstar * (Gmat \ Yused);
            sigma2 = kss - kstar*(Gmat \ kstar');
            sigma2 = max(sigma2, 1e-6);
        end

        function [mus, sig2s] = predictAll(obj, astar)
            mus   = zeros(obj.K, 1);
            sig2s = zeros(obj.K, 1);
            for k = 1:obj.K
                [mus(k), sig2s(k)] = obj.predictSingle(astar, k);
            end
        end
    end
end
