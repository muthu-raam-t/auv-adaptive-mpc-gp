classdef ControllerBlock < matlab.System
    % CONTROLLERBLOCK  Proposed controller: a bank of forgetting-factor
    %   GPs (ForgettingGP.m) blended by a regularised weight optimisation
    %   (dynamic_weight_qp.m), feeding a disturbance estimate + variance
    %   into an uncertainty-aware nonlinear MPC (nmpc_solve.m). Reuses the
    %   exact same functions as the MATLAB-only simulation so this block
    %   and simulate_method.m behave identically.
    %
    %   Inputs  t (scalar time), x (8x1 measured state)
    %   Outputs u (4x1 control), d_hat (4x1 disturbance estimate),
    %           sig2_hat (4x1 disturbance variance estimate)

    properties (Access = private)
        p
        gpBank
        lastMus
        errBuf
        x_prev
        u_prev
        u_guess
        lambdas
        Nwin
        rho
        hasHistory
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.p       = config();
            obj.lambdas = [1.0, 0.8, 0.6];
            obj.Nwin    = 20;
            obj.rho     = 0.05;   % regularisation strength (RDF-GP novelty)

            nd = 4; dimA = 4; H = 25;
            obj.gpBank  = cell(nd,1);
            obj.lastMus = cell(nd,1);
            obj.errBuf  = cell(nd,1);
            for i = 1:nd
                obj.gpBank{i}  = ForgettingGP(obj.lambdas, H, dimA);
                obj.lastMus{i} = zeros(numel(obj.lambdas), 1);
                obj.errBuf{i}  = [];
            end

            obj.x_prev     = zeros(8,1);
            obj.u_prev     = zeros(4,1);
            obj.u_guess    = zeros(obj.p.Nc, 4);
            obj.hasHistory = false;
        end

        function [u, d_hat, sig2_hat] = stepImpl(obj, t, x)
            nd = 4;
            d_hat    = zeros(nd,1);
            sig2_hat = zeros(nd,1);

            if obj.hasHistory
                d_meas = residual_disturbance(obj.x_prev, x, obj.u_prev, obj.p, obj.p.Ts);
                featA  = obj.x_prev([4 5 6 8])';
                for i = 1:nd
                    obj.gpBank{i}.addPoint(featA, d_meas(i));
                    errRow = (obj.lastMus{i}' - d_meas(i)).^2;
                    obj.errBuf{i} = [obj.errBuf{i}; errRow];
                    if size(obj.errBuf{i}, 1) > obj.Nwin
                        obj.errBuf{i} = obj.errBuf{i}(end-obj.Nwin+1:end, :);
                    end
                end
            end

            featStar = x([4 5 6 8])';
            for i = 1:nd
                [mus, sig2s] = obj.gpBank{i}.predictAll(featStar);
                Kn = numel(obj.lambdas);
                if size(obj.errBuf{i}, 1) >= 3
                    Nrows = size(obj.errBuf{i}, 1);
                    alpha = exp(0.05*((1:Nrows) - 1))';
                    weightedErr = obj.errBuf{i} .* alpha;
                    eta = dynamic_weight_qp(weightedErr, obj.rho);
                else
                    eta = zeros(Kn, 1); eta(1) = 1;
                end
                d_hat(i)    = eta' * mus;
                sig2_hat(i) = eta' * sig2s;
                obj.lastMus{i} = mus;
            end

            t_horizon = t + (1:obj.p.Nc)*obj.p.Ts;
            xref_seq  = reference_trajectory(t_horizon, obj.p);
            [u, ~] = nmpc_solve(x, xref_seq, d_hat, sig2_hat, obj.p, obj.u_guess);
            obj.u_guess = [obj.u_guess(2:end, :); u'];

            obj.x_prev     = x;
            obj.u_prev     = u;
            obj.hasHistory = true;
        end

        function resetImpl(obj)
            obj.setupImpl();
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(~)
            sz1 = [4 1]; sz2 = [4 1]; sz3 = [4 1];
        end
        function [d1, d2, d3] = getOutputDataTypeImpl(~)
            d1 = 'double'; d2 = 'double'; d3 = 'double';
        end
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end
        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end
    end
end
