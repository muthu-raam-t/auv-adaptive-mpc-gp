classdef ControllerBlockDecomposed < matlab.System
    % CONTROLLERBLOCKDECOMPOSED  Identical logic to ControllerBlock, used
    %   specifically with the block-decomposed Plant (Coriolis/Damping/
    %   Restoring/Kinematics as separate visible blocks, matching the
    %   base paper's diagram). That Plant uses Discrete-Time Integrator
    %   blocks (Euler integration), which need a much finer time step to
    %   stay numerically stable on this vehicle's stiff yaw axis - so
    %   this controller only actually re-solves the GP+MPC logic once
    %   every SUBSTEPS ticks and holds its output in between, keeping
    %   the real control rate at Ts even though the model itself ticks
    %   much faster.
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
        tickCounter
        u_hold
        d_hat_hold
        sig2_hat_hold
    end

    properties (Constant, Access = private)
        SUBSTEPS = 16;   % must match PLANT_SUBSTEPS in build_full_simulink_model_decomposed.m
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.p       = config();
            obj.lambdas = [1.0, 0.8, 0.6];
            obj.Nwin    = 20;
            obj.rho     = 0.05;

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

            obj.tickCounter   = 0;
            obj.u_hold        = zeros(4,1);
            obj.d_hat_hold    = zeros(4,1);
            obj.sig2_hat_hold = zeros(4,1);
        end

        function [u, d_hat, sig2_hat] = stepImpl(obj, t, x)
            if mod(obj.tickCounter, obj.SUBSTEPS) ~= 0
                u        = obj.u_hold;
                d_hat    = obj.d_hat_hold;
                sig2_hat = obj.sig2_hat_hold;
                obj.tickCounter = obj.tickCounter + 1;
                return;
            end

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

            obj.u_hold        = u;
            obj.d_hat_hold    = d_hat;
            obj.sig2_hat_hold = sig2_hat;
            obj.tickCounter   = obj.tickCounter + 1;
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
