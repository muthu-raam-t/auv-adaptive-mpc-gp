classdef PlantBlock < matlab.System
    % PLANTBLOCK  The AUV plant, integrated with the exact same
    %   auv_dynamics.m / rk4_integrate.m functions used by the
    %   MATLAB-only simulation - guaranteeing the Simulink plant and the
    %   MATLAB plant run identical physics, rather than two different
    %   integration schemes that can drift apart.
    %
    %   Inputs  tau (4x1 control), Delta (4x1 disturbance)
    %   Outputs x (8x1 full state), nudot (4x1 instantaneous body accel)

    properties (Access = private)
        p
        x
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.p = config();
            obj.x = zeros(8, 1);
        end

        function [x_out, nudot_out] = stepImpl(obj, tau, Delta)
            xdot_now  = auv_dynamics(obj.x, tau, Delta, obj.p);
            nudot_out = xdot_now([4 5 6 8]);

            obj.x = rk4_integrate(obj.x, tau, Delta, obj.p, obj.p.Ts);
            x_out = obj.x;
        end

        function resetImpl(obj)
            obj.setupImpl();
        end

        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [8 1]; sz2 = [4 1];
        end
        function [d1, d2] = getOutputDataTypeImpl(~)
            d1 = 'double'; d2 = 'double';
        end
        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false; c2 = false;
        end
        function [f1, f2] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true;
        end
    end
end
