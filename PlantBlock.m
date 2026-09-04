classdef PlantBlock < matlab.System
    % PLANTBLOCK  The AUV plant, integrated with the exact same
    %   auv_dynamics.m / rk4_integrate.m functions used by the
    %   MATLAB-only simulation - guaranteeing the Simulink plant and the
    %   MATLAB plant run identical physics.
    %
    %   Both outputs (x, nudot) depend ONLY on internally stored state,
    %   never on the current-step tau/Delta inputs. This block has zero
    %   direct feedthrough on either input, which is required to break
    %   the Controller <-> Plant algebraic loop (Controller needs
    %   Plant's x to compute u; if Plant's x depended on Controller's u
    %   in the same step, there would be no way to solve the loop).
    %
    %   Inputs  tau (4x1 control), Delta (4x1 disturbance)
    %   Outputs x (8x1 full state), nudot (4x1 instantaneous body accel)

    properties (Access = private)
        p
        x
        nudot
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.p     = config();
            obj.x     = zeros(8, 1);
            obj.nudot = zeros(4, 1);
        end

        function [x_out, nudot_out] = stepImpl(obj, tau, Delta)
            % Outputs = state left over from the PREVIOUS call only.
            % No dependency on the current tau/Delta whatsoever.
            x_out     = obj.x;
            nudot_out = obj.nudot;

            % Now update the internal state using the current inputs,
            % for use on the NEXT call.
            xdot_now  = auv_dynamics(obj.x, tau, Delta, obj.p);
            obj.nudot = xdot_now([4 5 6 8]);
            obj.x     = rk4_integrate(obj.x, tau, Delta, obj.p, obj.p.Ts);
        end

        function resetImpl(obj)
            obj.setupImpl();
        end

        function [flag1, flag2] = isInputDirectFeedthroughImpl(~, ~, ~)
            % Explicitly tell Simulink neither input has direct
            % feedthrough to any output - required to break the
            % Controller <-> Plant algebraic loop.
            flag1 = false;
            flag2 = false;
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
