classdef DisturbanceBlock < matlab.System
    % DISTURBANCEBLOCK  Ground-truth environmental disturbance, hidden
    %   from the controller. Wraps disturbance_profile.m so the Simulink
    %   model and the MATLAB-only simulation use the identical scenario.
    %   Input  t (scalar simulation time)
    %   Output d_true (4x1)

    methods (Access = protected)
        function d = stepImpl(~, t)
            d = disturbance_profile(t);
        end

        function sz = getOutputSizeImpl(~)
            sz = [4 1];
        end
        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end
        function cplx = isOutputComplexImpl(~)
            cplx = false;
        end
        function fixed = isOutputFixedSizeImpl(~)
            fixed = true;
        end
    end
end
