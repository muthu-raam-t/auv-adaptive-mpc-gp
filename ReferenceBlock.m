classdef ReferenceBlock < matlab.System
    % REFERENCEBLOCK  Reference trajectory point at the current time,
    %   used for logging/plotting. The controller computes its own
    %   internal horizon separately for the MPC cost - this block exists
    %   only so the reference is visible/loggable at the top level.
    %   Input  t (scalar simulation time)
    %   Output xref (8x1)

    methods (Access = protected)
        function xref = stepImpl(~, t)
            p = config();
            xref = reference_trajectory(t, p);
        end

        function sz = getOutputSizeImpl(~)
            sz = [8 1];
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
