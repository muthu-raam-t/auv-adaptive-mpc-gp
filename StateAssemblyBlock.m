classdef StateAssemblyBlock < matlab.System
    % STATEASSEMBLYBLOCK  x = [eta(1:3); nu(1:3); eta(4); nu(4)]   (Eq. 10)
    %   Inputs  eta = [x; y; z; psi],  nu = [ub; vb; wb; r]
    %   Output  x (8x1) = [x y z ub vb wb psi r]'

    methods (Access = protected)
        function x = stepImpl(~, eta, nu)
            x = [eta(1); eta(2); eta(3); nu(1); nu(2); nu(3); eta(4); nu(4)];
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
