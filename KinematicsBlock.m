classdef KinematicsBlock < matlab.System
    % KINEMATICSBLOCK  etadot = J(psi)*nu   (Eqs. 1,2,3,7)
    %   Inputs  eta = [x; y; z; psi],  nu = [ub; vb; wb; r]
    %   Output  etadot (4x1)

    methods (Access = protected)
        function etadot = stepImpl(~, eta, nu)
            psi = eta(4);
            u = nu(1); v = nu(2); w = nu(3); r = nu(4);

            etadot = [ cos(psi)*u - sin(psi)*v;
                       sin(psi)*u + cos(psi)*v;
                       w;
                       r ];
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
