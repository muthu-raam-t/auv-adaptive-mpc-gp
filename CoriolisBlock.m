classdef CoriolisBlock < matlab.System
    % CORIOLISBLOCK  Cnu = C(nu)*nu   (Eqs. 4,5,8 Coriolis/centripetal terms)
    %   Input  nu = [ub; vb; wb; r]  (4x1 body velocities)
    %   Output Cnu (4x1)

    methods (Access = protected)
        function Cnu = stepImpl(~, nu)
            m = 11.4; Xu_dot = -2.6; Yv_dot = -18.5;
            u = nu(1); v = nu(2); r = nu(4);

            Cnu = [ (m*v + Yv_dot*v)*r;
                   -(m*u + Xu_dot*u)*r;
                    0;
                   -(m*v - Yv_dot*v)*u - (Xu_dot*u - m*u)*v ];
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
