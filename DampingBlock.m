classdef DampingBlock < matlab.System
    % DAMPINGBLOCK  Dnu = D(nu)*nu   (linear + quadratic drag, Table I)
    %   Input  nu = [ub; vb; wb; r]
    %   Output Dnu (4x1)

    methods (Access = protected)
        function Dnu = stepImpl(~, nu)
            Xu = -0.09; Xuc = -34.9;
            Yv = -0.26; Yvc = -103.25;
            Zw = -0.19; Zwc = -74.2;
            Nr = -4.64; Nrc = -0.43;

            u = nu(1); v = nu(2); w = nu(3); r = nu(4);

            Dnu = [ (Xu + Xuc*abs(u))*u;
                    (Yv + Yvc*abs(v))*v;
                    (Zw + Zwc*abs(w))*w;
                    (Nr + Nrc*abs(r))*r ];
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
