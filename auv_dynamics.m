function xdot = auv_dynamics(x, u, d, p)
%AUV_DYNAMICS Continuous-time nonlinear AUV model (surge/sway/heave/yaw).
%   x = [x, y, z, ub, vb, wb, psi, r]'   (position + body velocities + heading + yaw rate)
%   u = [X, Y, Z, Mz]'                   (actuator forces/moment)
%   d = [dx, dy, dz, dMz]'               (lumped external disturbance)
%   p = struct from config()

ub  = x(4); vb = x(5); wb = x(6); psi = x(7); r = x(8);
X = u(1); Y = u(2); Z = u(3); Mz = u(4);
dx = d(1); dy = d(2); dz = d(3); dMz = d(4);

xdot = zeros(8,1);

xdot(1) = cos(psi)*ub - sin(psi)*vb;
xdot(2) = sin(psi)*ub + cos(psi)*vb;
xdot(3) = wb;

xdot(4) = (X + (p.m*vb + p.Yv_dot*vb)*r + (p.Xu + p.Xuc*abs(ub))*ub + dx) ...
          / (p.m - p.Xu_dot);

xdot(5) = (Y - (p.m*ub + p.Xu_dot*ub)*r + (p.Yv + p.Yvc*abs(vb))*vb + dy) ...
          / (p.m - p.Yv_dot);

xdot(6) = (Z + (p.Zw + p.Zwc*abs(wb))*wb + (p.m - p.Vsub*p.rho_water)*p.g + dz) ...
          / (p.m - p.Zw_dot);

xdot(7) = r;

xdot(8) = (Mz - (p.m*vb - p.Yv_dot*vb)*ub - (p.Xu_dot*ub - p.m*ub)*vb ...
           + (p.Nr + p.Nrc*abs(r))*r + dMz) / (p.Izz - p.Nr_dot);

end
