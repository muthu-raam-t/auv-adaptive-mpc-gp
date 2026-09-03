function xref = reference_trajectory(t, p)
%REFERENCE_TRAJECTORY Figure-eight (lemniscate) reference at constant depth.
%   t can be a scalar or a row vector of time stamps; returns an
%   8 x length(t) matrix matching the state ordering used everywhere else:
%   [x y z u v w psi r].

a = 2;      % scale [m]
w = 0.05;   % angular rate [rad/s]

xr  = a*sin(w*t);
yr  = (a/2)*sin(2*w*t);
dxr = a*w*cos(w*t);
dyr = a*w*cos(2*w*t);
psi_r = atan2(dyr, dxr);

zr = -1.0*ones(size(t));

xref = [xr; yr; zr; zeros(size(t)); zeros(size(t)); zeros(size(t)); psi_r; zeros(size(t))];

end
