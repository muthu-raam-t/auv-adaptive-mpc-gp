function xref = reference_trajectory(t, p)
%REFERENCE_TRAJECTORY Rotated figure-eight (lemniscate) reference at
%   constant depth, matching the tilted lemniscate used in the base
%   paper's Fig. 5. t can be a scalar or a row vector of time stamps;
%   returns an 8 x length(t) matrix matching the state ordering used
%   everywhere else: [x y z u v w psi r].

a        = 3;                 % scale [m] - lobe half-span before rotation
n_cycles = 1;                  % number of full figure-eight laps over Tf
w        = n_cycles*2*pi/p.Tf; % angular rate [rad/s], tied to config().Tf
theta    = deg2rad(-25);       % rotation of the whole figure-eight [rad]

x0  = a*sin(w*t);
y0  = (a/2)*sin(2*w*t);
dx0 = a*w*cos(w*t);
dy0 = a*w*cos(2*w*t);

ct = cos(theta); st = sin(theta);
xr  = ct*x0  - st*y0;
yr  = st*x0  + ct*y0;
dxr = ct*dx0 - st*dy0;
dyr = st*dx0 + ct*dy0;

psi_r = atan2(dyr, dxr);
zr = -1.0*ones(size(t));

xref = [xr; yr; zr; zeros(size(t)); zeros(size(t)); zeros(size(t)); psi_r; zeros(size(t))];

end
