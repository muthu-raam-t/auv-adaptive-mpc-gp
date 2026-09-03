function p = config()
%CONFIG Physical, control, and simulation parameters for the AUV project.
%   All tunable numbers live here so nothing is hard-coded elsewhere.

% --- Rigid-body / hydrodynamic parameters (BlueROV2-Heavy-like AUV) -----
p.m       = 11.4;    % mass [kg]
p.Izz     = 0.24;    % yaw moment of inertia [kg m^2]

p.Xu_dot  = -2.6;    p.Yv_dot  = -18.5;   p.Zw_dot  = -13.3;   p.Nr_dot = -0.28;
p.Xu      = -0.09;   p.Yv      = -0.26;   p.Zw      = -0.19;   p.Nr     = -4.64;
p.Xuc     = -34.9;   p.Yvc     = -103.25; p.Zwc     = -74.2;   p.Nrc    = -0.43;

p.g         = 9.81;
p.rho_water = 1000;
p.Vsub      = p.m / p.rho_water;  % chosen for near-neutral buoyancy

% --- Simulation ---------------------------------------------------------
p.Ts = 0.2;   % sample time [s]
p.Tf = 90;    % total simulation horizon [s]
p.Nc = 8;     % MPC prediction horizon [steps]

% --- Actuator limits [X, Y, Z, Mz] --------------------------------------
p.u_min = [-20; -20; -20; -5];
p.u_max = [ 20;  20;  20;  5];

% --- MPC cost weights: state = [x y z u v w psi r] ----------------------
p.Q  = diag([50 50 20 1 1 1 20 1]);
p.R  = diag([0.05 0.05 0.05 0.02]);
p.QT = 2 * p.Q;

end
