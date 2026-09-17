function [theta_grid, t_grid, dtheta_grid] = build_curvature_profile(a, w_base)
%BUILD_CURVATURE_PROFILE Computes a curvature-adaptive time-to-phase
%   mapping for one lap of the Gerono lemniscate x = a*sin(theta),
%   y = (a/2)*sin(2*theta). Progression is slowed near high-curvature
%   points (the lobe tips, where the path has to turn sharpest) and sped
%   up elsewhere, with the TOTAL lap time kept equal to what a constant
%   rate w_base would give (2*pi/w_base) - so overall mission timing
%   (and everything calibrated against it, like the disturbance regime
%   schedule) is unaffected. This is what stops the controller's limited
%   lookahead from being caught out at the sharpest turns.

N = 500;
theta_grid = linspace(0, 2*pi, N);

dx  = a*cos(theta_grid);
dy  = a*cos(2*theta_grid);
ddx = -a*sin(theta_grid);
ddy = -2*a*sin(2*theta_grid);

speed_mag = sqrt(dx.^2 + dy.^2);
curvature = abs(dx.*ddy - dy.*ddx) ./ max(speed_mag.^3, 1e-6);

k_curve    = 0.15;   % how aggressively to slow near sharp turns
min_factor = 0.35;   % never slow to less than this fraction of nominal speed

speed_factor = 1 ./ (1 + k_curve*curvature);
speed_factor = max(speed_factor, min_factor);

theta_dot = w_base * speed_factor;   % local rate of phase progression

dtheta   = gradient(theta_grid);
dt_local = dtheta ./ theta_dot;
t_raw    = cumtrapz(dt_local);

% Rescale so the total lap time matches the constant-rate lap time
% exactly, then rescale the local rate to match.
scale_factor = (2*pi/w_base) / t_raw(end);
t_grid      = t_raw * scale_factor;
dtheta_grid = theta_dot / scale_factor;

end
