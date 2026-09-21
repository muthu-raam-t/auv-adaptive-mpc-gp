function Delta = real_disturbance_profile(t, p)
%REAL_DISTURBANCE_PROFILE Disturbance built from real NorKyst800/NORA3
%   recorded ocean current data (Norwegian coastal waters, March 2017),
%   time-compressed to fit the simulation window [0, p.Tf].
%
%   Loads data/norkyst_current.csv once (persistent, so it's only read
%   from disk on the first call), converts speed+direction into an
%   x/y force pair, and interpolates smoothly for any query time t
%   (scalar or vector), matching the same output shape as
%   disturbance_profile.m so it's a drop-in replacement.

persistent t_real Fx_real Fy_real

if isempty(t_real)
    T = readtable('data/norkyst_current.csv');
    n = height(T);

    % Elapsed real time in hours -> compressed linearly onto [0, Tf]
    hours_elapsed = (0:n-1)';
    t_real = hours_elapsed / hours_elapsed(end) * p.Tf;

    % Convert (speed, compass direction) into an x/y force pair.
    % Compass direction: 0 = North, 90 = East (clockwise), so:
    %   Fx (east)  = speed * sin(direction)
    %   Fy (north) = speed * cos(direction)
    speed = T.current_speed;
    dir_rad = deg2rad(T.current_direction);
    scale = 15;   % [N per m/s] - scales real current speed into a force
                  % magnitude comparable to the synthetic profile's peak
    Fx_real = scale * speed .* sin(dir_rad);
    Fy_real = scale * speed .* cos(dir_rad);
end

Fx = interp1(t_real, Fx_real, t, 'pchip', 'extrap');
Fy = interp1(t_real, Fy_real, t, 'pchip', 'extrap');

Delta = [Fx; Fy; zeros(size(t)); zeros(size(t))];

end
