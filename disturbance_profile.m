function d = disturbance_profile(t)
%DISTURBANCE_PROFILE True (hidden-from-controller) environmental disturbance.
%   Three regimes over time, mirroring the kind of scenario used to stress
%   a forgetting-based estimator: a single sine, then a combined sine wave,
%   then an abrupt square wave.

if t < 30
    base = 1.0*sin(0.3*t);
elseif t < 60
    base = 0.7*sin(0.4*t) + 0.4*sin(0.9*t);
else
    base = 0.9*sign(sin(0.25*t));
end

d = [ base;
      0.6*base*cos(0.1*t);
      0.15*sin(0.2*t);
      0.05*base ];

end
