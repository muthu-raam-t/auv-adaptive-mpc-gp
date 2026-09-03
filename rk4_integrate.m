function x_next = rk4_integrate(x, u, d, p, Ts)
%RK4_INTEGRATE One 4th-order Runge-Kutta step of auv_dynamics.

k1 = auv_dynamics(x,            u, d, p);
k2 = auv_dynamics(x + Ts/2*k1,  u, d, p);
k3 = auv_dynamics(x + Ts/2*k2,  u, d, p);
k4 = auv_dynamics(x + Ts*k3,    u, d, p);

x_next = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);

end
