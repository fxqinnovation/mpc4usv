function xdot = model(x,tau)

% xdot is the time derivative of the state vector: x = [x y psi u v r]'

% x = [x y psi u v r]'

% x     = position in x-direction (m)
% y     = position in y-direction (m)
% psi   = yaw angle               (rad)
% u     = surge velocity          (m/s)
% v     = sway velocity           (m/s)
% r     = yaw velocity            (rad/s)

% tau   = [X, Y, N]' control force/moment

psi = x(3); u = x(4); v = x(5); r = x(6);
Fx = tau(1);
Fy = tau(2);
Fn = tau(3); 

% Parameters, hydrodynamic derivatives and main dimensions
m = 3980; Iz = 19703;
Xu = -50; Xuu = -135; Xuuu = 0;
Yv = -200; Yvv = -2000; Yvvv = 0; Yr = 0;
Nr = -3224; Nrr = 0; Nrrr = -3224; Nv = 0;


xdot = [u * cos(psi)-v * sin(psi)
        u * sin(psi)+v * cos(psi)
        r
        (Fx + m * v * r + Xu * u + Xuu * abs(u) * u + Xuuu * u^3) / m
        (Fy - m * u * r + Yv * v + Yr * r + Yvv * abs(v) * v + Yvvv * v^3) / m
        (Fn + Nv * v + Nr * r + Nrr * abs(r) * r + Nrrr * r^3) / Iz];
end