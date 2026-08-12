function [dx, y] = bldc_ab_ode(t, x, u, Ke, phi0, varargin)
Rs = 0.285;
Ls = 41.39e-6;

i_alpha   = x(1);  i_beta    = x(2);
V_alpha   = u(1);  V_beta    = u(2);
theta_enc = u(3);  omega_e   = u(4);

theta_e = theta_enc + phi0;

e_alpha = -sin(theta_e);
e_beta  =  cos(theta_e);

dx = zeros(2, 1);
dx(1) = (V_alpha - Rs * i_alpha - Ke * omega_e * e_alpha) / Ls;
dx(2) = (V_beta  - Rs * i_beta  - Ke * omega_e * e_beta)  / Ls;

y = [i_alpha; i_beta];
end
