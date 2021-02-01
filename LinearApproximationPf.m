function output = LinearApproximationPf(input)
%% Inputs
u_d = input(1:2);
q = input(3:5);
q_d = input(6:8);
kappa_d = input(9);

% Deploying inputs
theta = q(1); x = q(2); y = q(3);
theta_d = q_d(1); x_d = q_d(2); y_d = q_d(3);
omega_d = u_d(1); 
v_d = u_d(2);

%% Controller synthesis
% v_d = 1.0;
zeta_d = 1;
omega_n = 3.0; 
k_1 = omega_n^2;
k_2 = 2*zeta_d*omega_n;

%% Controller form
% steering signal
u_2 = zeta_d * v_d * 0.1;

% u_1 steering signal calculations
e_theta = theta - theta_d;
j_S = [cos(theta_d + pi/2);
       sin(theta_d + pi/2)];
e_l = sign([x - x_d; y - y_d]'*j_S)*norm([x - x_d; y - y_d]);
u = -k_1*zeta_d*v_d*e_l - k_2*v_d*e_theta;

u_1 = u + u_2*kappa_d*cos(e_theta)/(1 - e_l*kappa_d);

%% Output
output = [u_1 u_2 e_theta e_l];

