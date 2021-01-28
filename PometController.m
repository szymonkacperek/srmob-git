function output = PometController(input)
%% Inputs
q_d = input(1:3);
q = input(4:6);
t = input(7);

theta_d = q_d(1);   x_d = q_d(2);   y_d = q_d(3);
theta = q(1);       x = q(2);       y = q(3);

%% Tracking error
e = q_d - q;
e_theta = e(1); e_x = e(2); e_y = e(3);

e_theta_tilde = theta - theta_d;
e_x_tilde = cos(theta_d)*(x - x_d) + sin(theta_d)*(y - y_d);
e_y_tilde = -sin(theta_d)*(x - x_d) + cos(theta_d)*(y - y_d);

%% Controller synthesis
k_1 = 0.1;
k_2 = 0.2;
k_3 = 0.2;
k_4 = 0.4;
delta_p = 0.001;
Omega = 1;

%% Parametrical synthesis
% (5.35)
h = k_4*(norm([e_x_tilde; e_y_tilde], 2))^2*cos(Omega*t);
dhdt = -Omega*k_4*(norm([e_x_tilde; e_y_tilde], 2))^2*sin(Omega*t);


dVde_theta_tilde = k_1*(e_theta_tilde+h);
dVde_x_tilde = e_x_tilde*(k_2 + 2*k_4*cos(Omega*t)*k_1*(e_theta_tilde+h));
dVde_y_tilde = e_y_tilde*(k_3 + 2*k_4*cos(Omega*t)*k_1*(e_theta_tilde+h));
 
L_g1_V = dVde_theta_tilde;
L_g2_V = dVde_x_tilde*cos(e_theta_tilde) + dVde_y_tilde*sin(e_theta_tilde);

% (142), (143)
u_1 = -dhdt - L_g1_V;
u_2 = -L_g2_V;

%% Output
output = [u_1 u_2 e_theta e_x e_y];
 