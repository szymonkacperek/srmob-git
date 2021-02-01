function output = LinearApproximationTt(input)
%% Inputs
u_d = input(1:2);
q = input(3:5);
q_d = input(6:8);
e = q - q_d;

% Deploying inputs
theta = q(1); x = q(2); y = q(3);
theta_d = q_d(1); x_d = q_d(2); y_d = q_d(3);

%% Controller synthesis
% u_d = [1.0 1.0];
zeta = 1.0;
alpha = 2.0;
k_22 = -2*zeta*sqrt(u_d(1)^2 + alpha*u_d(2)^2);
k_11 = k_22;
k_13 = -alpha*u_d(2);
k_12 = 0; k_21 = 0; k_23 = 0;

%% Controller form
e_1_tilde = theta_d - theta;
e_2_tilde = cos(theta)*(x_d - x) + sin(theta)*(y_d - y);
e_3_tilde = -sin(theta)*(x_d - x) + cos(theta)*(y_d - y);

u = [u_d(1) - k_11*e_1_tilde - k_12*e_2_tilde - k_13*e_3_tilde;
     u_d(2)*cos(e_1_tilde) - k_21*e_1_tilde - k_22*e_2_tilde - k_23*e_3_tilde];
 
%% Output 
output = [u(1) u(2) e(1) e(2) e(3)];

