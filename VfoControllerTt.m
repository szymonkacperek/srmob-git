function output = VfoControllerTt(input)
%% Inputs
q_d = input(1:3);
q = input(4:6);
f_d_dot = input(7:8);
q_dot = input(9:11);
q_d_dotdot = input(12:13);

% Deploying inputs
theta = q(1); x = q(2); y = q(3);
theta_d = q_d(1); x_d = q_d(2); y_d = q_d(3);
x_d_dot = f_d_dot(1); y_d_dot = f_d_dot(2);
theta_dot = q_dot(1); x_dot = q_dot(2); y_dot = q_dot(3);
x_d_dotdot = q_d_dotdot(1); y_d_dotdot = q_d_dotdot(2);

e_theta = theta_d - theta;
e_x = x_d - x; e_y = y_d - y;
e = [e_x; e_y];
e_dot = [x_d_dot - x_dot; y_d_dot - y_dot];

%% Controller synthesis
zeta_d = 1;
k_p = 1.0;
K_p = diag([k_p k_p]);
eta = 0.8*k_p;
delta = 0.001;
k_a = 2*k_p;
sigma = zeta_d;

%% Controller form (u_2 calculations)
g_2 = [cos(theta); sin(theta)];
g_2d = [cos(theta_d); sin(theta_d)];

v_x = x_d_dot;
v_y = y_d_dot;
v = [v_x; v_y];
v_dot = [x_d_dotdot; y_d_dotdot];

% (6.11)
h_xy = K_p*e + v;
h_x = h_xy(1); h_y = h_xy(2);
h_x_dot = k_p*e_dot(1) + v_dot(1);
h_y_dot = k_p*e_dot(2) + v_dot(2);

% steering signal (u_2)
u_2 = h_xy'*g_2;

%% Controller form (u_1 calculations)
% conditional orientation (6.32)
epsilon = abs(norm(h_xy, 2) - (norm(h_xy, 2)/10));

global old_theta_a
if norm(h_xy, 2) > epsilon
    theta_a = Atan2c(sigma*h_y, sigma*h_x, old_theta_a);
    theta_a_dot = (h_y_dot*h_x - h_y*h_x_dot)/(h_x^2 + h_y^2);
else
    theta_a = theta_d;
    theta_a_dot = 0;
    u_2 = 0;
end
old_theta_a = theta_a;

% Steering signal (u_1)
u_1 = k_a*(theta_a - theta) + theta_a_dot;

%% Output
output = [u_1 u_2 e_theta e_x e_y];
