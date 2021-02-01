function output = VfoControllerPf(input)
%% Inputs
f_d = input(1);
q_d_dot = input(2:3);
q_d_dotdot = input(3:4);
q = input(5:7); 
f_xyd_dot = input(8);

% deploying inputs
f_xd_dot = q_d_dot(1); f_yd_dot = q_d_dot(2);
f_xd_dotdot = q_d_dotdot(1); f_yd_dotdot = q_d_dotdot(2);
theta = q(1); x = q(2); y = q(3);

%% Controller synthesis
k_p = 1.0;
K_p = diag([k_p k_p]);
k_a = 2*k_p;
zeta_d = 1;

%% Controller form (u_2 calculations)
g_2 = [cos(theta); sin(theta)];
% g_2d = [cos(theta_d); sin(theta_d)];

% constant speed (4)
v_d = zeta_d*[1.0 1.0];

% (10) - (16)
nabla_f_dot = [f_xd_dot; f_yd_dot];
norm_nabla_f_dot = norm(nabla_f_dot, 2);
R = [0 1; -1 0];
vartheta = (1/norm_nabla_f_dot)*(-nabla_f_dot);
h_xy = (v_d*R + k_p*f_d*ones(2, 2))*vartheta;

% steering signal (u_2)
u_2 = h_xy'*g_2;

%% Controller form (u_1 calculations)
h_x = h_xy(1); h_y = h_xy(2);
f_dot = (f_xd_dot*cos(theta) + f_yd_dot*sin(theta))*h_xy'*g_2;
vartheta_dot = ((h_xy'*g_2)/norm_nabla_f_dot^3)*[f_yd_dot*((f_xd_dot*f_xyd_dot - f_yd_dot*f_xd_dotdot)*cos(theta) + (f_xd_dot*f_yd_dotdot - f_yd_dot*f_xyd_dot)*sin(theta));
                                                 f_xd_dot*((f_yd_dot*f_xd_dotdot - f_xd_dot*f_xyd_dot)*cos(theta) + (f_yd_dot*f_xyd_dot - f_xd_dot*f_yd_dotdot)*sin(theta))];
h_xy_dot = v_d*R*vartheta_dot + k_p*(f_dot*vartheta + f_d*vartheta_dot);
h_x_dot = h_xy_dot(1); h_y_dot = h_xy_dot(2);

global old_theta_a
theta_a = Atan2c(zeta_d*h_y, zeta_d*h_x, old_theta_a);
theta_a_dot = (h_y_dot*h_x - h_y*h_x_dot)/(h_x^2 + h_y^2);
old_theta_a = theta_a;

% Steering signal (u_1)
u_1 = k_a*(theta_a - theta) + theta_a_dot;

%% Output
output = [u_1 u_2];

