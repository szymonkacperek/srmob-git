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
k_1 = 1;
k_2 = 1;
k_3 = 1;
k_4 = 1;
delta_p = 0.01;
Omega = 1;

%% Parametrical synthesis
norm_e_tilde_v1 = (norm([e_x_tilde; e_y_tilde], 2))^2;
norm_e_tilde_v2 = (norm_e_tilde_v1)/(norm_e_tilde_v1 + delta_p);

% h (5.35)
h_1 = k_4*norm_e_tilde_v1*cos(Omega*t);
dhdt_1 = -Omega*k_4*norm_e_tilde_v1*sin(Omega*t);

% h (5.36)
h_2 = k_4*cos(Omega*t)*norm_e_tilde_v2;
dhdt_2 = -Omega*k_4*norm_e_tilde_v2*sin(Omega*t);

% Choose 'h' function form (5.35) or (5.36)
chosen_h = 2;
switch chosen_h
    case 1
        h = h_1; dhdt = dhdt_1;
    case 2
        h = h_2; dhdt = dhdt_2;
end

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
 