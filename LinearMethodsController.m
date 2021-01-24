function output = LinearMethodsController(input)
%% Inputs
q_dZ = input(1:3);
q_Z = input(4:6);

%% Constants
% distance from P point [m]
L_Z = 0.2; 

% angle between L_Z and x^L [deg]
beta_Z = 0.3;

%% Movement of Z = (z_x z_y) point 
% Z coordinates
x_Z = x +

K = [-L_Z*sin(theta + beta_Z)
    L_Z*cos(theta + beta_Z)];



w = q_dZ_dot + K*e_z;

%% Output
output = [omega_d u_d];

