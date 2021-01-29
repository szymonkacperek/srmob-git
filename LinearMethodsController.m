function output = LinearMethodsController(input)
%% Inputs
q_dZ = input(1:2);
q = input(3:5);
q_dZ_dot = input(6:7);

q_Z = [q(2) q(3)];
theta = q(1);

%% Constants
global L_Z beta_Z
% distance from P point [m]
L_Z = 0.5; 

% angle between L_Z and x^L [deg]
beta_Z = 0.03;

x = q_Z(1);
y = q_Z(2);

%% Movement of Z = (z_x z_y) point 
% Z coordinates
x_Z = x + L_Z*cos(theta + beta_Z);
y_Z = y + L_Z*sin(theta + beta_Z);
q_Z = [x_Z; y_Z];

e_Z = q_dZ - q_Z;

D = [-L_Z*sin(theta + beta_Z) cos(theta);
      L_Z*cos(theta + beta_Z) sin(theta)];
inv_D = inv(D);
  
K = diag([2.0; 1.0]);
w = q_dZ_dot + K*e_Z;
u = inv_D*w;

%% Output
omega_d = u(1);
v_d = u(2);
output = [omega_d v_d e_Z(1) e_Z(2)];

