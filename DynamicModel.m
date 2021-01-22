%% LABORATORIUM SRMob
% ĆWICZENIE 1 - Model matematyczny robota mobilnego

% @brief: Dynamic model file
% @notes:

function u_dot = DynamicModel(input)
%% Inputs 
u = input(1:2);
T = input(3:4);

omega = u(1);
v = u(2);

%% Constants
global D_aomega D_av C_r v_factor r b I_c m g I_k ksi J I_m n_g ksi_m
% Velocities of each wheel (2.15)
Omega = inv(J)*u;
omega_P = Omega(1);
omega_L = Omega(2);

%% Mathematical model (2.37) 
% Dynamics 
% M = [I_c 0; 0 m];
% V = zeros(2);
% B = [b/r -b/r; 1/r 1/r];

% Output (2.37)
% u_dot = inv(M)*(B*T - V*u);

%% Model (2.47)
% @brief: With resistance factors of air and wheel rolling.

% Resistance (2.41), (2.46)
F_a = [D_aomega*abs(omega)*omega; D_av*abs(v)*v];
F_r = [(b/2)*m*g*C_r*(tanh(v_factor*r*omega_P)-tanh(v_factor*r*omega_L)); (1/2)*m*g*C_r*tanh((v_factor*r*omega_P)+tanh(v_factor*r*omega_L))];

% Dynamics
% M = [I_c 0; 0 m];
% V = zeros(2);
% B = [b/r -b/r; 1/r 1/r];

% Output (2.47)
% u_dot = inv(M)*(B*T - V*u - F_a - F_r);

%% Model (2.51)
% @brief: with wheel inertias, bearrings resistances

% B = [b/r -b/r; 1/r 1/r];
% M = [I_c+(2*b^2*I_k)/(r^2) 0; 0 m+(2*I_k)/(r^2)];
% H = [(2*b^2*ksi)/(r^2) 0; 0 (2*ksi)/(r^2)];
% T_n = T;

% Output (2.51)
% u_dot = inv(M)*(B*T_n - H*u - F_a - F_r);

%% Model (2.62)
% @brief: with electric rotor inertia, steering based on torque on electric
%         motor driveshaft

M = [I_c+(2*b^2*I_k)/(r^2)+(2*b^2*I_m)/(r^2*n_g^2) 0; 0 m+(2*I_k)/(r^2)+(2*I_m)/(r^2*n_g^2)];
H = [((2*b^2)/(r^2))*(ksi+(ksi_m/n_g^2)) 0; 0 (2/r^2)*(ksi+(ksi_m/n_g^2))];
B = [b/(r*n_g) -b/(r*n_g); 1/(r*n_g) 1/(r*n_g)];
T_m = T;

% Output (2.62)
u_dot = inv(M)*(B*T_m - H*u - F_a - F_r);

