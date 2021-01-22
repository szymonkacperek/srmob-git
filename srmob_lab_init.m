%% LABORATORIUM SRMob
% ĆWICZENIE 1 - Model matematyczny robota mobilnego

% Init file
close all; clear all; clc

%% Simulation parameters
f_p = 18; % [Hz]
t_p = 2.0; % [s]
t_pp = 0.2; % [s]
t_end = 5.0; % [s]

%% Robot parameters
global I_c m b r I_k J I_m n_g
m = 0.6; % mass with battery [kg]
b = 0.033; % half of the track width [m]
r = 0.026; % wheel radius [m]
a = 0.078; % robot is in shape of cuboid - here is length of first base [m]
c = 0.078; % as above, second line of base [m]
I_c = (m*(a^2 + c^2))/12; % whole robot inertia [kg*m^2]
n_g = 1/12; % transmission factor [-]
N = [n_g 0; 0 n_g];
m_k = 0.03; % mass of a wheel [kg]
I_k = (m_k*r^2)/2; % wheel inertia [kg*m^2]
I_m = 4.22 * 1e-3 * 1e-4; % electric rotor inertia [kg*m^2/A]`
R_m = [3.78 0; 0 3.78]; % coil resistance [ohm]
k_m = 8.55 * 1e-3; % torque constant of motor [N*m/A]
K_m = [k_m 0; 0 k_m]; 

% Velocity and position conversion to wheel velocities
J = [r/(2*b) -r/(2*b); r/2 r/2];
inv_J = inv(J);
inv_R_m = inv(R_m);
inv_N = inv(N);

%% Integrator's initial conditions
% robot start position [deg, m, m]
q_initial_conditions = [0; -0.5; -1];

% robot start velocities [rad/s, rad/s^2]
u_initial_conditions = [0; 0];

%% Resistance factors
% wheel rolling, air resistance
global D_aomega D_av g C_r v_factor
D_aomega = 5e-4;
D_av = 5e-3;
C_r = 5e-4;
g = 9.81; % [m/s^2]
v_factor = 2.0;

% wheel inertia, bearings resistance
global ksi ksi_m
ksi = 1e-4;
ksi_m = 1.95e-8;

%% Power supply
% voltage limits
u_limit = 9.0; % V
u_max = [u_limit; u_limit];
u_min = [-u_limit; -u_limit];

%% Controller synthesis
% @brief: Inertial object, I rule
% @param K_p: 


K = 84.48/9.0; % [rad/s / V]; 
T = 0.098; % for 63% of object's answer [s]

% Oscillation object parameters for mobile robots
zeta_0 = 1; % suppression in [0.707; 1]
omega_0 = 126.0; % pulsation

% PI tuning
k_p = (2*T*zeta_0*omega_0 - 1) / K;
k_i = (T*omega_0^2) / (K*k_p);

% Filter
T_F = 1/k_i;

% Anti wind-up
k_c = k_i+50; %-1;%54.1;

omega_0_range_and_kp_ki = [1/(2*T*zeta_0) (2*pi*f_p)/zeta_0; k_p k_i]

%% Velocity scaling block
% @brief: In order to scale velocities - lets wheel scale its velocity when
%         signal saturation occurs, making going round a circle available
%         with low voltages
global omega_k_max

% @param: Maximal wheel velocity
omega_k_max = 80.0; % [rad/s]
