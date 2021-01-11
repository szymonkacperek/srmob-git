function output = RsgTrajectoryTracking(input)

%% Inputs
% t = input(1);
t = input(1);
chosen_trajectory = 3;

%% Parameters
% Orientation taken:  1 - forward 
%                    -1 - backward
zeta_d = 1;

%% Initial conditions
% ksi_d = 1;
% theta_d_t0 = 0.3;

%% Ga method (3.1 - 3.9)
% omega_d = 0;
% v_d = 2;

% % (3.1) - (3.3)
% theta_d = theta_d_t0 + ode45(omega_d(t));
% x_d = ;
% y_d = ;

% % (3.4) - (3.6)
% theta_d_dot = omega_d;
% x_d_dot = v_d*cos(theta_d);
% y_d_dot = v_d*sin(theta_d);
% q_d_dot = [theta_d_dot; x_d_dot; y_d_dot];

% % (3.7) - (3.9)
% theta_d_dotdot = omega_d_dot;
% x_d_dotdot = -omega_d*v_d*sin(omega_d) + v_d_dot*cos(theta_d);
% y_d_dotdot = omega_d*v_d*cos(theta_d) + v_d_dot*sin(omega_d);
% q_d_dotdot = [theta_d_dotdot; x_d_dotdot; y_d_dotdot];


%% Universal function for trajectory drawing (page 108)
% @param X_d, Y_d: middle of the trajectory shape described relatively to global
%                  coordinate system

X_d = 0;
Y_d = 0;

% Choose trajectory between 1 - 3 with 'chosen_trajectory' variable
switch chosen_trajectory
    % Circle shape
    case 1
        A_dx = 1;
        A_dy = 1;
        omega_dx = 0.5;
        omega_dy = 0.5;
        psi_dx = 0;
        psi_dy = 0;

    % Ellipse
    case 2    
        A_dx = 1;
        A_dy = 0.5;
        omega_dx = 0.5;
        omega_dy = 0.5;
        psi_dx = 0;
        psi_dy = 0;

    % Eight
    case 3
        A_dx = 0.5;
        A_dy = 0.5;
        omega_dx = 0.5;
        omega_dy = 2*omega_dx;
        psi_dx = -pi/2;
        psi_dy = 0;
end

% Trajectory shape and their derivatives
f_xd = X_d + A_dx*cos(omega_dx*t + psi_dx);
f_yd = Y_d + A_dy*sin(omega_dy*t + psi_dy);

f_xd_dot = -A_dx*omega_dx*sin(omega_dx*t + psi_dx);
f_yd_dot = A_dy*omega_dy*cos(omega_dy*t + psi_dy);

f_xd_dotdot = -A_dx*omega_dx^2*cos(omega_dx*t + psi_dx);
f_yd_dotdot = -A_dy*omega_dy^2*sin(omega_dy*t + psi_dy);

% Velocities
v_d = zeta_d*sqrt(f_xd_dot^2 + f_yd_dot^2);
omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot^2 + f_yd_dot^2);
u_d = [v_d; omega_d];

% Configuration
theta_d = atan2(zeta_d*f_yd_dot, zeta_d*f_xd_dot);
x_d = f_xd;
y_d = f_yd;
q_d = [theta_d x_d y_d];

%% Output
output = [theta_d x_d y_d omega_d v_d];
% output = [theta_d x_d y_d];
