function output = RsgTrajectoryTracking(input)

%% Inputs
t = input(1);
chosen_trajectory = input(2);

%% Universal function (f_xd) for trajectory drawing (page 108)
% Orientation taken:  (1) forwards, (-1) backwards
zeta_d = 1;

% @param X_d, Y_d: middle of the trajectory shape described relatively to 
%                  global coordinates system
X_d = -1;
Y_d = -1;

switch chosen_trajectory
    % Straight line
    case 0
        A_dx = 0.5;
        A_dy = A_dx;

    % Circle shape
    case 1
        A_dx = 1;
        A_dy = 1;
        omega_dx = 0.2;
        omega_dy = omega_dx;
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
        
    % Point
    case 4
        A_dx = 0.5;
        A_dy = 0.5;
        theta_d = 0;
        omega_dx = 0;
        omega_dy = 0;
        psi_dx = 0;
        psi_dy = 0;
end

% Trajectory shape and their derivatives. Calculate derivatives in a
% different way because of straight line shape
if chosen_trajectory == 0
    f_xd = X_d + A_dx*t;
    f_yd = Y_d + A_dy*t;
    
    f_xd_dot = A_dx;
    f_yd_dot = A_dy;
    
    f_xd_dotdot = 0;
    f_yd_dotdot = 0;
    
elseif chosen_trajectory == 4
    f_xd = A_dx;
    f_yd = A_dy;

    f_xd_dot = 0.01;
    f_yd_dot = 0.01;
    
    f_xd_dotdot = 0.01;
    f_yd_dotdot = 0.01;
    
else
    f_xd = X_d + A_dx*cos(omega_dx*t + psi_dx);
    f_yd = Y_d + A_dy*sin(omega_dy*t + psi_dy);
    
    f_xd_dot = -A_dx*omega_dx*sin(omega_dx*t + psi_dx);
    f_yd_dot = A_dy*omega_dy*cos(omega_dy*t + psi_dy);
    
    f_xd_dotdot = -A_dx*omega_dx^2*cos(omega_dx*t + psi_dx);
    f_yd_dotdot = -A_dy*omega_dy^2*sin(omega_dy*t + psi_dy);
end

%% Velocities with trajectory degeneration prevention (page 81)
v_d = zeta_d*sqrt(f_xd_dot^2 + f_yd_dot^2);
if v_d == 0         
    v_d = 1.0;    
end
omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot^2 + f_yd_dot^2);

% Configuration, theta_d \in (-pi, pi], theta_d_cont \in R
global old_theta_d
if chosen_trajectory ~= 4 
    theta_d = atan2(zeta_d*f_yd_dot, zeta_d*f_xd_dot);
    theta_d_cont = Atan2c(zeta_d*f_yd_dot, zeta_d*f_xd_dot, old_theta_d);
end
old_theta_d = theta_d;

x_d = f_xd;
y_d = f_yd;

%% Output
output = [theta_d x_d y_d omega_d v_d f_xd_dot f_yd_dot f_xd_dotdot f_yd_dotdot];

