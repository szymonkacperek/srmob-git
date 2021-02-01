function output = RsgPathFollowingParametred(input)
%% Inputs
chosen_trajectory = input(1);
q = input(2:4);

%% Constants
theta = q(1);
x = q(2);
y = q(3);

% Orientation taken:  (1) forwards, (-1) backwards
zeta_d = 1;

% middle of the trajectory shape described relatively to global coordinates 
% system
X_d = 0;
Y_d = 0;

%% Trajectory choice 
switch chosen_trajectory
    case 0 % straight line
        A_x = 0.5;
        A_y = -0.5;
        % P_S point coordinates (3.54, 3.55)
        p_1 = ((A_x*A_y)/(A_x^2 + A_y^2))*y + (A_x^2/(A_x^2 + A_y^2))*x;
%         p_2 = (A_y^2/(A_x^2 + A_y^2))*y + ((A_x*A_y)/(A_x^2 + A_y^2))*x;
        
        % s param value (8.53)
        s = (p_1*sqrt(A_x^2 + A_y^2))/A_x;
        
        % trajectory (8.53)
        f_xd = (A_x/sqrt(A_x^2 + A_y^2))*s;
        f_yd = (A_y/sqrt(A_x^2 + A_y^2))*s;
        f_xd_dot = A_x/sqrt(A_x^2 + A_y^2);
        f_yd_dot = A_y/sqrt(A_x^2 + A_y^2);
        f_xd_dotdot = 0;
        f_yd_dotdot = 0;
        
    case 1 % circle - q_initial_conditions must be != 0
        A = 1.0; % radius [m]
        mi = 0.1;        
        p_1 = sign(x)*abs(A)*abs(x)/sqrt(x^2 + y^2);
        
        % s param (8.57)
        s = (atan(p_1/A)*abs(A*mi))/mi;
        
        % trajectory
        f_xd = A*cos((mi*s)/abs(A*mi));
        f_yd = A*sin((mi*s)/abs(A*mi));
        f_xd_dot = -sign(A*mi)*sin((mi*s)/abs(A*mi));
        f_yd_dot = sign(A*mi)*cos((mi*s)/abs(A*mi));
        f_xd_dotdot = -(1/A)*cos((mi*s)/abs(A*mi));
        f_yd_dotdot = -(1/A)*sin((mi*s)/abs(A*mi));
end

%% Signals
% Configuration, theta_d \in (-pi, pi]
theta_d = atan2(zeta_d*f_yd_dot, zeta_d*f_xd_dot);
x_d = f_xd;
y_d = f_yd;

% Velocities with trajectory degeneration prevention (page 81)
omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot^2 + f_yd_dot^2);
v_d = zeta_d*sqrt(f_xd_dot^2 + f_yd_dot^2);
if v_d == 0         
    v_d = 0.001;
end
 
% Curvature (3.28)
kappa_d = omega_d*zeta_d;

%% Output
output = [theta_d x_d y_d omega_d v_d kappa_d];

