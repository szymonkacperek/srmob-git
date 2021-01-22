function output = RsgPathFollowingNonParametred(input)
%% Inputs
chosen_trajectory = input(1);
q = input(2:4);

% Constants
theta = q(1);
x = q(2);
y = q(3);

%% Parameters
zeta_d = 1;
sigma = 0.6;
a = 1.0;
b = 2.0;

%%
switch chosen_trajectory
    case 0 % straight line
        f_d = sigma*(y-a*x-b);
        f_xd_dot = sigma*a;
        f_yd_dot = sigma;
        f_xd_dotdot = 0;
        f_yd_dotdot = 0;
    case 1 %  circle, ellipse
        f_d = sigma*((x/a)^2+(y/b)^2-1);
        f_xd_dot = sigma*2*x/(a^2);
        f_yd_dot = sigma*2*y/(b^2);
        f_xd_dotdot = sigma*2/(a^2);
        f_yd_dotdot = sigma*2/(b^2);
    case 2 % superellipse
        n = 10;
        f_d = sigma*(abs(x/a)^n + abs(y/b)^n - 1);
        f_xd_dot = sigma*((1/(a^n))*n*x^(n-1)*(sign(x))^n);
        f_yd_dot = sigma*((1/(b^n))*n*y^(n-1)*(sign(y))^n);
        f_xd_dotdot = sigma*((1/(a^n))*n*(n-1)*x^(n-2)*(sign(x))^n);
        f_yd_dotdot = sigma*((1/(b^n))*n*(n-1)*y^(n-2)*(sign(y))^n);     
end

%% Vectors needed to count velocities
% Ortogonal to contour gradient 
% nabla_f_d = gradient(f_d);

%% Velocities with trajectory degeneration prevention (page 81)
omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot^2 + f_yd_dot^2);
v_d = zeta_d*sqrt(f_xd_dot^2 + f_yd_dot^2);
if v_d == 0         
    v_d = 0.001;
end

%% Output
output = [omega_d v_d f_xd_dot f_yd_dot];
