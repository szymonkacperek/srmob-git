%% LABORATORUM SRMob
% ĆWICZENIE 1 - Model matematyczny robota mobilnego

% Plik funkcji
%---- • NOTATKI
%
%% 
function q_dot = KinematicModel(input)

theta = input(1);
omega = input(2);
v = input(3);


u = [omega; v];
G = [1 0; 0 cos(theta); 0 sin(theta)];

q_dot = G * u;