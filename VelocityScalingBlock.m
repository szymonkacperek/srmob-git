%% LABORATORIUM SRMob
% ĆWICZENIE 1 - Model matematyczny robota mobilnego

% Velocity scaling block

function Omega_ds = VelocityScalingBlock(input)

%% Inputs
Omega_dc = input(1:2);

%% Constants
global omega_k_max

%%
k_s = max([1; abs(Omega_dc(1))/omega_k_max; abs(Omega_dc(2))/omega_k_max]);

Omega_ds = (1/k_s)*Omega_dc;