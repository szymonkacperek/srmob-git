function output = Atan2c(y, x, phi_1)

Phi = atan2(y, x);
Phi_1 = atan2(sin(phi_1), cos(phi_1));
delta_Phi = Phi - Phi_1;

if delta_Phi > pi
    delta_phi = delta_Phi - 2*pi;
elseif delta_Phi < -pi
    delta_phi = delta_Phi + 2*pi;
else
    delta_phi = delta_Phi;
end

phi = phi_1 + delta_phi;

output = phi;