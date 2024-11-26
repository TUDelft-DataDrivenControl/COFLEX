function K = optimalmodegain(rho, R, tsr, Cp, G)
% Calculates optimal mode gain K for use in a K-omega-squared controller in
% [Nm/(rad/s)^2].
% 
% K = optimalmodegain(rho, R, tsr, Cp, G), with
% rho = Air density [kg/m^3]
% R = rotor radius [m]
% tsr = tip-speed ratio for provided power coefficient C_p [-]
% Cp = Power coefficient value for considered operating point [-]
% G = Gearbox ratio (>= 1, w_g/w_r) [-]. Optional, if not provided, G = 1
% and calculates K for the LSS

if nargin == 4
    G = 1;
end

K = pi*rho*R^5*Cp/(2*tsr^3*G^3);


%% Add an expression for VAWT
% K = rho*A*R^3*Cp/(2*tsr^3*G^3);

end

