function T = aerodynamictorque(rho, R, Cp, V, TSR)
% aerodynamictorque - Calculates the aerodynamic torque of a wind turbine
% rotor
%
% T = aerodynamictorque(RHO, R, CP, V, TSR) - Calculates the aerodynamic 
% rotor torque given the air density RHO, rotor radius R, power coefficient
% value CP, rotor effective wind speed V, and tip-speed ratio TSR.
% 
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

A = pi*R^2;

T = 0.5*rho*A*R*V^2*Cp/TSR;

end