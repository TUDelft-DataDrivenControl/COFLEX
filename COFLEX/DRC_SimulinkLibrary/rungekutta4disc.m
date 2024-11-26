function [Ad, Bd] = rungekutta4disc(A, B, Ts)
% rungekutta4disc - A classic Runge–Kutta method discretization implementation for state space systems.
%
% [AD, BD] = rungekutta4disc(A, B, TS) - Converts the continuous time A (system), B (input) matrices to
% their discrete time counterparts AD, BD using the Runge–Kutta method. A sampling time TS should be provided in seconds.
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

Ad = 1/24*A^4*Ts^4 + 1/6*A^3*Ts^3 + 1/2*A^2*Ts^2 + A*Ts + eye(size(A));
Bd = 1/24*A^3*B*Ts^4 + 1/6*A^2*B*Ts^3 + 1/2*A*B*Ts^2 + B*Ts;

end

