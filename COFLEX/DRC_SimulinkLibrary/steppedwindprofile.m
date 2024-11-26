function [U, T] = steppedwindprofile(VSTART, VFINAL, TS, TMAX)
% STEPPEDWINDPROFILE - Generates a time-series of a stepped wind profile
%
% [U, T]=STEPPEDWINDPROFILE(VSTART, VFINAL, TS, TMAX) - returns an equally 
% spaced wind speed vector U and corresponding time vector T, representing 
% a stepped wind signal starting from the minimum initial wind speed VSTART 
% and ending at VFINAL. The constant TS defines the signal sampling time and 
% TMAX the total length of the signal.
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

T = 0:TS:TMAX;

if VSTART <= VFINAL
    U = VSTART-1+ceil((VFINAL-VSTART+1)*T/TMAX);
    U(1) = VSTART;  
else
    U = VSTART+ceil((VFINAL-VSTART-1)*T/TMAX);
    U(end) = VFINAL; 
end

end