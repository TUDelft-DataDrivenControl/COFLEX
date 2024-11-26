function [U, T] = mexicanhatwindprofile(VSTART, TS, TMAX, varargin)
% MEXICANHATWINDPROFILE - Generates a time-series of a Mexican hat wind profile
%
% [U, T]=MEXICANHATWINDPROFILE(VSTART, VFINAL, TS, TMAX) - returns an equally
% spaced wind speed vector U and corresponding time vector T, representing
% a Mexican hat wind signal starting from the minimum initial wind speed VSTART
% and ending at VFINAL. The constant TS defines the signal sampling time and
% TMAX the total length of the signal.
%
% [U, T]=MEXICANHATWINDPROFILE(VSTART, VFINAL, TS, TMAX, NCYCLES, AMPFAC) -
% XXX
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021
if ~isempty(varargin)
    NCYCLES = varargin{1};
    AMPFAC = varargin{2};
    NUMMINWIND = varargin{3};
else
    NCYCLES = 1;
    AMPFAC = 4;
    NUMMINWIND = 10;
end

timeHose = -NCYCLES*pi:TS:NCYCLES*pi;
% timeHose = linspace();
Hose = AMPFAC*(sin(timeHose)./timeHose) + VSTART;

% fill wind data array with min minSpeed in start & end
minWind = (zeros(1,length(timeHose)*NUMMINWIND) + 1)*VSTART;
totalWind = [minWind, Hose, minWind];

U = totalWind;
T = 0:TS:(length(totalWind)-1)*TS;

end

