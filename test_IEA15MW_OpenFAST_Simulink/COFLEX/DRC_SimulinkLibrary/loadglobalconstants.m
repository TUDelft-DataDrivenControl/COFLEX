function loadglobalconstants()

% LOADGLOBALCONSTANTS - Provides frequently used constants
% 
% LOADGLOBALCONSTANTS() - Loads frequently used constants in the base
% workspace. Constants include conversion factors, ... .
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

rpm2rads = pi/30; % Conversion factor [(rad/s)/rpm]
assignin('base','rpm2rads',rpm2rads);

rads2rpm = 30/pi; % Conversion factor [rpm/(rad/s)]
assignin('base','rads2rpm',rads2rpm);

currentDT = datestr(now(), 'yyyymmddHHMM');
assignin('base','currentDT',currentDT);

end