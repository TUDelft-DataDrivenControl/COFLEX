function [index] = findnearest(array, value)
% findnearest - Finds the nearest number in an array
% [INDEX] = findnearest(NUMARR, VAL) - Giving a povided n-dimensional array NUMARR
% and a value VAL, return the index of the nearest value in the array. 
% NUMARR does not have to be monotonically increasing.
% 
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

	if length(array) <= 1
		error('Provided NUMARR should have a length > 1')
	end
	
    [~,index] = (min(abs(array - value)));
end