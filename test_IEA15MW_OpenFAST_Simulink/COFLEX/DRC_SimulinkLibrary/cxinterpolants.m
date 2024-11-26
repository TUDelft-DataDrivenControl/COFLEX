function [CXInterpolant, dCXPITCHInterpolant, dCXTSRInterpolant] = cxinterpolants(CX, PITCH, TSR, varargin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

PLOT = false;
[m,n] = size(CX);

if ~isempty(varargin)
    PLOT = varargin{1};
end

if ~isvector(PITCH) || ~isvector(TSR)
    error('PITCH and TSR should both be one-dimensional vectors');
end

if length(PITCH) ~= m || length(TSR) ~= n
    error('The dimensions of CX are inconsistent with these of PITCH and/or TSR');
end

[PITCH_ND, TSR_ND] = ndgrid(PITCH, TSR); % For griddedInterpolant
[dCXPITCH, dCXTSR] = gradient(CX', PITCH, TSR); % Numerical gradient CX table

CXInterpolant = griddedInterpolant(PITCH_ND, TSR_ND, CX);
dCXPITCHInterpolant = griddedInterpolant(PITCH_ND, TSR_ND, dCXPITCH');
dCXTSRInterpolant = griddedInterpolant(PITCH_ND, TSR_ND, dCXTSR');

if PLOT
    figure(100)
    mesh(PITCH_ND, TSR_ND, CX)
    xlabel('Pitch angle')
    ylabel('Tip-speed ratio')
    zlabel('Value')
    title('Coefficient plot')
    
    figure(101)
    mesh(PITCH_ND, TSR_ND, dCXPITCH')
    xlabel('Pitch angle')
    ylabel('Tip-speed ratio')
    zlabel('Value')
    title('Gradient coefficient wrt. PITCH')
    
    figure(102)
    mesh(PITCH_ND, TSR_ND, dCXTSR')
    xlabel('Pitch angle')
    ylabel('Tip-speed ratio')
    zlabel('Value')
    title('Gradient coefficient wrt. TSR')
end

end

