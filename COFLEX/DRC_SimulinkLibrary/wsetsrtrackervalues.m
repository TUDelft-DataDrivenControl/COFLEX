function [A1, A2, B11, B12, B22] = wsetsrtrackervalues(P, U, PITCH, TSR, Cp, Cp_Pitch, Cp_TSR, CPUncertainty, varargin)
% WSETSRTRACKERVALUES - Gives numerical values in the symbolical WSE+TSR
% tracker derivation.
%
% [B22] = wsetsrtrackervalues(P, U_bar, pitch_bar, lambda_bar) - XXX

if ~isempty(varargin)
    if strcmpi(varargin{1}, '+') || strcmpi(varargin{1}, 'plus')
        multadd = false;
    elseif strcmpi(varargin{1}, '*') || strcmpi(varargin{1}, 'times')
        multadd = true;
    end
else
    multadd = true;
end

J = P.Jlss;
RHO = P.rho;
R = P.R;
A = P.A;

WR = tsr(TSR, U, R, 'WR');

[Cpint, dCpPint, dCpTint] = cxinterpolants(Cp, Cp_Pitch, Cp_TSR);
CPVAL = Cpint(PITCH,TSR);
dCPPVAL = dCpPint(PITCH,TSR);
dCPTVAL = dCpTint(PITCH,TSR);

if multadd % product
    [Cphatint, dCpPhatint, dCpThatint] = cxinterpolants(Cp*CPUncertainty, Cp_Pitch, Cp_TSR);
else % additive
    [Cphatint, dCpPhatint, dCpThatint] = cxinterpolants(Cp+CPUncertainty, Cp_Pitch, Cp_TSR);
end
CPHATVAL = Cphatint(PITCH,TSR);
dCPPHATVAL = dCpPhatint(PITCH,TSR);
dCPTHATVAL = dCpThatint(PITCH,TSR);

A1 = (1/J)*0.5*RHO*A*U^3*(dCPTVAL*R/(U*WR) - CPVAL/WR^2); %1/J * dTr/dWR
A2 = (1/J)*0.5*RHO*A*U^3*(dCPTHATVAL*R/(U*WR) - CPHATVAL/WR^2); %1/J * dTrhat/dWR
B11 = -1/J;
B12 = (1/J)*0.5*RHO*A*U*(3*U*CPVAL/WR - R*dCPTVAL); %1/J * dTr/dU
B21 = -1/J;
B22 = (1/J)*0.5*RHO*A*U*(3*U*CPHATVAL/WR - R*dCPTHATVAL); %1/J * dTrhat/dUhat

end

