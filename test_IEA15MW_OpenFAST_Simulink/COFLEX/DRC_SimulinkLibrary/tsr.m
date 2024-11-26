function output = tsr(inputArg1,inputArg2, inputArg3, varargin)
% tsr - Calculates the tip-speed ratio given rotor speed, wind speed and rotor radius.
%
% LAMBDA = tsr(WR, U, R) - Calculates the tip-speed ratio LAMBDA [-] given
% the rotor speed WR [rad/s], wind speed U [m/s] and rotor radius R [m].
%
% Optionally another output quantity can be calculated by specifying an
% optional 4th input. The following arguments are allowed:
% 1. WR = tsr(LAMBDA, U, R, 'WR');
% 2. U = tsr(WR, LAMBDA, R, 'U');
% 3. R = tsr(WR, U, LAMBDA, 'R');
% 
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

nvarargin = length(varargin);
switch nvarargin
    case 0
        wr = inputArg1;
        U = inputArg2;
        R = inputArg3;
        lambda = wr*R/U;
        output = lambda;
    case 1
        if strcmpi(varargin{1}, 'WR')
            lambda = inputArg1;
            U = inputArg2;
            R = inputArg3;
            wr = lambda*U/R;
            output = wr;
        elseif strcmpi(varargin{1}, 'U')
            wr = inputArg1;
            lambda = inputArg2;
            R = inputArg3;
            U = wr*R/lambda;
            output = U;
        elseif strcmpi(varargin{1}, 'R')
            wr = inputArg1;
            U = inputArg2;
            lambda = inputArg3;
            R = lambda*U/wr;
            output = R;
        else
            error('Invalid optional argument given at position 4');
        end
    otherwise
        error('Invalid amount of arguments provided');
end

end

