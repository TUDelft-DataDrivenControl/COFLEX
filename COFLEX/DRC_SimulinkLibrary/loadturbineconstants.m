function P = loadturbineconstants(turbineName,path_to_HS2data,flag_load_all)
% Syntax:
%   P = loadturbineconstants(turbineName, path_to_HS2data, flag_load_all)
%
% Description:
% loadturbineconstants - Returns a structure with constants and
% (optionally) B-splines of the performance of the turbine loaded from HS2
%
% Input Arguments:
%   turbineName     - string specifying the reference wind turbine model.
%
% Optional Input Arguments:
%   path_to_HS2data - string specifying the HS2 file with reference wind turbine model performance. (default: empty)
%   flag_load_all   - boolean specifying if all the performance must be loaded, if set to false only 'Cp' and 'Ct' are loaded (default: empty)
% Output Arguments:
%   P               - structure containing all the wind turbines
%   characteristics
%
% Currently the following turbines are supported:
% - NREL5MW
% - IEA15MW
% S.P. Mulders (Sebastiaan) and G.Lazzerini (Guido)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2024

% Check if path_to_HS2data is provided, otherwise assign a default value
if nargin < 2 || isempty(path_to_HS2data)
    flag_load_HS2 = false; % Default value for "flag_load_HS2"
else
    flag_load_HS2 = true;
end

% Check if flag_load_all is provided, otherwise assign a default value
if nargin < 3 || isempty(flag_load_all)
    flag_load_all = false; % Default value for "flag_load_all"
end

rpm2rads = pi/30;

switch turbineName

    case 'NREL5MW'

    P.H = 90;                               % Tower height [m]
    P.R = 63;                               % Rotor length [m]
    P.A = pi*P.R^2;                         % Rotor area [m^2]
    P.rho = 1.225;                          % Air density [kgm^-3]
    P.G = 97;                               % Gearbox ratio [-]
    P.Jb = 11776047;                        % Second mass moment of inertia of a single blade, at LSS [kg m^2]
    P.Jh = 115926;                          % Second mass moment of inertia of the hub, at LSS [kg m^2]
    P.Jg = 534.116;                         % Second mass moment of inertia of the generator, at HSS [kg m^2]
    P.Jlss = 3*P.Jb + P.Jh + P.Jg*P.G^2;    % Second mass moment of inertia of rotor, at LSS [kg m^2]
    P.Jhss = P.Jlss/P.G^2;                  % Second mass moment of inertia of rotor, casted to HSS [kg m^2]
    
    P.k_dt = 8.67637e08;                    % Drivetrain torsional spring constant, at LSS (N-m/rad)
    P.c_dt = 6.215e06;                      % Drivetrain torsional damper coefficient, at LSS (N-m/(rad/s))
    P.etag = 0.944;                         % Generator efficiency [-]
    
    P.wg_min = 0*rpm2rads;                  % Minimum generator speed [rad/s]
    P.wg_max = 1600*rpm2rads;               % Maximum generator speed [rad/s]
    P.wg_rated = 1173.7*rpm2rads;           % Rated generator speed [rad/s]
    P.wr_min = P.wg_min/P.G;                % Minimum generator speed [rad/s]
    P.wr_max = P.wg_max/P.G;                % Maximum generator speed [rad/s]
    P.wr_rated = P.wg_rated/P.G;            % Maximum generator speed [rad/s]
    P.beta_fine = deg2rad(0);               % Fine pitch angle [rad]
    P.Tg_rated = 43093.55;                  % Minimum generator torque (mechanical) [Nm]
    P.Tg_min = -100;                        % Minimum generator torque (mechanical)[Nm]
    P.Tg_max = P.Tg_rated*2;                % Minimum generator torque (mechanical) [Nm]
    P.Pg_rated = 5e6;                       % Rated power [W]
    P.Pg_min = -100;                        % Rated power [W]

    case 'IEA15MW'
%   Onshore version

%   - Environmental 
    P.rho = 1.225;                          % Air density [kgm^-3]

%   - Turbine dimensions 
    P.R = 120;                              % Rotor length [m]
    P.A = pi*P.R^2;                         % Rotor area [m^2]
    P.H = 150;                              % Tower height [m]
    P.shaft_tilt_angle = deg2rad(6);        % Shaft tilt angle [rad]

%   - Turbine masses and inertia 
    P.Jb     = 0;                           % Second mass moment of inertia of a single blade, at LSS [kg m^2]
    P.Jrotor = 3.524605e8;                  % Second mass moment of inertia of rotor, at LSS [kg m^2] calculated  from  the  HAWC2  blade  structural  file  while  ignoring  coning  and  prebend
    P.JShaft = 2.81030e6;                   % Second mass moment of inertia of shaft from HAWC2 output (also equal to generator + hub inertia in OF files) [kg m^2]
    P.Jh     = 9.7352e5;                    % NOT USED % Second mass moment of inertia of the hub, at LSS [kg m^2]
    P.Jg     = 1.836784e6;                  % NOT USED % Second mass moment of inertia of the generator, at HSS [kg m^2]
    P.Jlss   = P.Jrotor + P.JShaft;         % Second mass moment of inertia of rotor, at LSS [kg m^2]
    P.G      = 1;                           % Gearbox ratio [-]
    P.Jhss   = P.Jlss/P.G^2;                % Second mass moment of inertia of rotor, casted to HSS [kg m^2]

%   - Drivetrain     
    P.k_dt = 0;                             % Drivetrain torsional spring constant, at LSS (N-m/rad)
    P.c_dt = 0;                             % Drivetrain torsional damper coefficient, at LSS (N-m/(rad/s))
    P.etag = 0.9655;                        % Generator efficiency [-] (at full load)
    
    k1 = 0.0;
    k2 = 0.0;
%   - Rotational speeds - Generator
    P.wg_min   = (1-k1)*5.00*rpm2rads;      % Minimum generator speed [rad/s]
    P.wg_max   = (1+k1)*7.55*rpm2rads;      % Maximum generator speed [rad/s]
    P.wg_rated = 7.55*rpm2rads;             % Rated generator speed [rad/s]

%   - Rotational speeds - Rotor
    P.wr_min   = P.wg_min/P.G;              % Minimum rotor speed [rad/s]
    P.wr_max   = P.wg_max/P.G;              % Maximum rotor speed [rad/s]
    P.wr_rated = P.wg_rated/P.G;            % Rated rotor speed [rad/s]

%   - Power and Torque
    P.Pg_rated  = 1.5e7;                         % Rated generator power [W]
    P.Pg_min    = 0;                             % Minimum power [W]
    P.Tg_rated  = P.Pg_rated/P.wg_rated;         % Rated generator torque (mechanical) [kNm]
    P.Tg_min    = 0;                             % Minimum generator torque (mechanical)[kNm]
    P.Tg_max    = (1+k2)*P.Tg_rated;             % Maximum generator torque (mechanical) [kNm]
    P.Tr_max    = (1+k2)*P.Tg_rated/P.etag*P.G;  % Maximum rotor torque (mechanical) [kNm]
    P.P_rated   = P.Pg_rated/P.etag;             % Rated rotor power [W]

%   - Controller parameters
    P.beta_fine = deg2rad(0);               % Fine pitch angle [rad]
    P.Cp_max    = 0.50;                     % Max Cp [-]
    P.TSR_star  = 9;                        % TSR star [-]
    P.ws_in     = 3;                        % Cut-in wind-speed [m/s]
    P.ws_out    = 25;                       % Cut-out wind-speed [m/s]
    P.pitch_min = deg2rad(-5);              % Minimum collective pitch [rad]
    P.pitch_max = deg2rad(30);              % Maximum collective pitch [rad]
    P.pitch_min_rate = -0.0349;             % Minimum collective pitch rate [rad/s]
    P.pitch_max_rate =  0.0349;             % Maximum collective pitch rate [rad/s]

    if flag_load_HS2 == true

        %%   - Create symbolic functions interpolating HAWCStab2 performance
        %   using CasADi interface (needed)

        % Load information about the NetCDF file
        info = ncinfo(path_to_HS2data);

        % Access the variables present in the NetCDF file
        variables = {info.Variables.Name};

        % Exclude variables with the same name as dimensions
        dimensions = {info.Dimensions.Name};
        variables_to_load = {};

        for i = 1:numel(variables)
            varName = variables{i};
            if ~any(strcmp(varName, dimensions))
                variables_to_load = [variables_to_load, varName];
            end
        end

        % If the flag for loading all the variables is active, then all outputs
        % of HAWCStab2 are loaded in CasADi 3D splines, otherwise only "Cp" and
        % "Ct" are loaded as 3D B-splines.

        if flag_load_all == true
            % Create splines for all variables
            for i = 1:numel(variables_to_load)
                varName = variables_to_load{i};

                % Display the variable name being processed
                fprintf('Creating B-spline for: %s\n', varName);

                % Generate and store the spline
                splineVar = generateSpline(path_to_HS2data, varName);
                P.(varName) = splineVar;
            end

            % Calculate new TipDisp and OOPTipDisp (m) based on hardcoded initial
            % position of the tip.
            Tipx0 = -0.0658936;
            Tipy0 = -4.00143 - 8.15;
            Tipz0 = 117;

            TipDisp = @(v) sqrt((P.Tipx(v) - Tipx0)^2 + (P.Tipy(v) - Tipy0)^2 + (P.Tipz(v) - Tipz0)^2);
            OOPTipDisp = @(v) P.Tipy(v) - Tipy0;

            Q = @(v) (0.5 * P.rho * P.V(v)^3 * P.A * P.Cp(v)) / (1000 * P.Speed(v) * rpm2rads);  % Rotor Torque (kNm)

            % Store splines in structure P
            P.TipDisp = TipDisp;
            P.OOPTipDisp = OOPTipDisp;
            P.Q = Q;

        else
            % Load only Cp and Ct
            varName = 'Cp';
            fprintf('Creating B-spline for: %s\n', varName);
            splineCp = generateSpline(path_to_HS2data, varName);
            P.Cp = splineCp;

            varName = 'Ct';
            fprintf('Creating B-spline for: %s\n', varName);
            splineCt = generateSpline(path_to_HS2data, varName);
            P.Ct = splineCt;
        end

        % You can retrieve 3D splines symbolic functions from this structure like this:
        % Cp = P.splineCp([RotSpd,WndSpd,Pitch]);
        % Ct = P.splineCt([RotSpd,WndSpd,Pitch]);
        % where RotSpd,WndSpd and Pitch can be symbolic variables themselves or
        % exact values. These can also be mixed like  Cp =
        % P.splineCp(([RotSpd,10,Pitch])
    else
        fprintf('HS2 data not loaded.\n');
    end
    otherwise

    error('Invalid turbine name')

end

end