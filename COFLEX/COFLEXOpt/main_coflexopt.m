%% Create controller set points based on non-linear optimisation formulation
% This script sets up and solves an optimisation problem to find set points for wind turbines.
% It loads necessary data, defines constraints, and runs an optimisation
% over a range of wind speeds to generate control set points.

fprintf('--- Initialising script for wind turbine control set point optimisation ---\n');

%% %%%%%%%%%%%%%%%              INPUTS            %%%%%%%%%%%%%%%%%%%%%% %%
%% Load paths
fprintf('Loading necessary paths...\n');
addpath(fullfile('functions'));                        % Add path for utility functions
addpath(fullfile(pwd, '..', 'DRC_SimulinkLibrary'));   % Add path for Simulink library

%% Load global constants
fprintf('Loading global constants...\n');
loadglobalconstants();                                 % Load constants used globally across the script

%% Load Casadi functionalities from ACADOS
fprintf('Setting up ACADOS environment for Casadi functionalities...\n');
load_acados_files = fullfile('functions','acados_env_variables_windows.m');
run(load_acados_files);

%% Load turbine characteristics
fprintf('Loading turbine characteristics...\n');
turbine_name = 'IEA15MW';
path_to_data = fullfile(pwd, '..', 'data', 'inputs','pwr_tables_FLEX.nc'); % Path to NetCDF dataset from HAWCStab2
turbine = loadturbineconstants(turbine_name, path_to_data, true);          % Load turbine specs and performance characteristics

%% Define saturation limits instead of set points
flag_sat_limits = false;                                                   

%% Specify output filename for set points
output_filename = 'prova.dat';
fprintf('Output file for set points will be saved as: %s\n', output_filename);

%% Set saving options: flag_plot enables plotting, flag_savetotext enables saving to a file
flag_plot = true;
flag_savetotext = true;

%% Set optimisation problem weights for Cp, Cq, and Ct
opt_weights = [-1; 0.1; 0];  % Currently: -Cp + 0.1 * Cq

%% Define output constraints based on physical turbine limits
fprintf('Defining output constraints...\n');
TLowLim          = -inf;                            % Minimum thrust limit [kN]
TUpLim           = 1500;                            % Maximum thrust limit [kN]
OOPTipDispLowLim = -inf;                            % Minimum out-of-plane tip displacement [m]
OOPTipDispUpLim  = inf;                             % Maximum out-of-plane tip displacement [m]
PLowLim          = 0;                               % Minimum power limit [kW]
PUpLim           = (turbine.P_rated) / 10^3;        % Maximum power limit [kW]
QLowLim          = 0;                               % Minimum rotor torque [kNm]
QUpLim           = (turbine.Tg_max / turbine.etag) / 10^3; % Maximum rotor torque [kNm]
FlapMLowLim      = -inf;                            % Minimum flap bending moment [kNm]
FlapMUpLim       = inf;                             % Maximum flap bending moment [kNm]

%% Overwrite minimum pitch and rotational speed (optional)
% turbine.pitch_min = deg2rad(0);             % Minimum collective pitch [rad]
% turbine.wr_min    = rpm2rads * 5;           % Minimum rotational speed [rad/s]

%% Define constraints for control variables
rotLowLim = turbine.wr_min;
rotUpLim = turbine.wr_max;

%% Define wind speed mesh
fprintf('Creating wind speed mesh...\n');
N_ws = turbine.ws_out - turbine.ws_in;
multiplier = 4;                       % Mesh density factor
N_mesh = N_ws * multiplier + 1;
windspeed_vector = linspace(turbine.ws_in, turbine.ws_out, N_mesh);

%% Fixed TSR-tracking scheme parameters (optional)
tsr_fix.flag = false;
if tsr_fix.flag
    fprintf('Defining parameters for fixed TSR-tracking scheme...\n');
    tsr_fix.ws_min = 6.975;
    tsr_fix.ws_max = 10.55;
    tsr_fix.tsr = 9;
    tsr_fix.min_ws_cap = tsr_fix.ws_min;
end

%% %%%%%%%%%%%%%%              EXECUTION            %%%%%%%%%%%%%%%%%%%% %%
%% Optionally create saturation limits by modifying constraints on power and torque
if flag_sat_limits == true
    fprintf('Applying correction to constraints to calculate saturation limits.\n');
    PUpLim = inf;
    QUpLim = inf;
end

%% Set up constraints vectors for non-linear program (NLP)
fprintf('Setting up constraints for optimisation...\n');
x_constraints = struct('RotSpd', [rotLowLim; rotUpLim], 'Pitch', [turbine.pitch_min; turbine.pitch_max]);
g_constraints = struct('P', [PLowLim; PUpLim], 'T', [TLowLim; TUpLim], 'OOPTipDisp', [OOPTipDispLowLim; OOPTipDispUpLim], 'FlapM', [FlapMLowLim; FlapMUpLim], 'Torque', [QLowLim; QUpLim]);

% Generate required fields based on constraints
required_fields = extract_required_fields(g_constraints);
% Validate turbine struct with dynamically created required fields
validate_turbine_struct(turbine, required_fields);

%% Preallocate arrays for optimisation results
fprintf('Preallocating output arrays for optimisation results...\n');
optstruct = struct('f', [], 'g', [], 'lam_g', [], 'lam_p', [], 'lam_x', [], 'x', []);
output = struct();
output.P_values = zeros(length(windspeed_vector), 1);
output.OOPTip_values = zeros(length(windspeed_vector), 1);
output.T_values = zeros(length(windspeed_vector), 1);
output.omega_values = zeros(length(windspeed_vector), 1);
output.pitch_values = zeros(length(windspeed_vector), 1);
output.region_values = zeros(length(windspeed_vector), 1);
output.cp_values = zeros(length(windspeed_vector), 1);
output.ct_values = zeros(length(windspeed_vector), 1);
output.BlFlM_values = zeros(length(windspeed_vector), 1);
output.BlEdM_values = zeros(length(windspeed_vector), 1);
output.GenQ_values = zeros(length(windspeed_vector), 1);

%% Set initial values
fprintf('Setting initial values for optimisation...\n');
omega_init    = turbine.wr_min;
pitch_init    = deg2rad(1);
P_values_init = 0;
region        = 1;

%% Tolerances for region switching (display only)
rot_tol = 0.001;  % Rotational speed tolerance
pow_tol = 1;      % Power tolerance (1 kW above rated)

%% Solve non-linear programs (NLPs)
verbose = false; % Verbose output flag
fprintf('Starting optimisation over wind speed range...\n');

% Initialize progress bar and start timer
progress_bar = waitbar(0, 'Calculating...');
tic;

% Loop over wind speed mesh for optimisation
for i = 1:length(windspeed_vector)
    windspeed_fixed = windspeed_vector(i);
    
    % Solve NLP for each wind speed, using previous solution as an initial guess if available
    if i == 1
        optstruct(i) = solve_NLP(turbine, opt_weights, windspeed_fixed, x_constraints, g_constraints, tsr_fix, verbose);
    else
        optstruct(i) = solve_NLP(turbine, opt_weights, windspeed_fixed, x_constraints, g_constraints, tsr_fix, verbose, optstruct(i-1));
    end
    
    % Store solution in output struct
    output.P_values(i)      = full(optstruct(i).g(1, 1)); % Power (kW)
    output.T_values(i)      = full(optstruct(i).g(2, 1)); % Thrust (kN)
    output.OOPTip_values(i) = full(optstruct(i).g(3, 1));
    output.omega_values(i)  = full(optstruct(i).x(1, 1));
    output.pitch_values(i)  = full(optstruct(i).x(2, 1));
    output.cp_values(i)     = full(turbine.Cp([output.omega_values(i), windspeed_fixed, output.pitch_values(i)]));
    output.ct_values(i)     = full(turbine.Ct([output.omega_values(i), windspeed_fixed, output.pitch_values(i)]));
    output.BlFlM_values(i)  = full(turbine.FlapM([output.omega_values(i), windspeed_fixed, output.pitch_values(i)]));
    output.BlEdM_values(i)  = full(turbine.EdgeM([output.omega_values(i), windspeed_fixed, output.pitch_values(i)]));
    output.GenQ_values(i)   = (output.P_values(i) * turbine.etag) / output.omega_values(i);

    % Determine operational region based on power and rotational speed
    if i == 1
        show_region = 'region 1';
    elseif abs(output.P_values(i-1) - PUpLim) > pow_tol && region == 1 && abs(output.omega_values(i-1) - turbine.wr_min) < rot_tol
        show_region = 'region 1';
    elseif abs(output.P_values(i-1) - PUpLim) > pow_tol && region == 1
        region = 2;
        show_region = 'region 2';
    elseif abs(output.P_values(i-1) - PUpLim) > pow_tol && region == 2
        show_region = 'region 2';
    else
        region = 3;
        show_region = 'region 3';
    end
    output.region_values(i) = int16(region);

    % Update progress bar
    waitbar(i / length(windspeed_vector), progress_bar, sprintf('Calculating values for %s - Elapsed time: %.1f seconds', show_region, toc));
end

% Close progress bar and display execution time
close(progress_bar);
execution_time = toc;
fprintf('Execution completed in %.1f seconds.\n', execution_time);

%% %%%%%%%%%%%%%%               OUTPUT              %%%%%%%%%%%%%%%%%%%% %%
%% Plot results
if flag_plot == true
    fprintf('Plotting optimisation results...\n');
    plot_output(output, windspeed_vector, turbine);
end

%% Save results to a text file
if flag_savetotext == true
    fprintf('Saving results to %s...\n', output_filename);
    results_matrix = [windspeed_vector', output.region_values, output.P_values, output.T_values, output.OOPTip_values, output.omega_values, output.pitch_values];
    header_text = 'Wind speed [m/s], Region [-], Power [kW], Thrust [kN], Out-of-plane Tip displacement [m], Rotational speed [rad/s], Pitch [rad]';
    writematrix(results_matrix, output_filename, 'Delimiter', 'tab');
    fprintf('Results saved successfully.\n');
end