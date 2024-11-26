% clc, 
% clear all
% close all

addpath(fullfile(pwd, '..', 'DRC_SimulinkLibrary'));

%% Load global and turbine constants
loadglobalconstants()
T = loadturbineconstants('IEA15MW');

%% Choose inputs
path_to_CP_LUT     = fullfile(pwd, '..', 'data', 'lookup_tables','def_lookup_table.dat');
path_to_setpoints  = fullfile(pwd, '..', 'data', 'set_points','set_points_peakshaving.dat');
path_to_satlimits  = fullfile(pwd, '..', 'data', 'sat_limits','sat_limits_peakshaving.dat');

%% Load prior information on rotor performance (Cp) for WSE and LUT for FF and saturation limits

flag_create_new_LUT_for_WSE = false;
flag_create_new_set_points  = true;
flag_create_new_sat_limits  = true;

if flag_create_new_LUT_for_WSE == true
    disp(' ');
    disp('- Creating LUT for WSE -');
    tablesPathWSE = create_LUT_for_WSE(path_to_CP_LUT, fullfile(pwd), 'LUT_for_WSE.mat');
    disp(['LUT for WSE saved at: ', tablesPathWSE]);
else
    tablesPathWSE = fullfile('LUT_for_WSE.mat');
    disp(' ');
    disp(['- LUT for WSE retrieved from: ', tablesPathWSE]);
end

if flag_create_new_set_points == true
    disp(' ');    
    disp('- Creating LUT for set points -');
    tablesPathSP = create_LUT_for_FF(path_to_setpoints, fullfile(pwd), 'LUT_for_setpoints.mat');
    disp(['LUT for set points saved at: ', tablesPathSP]);
else
    tablesPathSP = fullfile('LUT_for_WSE.mat');
    disp(' ');
    disp(['- LUT for set points retrieved from: ', tablesPathSP]);
end

if flag_create_new_sat_limits == true
    disp(' ');
    disp('- Creating LUT for saturation limits -');
    tablesPathSL = create_LUT_for_sat_limits(path_to_satlimits, fullfile(pwd), 'LUT_for_satlimits.mat');
    disp(['LUT for saturation limits saved at: ', tablesPathSL]);
else
    tablesPathSL = fullfile('LUT_for_satlimits.mat');
    disp(' ');
    disp(['- LUT for saturation limits retrieved from: ', tablesPathSL]);
end

load(tablesPathWSE);
load(tablesPathSP);
load(tablesPathSL);

%% Initial values # MODIFY THIS FOR EACH SIMULATION!

Wr0 = 0.60; %  Initial value for the rotor rotational speed [rad/s]
U0  = 3;    %  Initial value for the horizontal wind speed [m/s]

T.Cp           = Tables.Cp; T.Cp(T.Cp <= 0.0001) = 0.0001;
T.CPitch       = Tables.Pitch;
T.CWindSpeed   = Tables.wndSpd;
T.COmega       = Tables.rotSpd * rpm2rads;

T.WindSpeedFF  = LUT_FF.ws_FF;
T.GenTorqueFF  = LUT_FF.torque_FF;
T.PitchFF      = LUT_FF.pitch_FF;
T.OmegaFB      = LUT_FF.omega_FF;

T.WindSpeedSatMap = sat_lim.ws_sat_lim;
T.PitchSatMap     = sat_lim.pitch_sat_lim;

% Define Simulink model
simmdl = 'DISCON_FFFB_IEA15MW.slx';

% Simulation parameters
Control.DT = 0.01; 
Ts = Control.DT;
TsLog = Ts*100;
TMax = 9999;

% Open the system
open_system(simmdl)
