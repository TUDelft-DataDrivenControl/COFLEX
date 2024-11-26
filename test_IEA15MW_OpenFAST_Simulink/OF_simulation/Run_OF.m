% make sure the OpenFAST directory where the FAST_SFunc.mex* file is located
% is in the MATLAB path (also make sure any other OpenFAST library files that
% are needed are on the MATLAB path)
addpath('C:\downloaded_codes\openfast-3.5.0')
addpath(genpath('C:\cloned_repositories\matlab-openfast'))


addpath(fullfile(pwd, '..','COFLEX','DRC_SimulinkLibrary'));
addpath(fullfile(pwd, '..','COFLEX','controller'));

loadglobalconstants();
T = loadturbineconstants('IEA15MW');

%% Choose inputs
path_to_CP_LUT     = fullfile(pwd, '..','COFLEX', 'data', 'lookup_tables','def_lookup_table.dat');
path_to_setpoints  = fullfile(pwd, '..','COFLEX', 'data', 'set_points','set_points_peakshaving.dat');
path_to_satlimits  = fullfile(pwd, '..','COFLEX', 'data', 'sat_limits','sat_limits_peakshaving.dat');

%% Load prior information on rotor performance (Cp) for WSE and LUT for FF and saturation limits

flag_create_new_LUT_for_WSE = false;
flag_create_new_set_points  = false;
flag_create_new_sat_limits  = false;

if flag_create_new_LUT_for_WSE == true
    disp(' ');
    disp('- Creating LUT for WSE -');
    tablesPathWSE = create_LUT_for_WSE(path_to_CP_LUT, fullfile(pwd), 'LUT_for_WSE.mat');
    disp(['LUT for WSE saved at: ', tablesPathWSE]);
else
    tablesPathWSE = fullfile(pwd, '..','COFLEX','controller','LUT_for_WSE.mat');
    disp(' ');
    disp(['- LUT for WSE retrieved from: ', tablesPathWSE]);
end

if flag_create_new_set_points == true
    disp(' ');    
    disp('- Creating LUT for set points -');
    tablesPathSP = create_LUT_for_FF(path_to_setpoints, fullfile(pwd), 'LUT_for_setpoints.mat');
    disp(['LUT for set points saved at: ', tablesPathSP]);
else
    tablesPathSP = fullfile(pwd, '..','COFLEX','controller','LUT_for_setpoints.mat');
    disp(' ');
    disp(['- LUT for set points retrieved from: ', tablesPathSP]);
end

if flag_create_new_sat_limits == true
    disp(' ');
    disp('- Creating LUT for saturation limits -');
    tablesPathSL = create_LUT_for_sat_limits(path_to_satlimits, fullfile(pwd), 'LUT_for_satlimits.mat');
    disp(['LUT for saturation limits saved at: ', tablesPathSL]);
else
    tablesPathSL = fullfile(pwd, '..','COFLEX','controller','LUT_for_satlimits.mat');
    disp(' ');
    disp(['- LUT for saturation limits retrieved from: ', tablesPathSL]);
end

load(tablesPathWSE);
load(tablesPathSP);
load(tablesPathSL);

%% Initial values # MODIFY THIS FOR EACH SIMULATION!

Wr0 = 0.5236;   %  Initial value for the rotor rotational speed [rad/s]
U0  = 3;        %  Initial value for the horizontal wind speed [m/s]

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


% Simulation parameters
Control.DT = 0.001; 
Ts = Control.DT;
TsLog = Ts*10;

% these variables are defined in the OpenLoop model's FAST_SFunc block:
FAST_InputFileName = 'C:\Users\glazzerini\OneDrive - Delft University of Technology\Bureaublad\work\models\coflex_repo\test_IEA15MW_OpenFAST_Simulink\OF_simulation\IEA-15-240-RWT-Onshore\IEA-15-240-RWT-Onshore.fst';
TMax               = 10; % seconds

sim('ClosedLoop.mdl',[0,TMax]);