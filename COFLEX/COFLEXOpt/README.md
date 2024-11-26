# COFLEXOpt: the Set Point Optimiser

The **COFLEXOpt** module determines the optimal operating points for rotational speed, collective pitch angle and generator torque $(\omega^*, \beta^*_{\text{FF}}, Q_{\text{g,FF}})$ for a large, flexible wind turbine by following this workflow, as detailed below.

## Folder Content
- `main_coflexopt.m`: MATLAB script to run the set point optimiser.
- `functions\`: functions used by COFLEXOpt.
 - `README.md`: The file you are reading.

## COFLEXOpt Workflow
### Step 1: Input Steady-State Performance
Open `main_coflexopt.m` and define the right paths to load functions and external inputs.  
Modify the following block, (and possibly the function `loadturbineconstants.m`) to use your turbine characteristics:
 ```matlab
%% Load turbine characteristics
fprintf('Loading turbine characteristics...\n');
turbine_name = 'IEA15MW';
path_to_data = fullfile(pwd, '..', 'data', 'inputs','pwr_tables_FLEX.nc'); % Path to NetCDF dataset from HAWCStab2
turbine = loadturbineconstants(turbine_name, path_to_data, true);          % Load turbine specs and performance characteristics
```

You will have to use precomputed steady-state performance metrics evaluated over a fine grid defined by rotor speed $(\omega)$, wind speed $(V)$, and blade pitch angle $(\beta)$:

$(\omega, V, \beta) \in [\omega_{\text{min}}, \omega_{\text{max}}] \times [V_{\text{min}}, V_{\text{max}}] \times [\beta_{\text{min}}, \beta_{\text{max}}]
$

such as:

- **Aerodynamic performance**:  
  - Power coefficient $C_P(\omega, V, \beta)$  
  - Thrust coefficient $C_T(\omega, V, \beta)$  
  - Torque coefficient $C_Q(\omega, V, \beta)$  

- **Structural-loads properties**:  
  - Out-of-plane blade tip displacement $\text{OoP tip disp.}(\omega, V, \beta)$
  - Blade root moments
  - etc...

This step relies on prior steady-state aerodynamic or aeroelastic simulations. We used **HAWCStab2**.  
The function `loadturbineconstants.m` uses the precomputed steady-state data to create smooth, continuous, and differentiable **B-spline interpolators**. These interpolators are essential for formulating the optimisation problem.
### Step 2. Choose between Set Points or Saturation Limits:
To calculate the saturation limits for the collective pitch angle, set this flag to `true`
```matlab
%% Define saturation limits instead of set points
flag_sat_limits = false;
```
### Step 3. Formulate Set Point Optimisation as a Nonlinear Programming (NLP) Problem

Define the optimisation problem to **maximise the power coefficient** $(C_P)$ while incorporating a weighted penalty for generator torque $(C_Q)$:

**Subject to arbitrary constraints**:
  - Maximum thrust  
  - Torque limits  
  - Blade tip deflection  
  - etc...  
These constraints must have the **same variable name** as in the turbine struct that was loaded by `loadturbineconstants.m`, as shown in this block of code:
```matlab
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
```

### Step 4. Solve the NLP to Obtain Optimised Set Points

Solve the optimisation problem for each wind speed $(\overline{V})$ over the full wind speed operating range $V_{\text{cut-in}} \ \text{to} \ V_{\text{cut-out}}$.

**Output for each wind speed**:
- Optimal rotor speed set point - $\omega^*(\overline{V})$
- Optimal collective pitch angle set point - $\beta^*_{\text{FF}}(\overline{V})$  
- Generator torque set point - $Q_{\text{g,FF}}(\overline{V})$
- **Saturation limit** for the collective pitch angle - $j (\overline{V})$

These outputs are stored and gathered for the entire operating region in **lookup tables** for direct use in the **COFLEX controller**.

