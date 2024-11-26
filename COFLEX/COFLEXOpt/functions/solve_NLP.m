%% Use this function to find optimal control trajectories based on CP - CT surfaces calculated with hawcstab2
function res = solve_NLP(turbine, opt_weights, wndspd_fix, x_constraints, g_constraints, tsr_fix,verbose, res_prev)
% Main function to find optimal control trajectories for a wind turbine.

import casadi.*

% Named symbolic variables
RotSpd   = MX.sym('RotSpd');
Pitch    = MX.sym('Pitch');

% Tolerance on initial value for pitch (to avoid division by 0)
eps = 0.01;

% Check if optional argument is provided
if nargin > 7
    % Use the provided optional argument
    optstruct_prev = res_prev;
else
    % Provide a default value if the optional argument is not provided
    num_constraints = numel(fieldnames(g_constraints));
    default_g       = zeros(num_constraints, 1);
    default_lam_g   = zeros(num_constraints, 1);
    optstruct_prev  = struct('f', 0, 'g', default_g, 'lam_g', default_lam_g, 'lam_p', [0; 0], 'lam_x', [0; 0], 'x', [turbine.wr_min; turbine.beta_fine + eps]);
end


if tsr_fix.flag == true
    if (wndspd_fix >= tsr_fix.ws_min) && (wndspd_fix<=tsr_fix.ws_max)
        rotLowLim   = (tsr_fix.tsr*wndspd_fix)/turbine.R;                   % Rotor rot. (rad/s)
        rotUpLim    = (tsr_fix.tsr*wndspd_fix)/turbine.R;                   % Rotor rot. (rad/s)
        pitchLowLim = x_constraints.Pitch(1,1);                    % Pitch (rad)
        pitchUpLim  = x_constraints.Pitch(2,1);                    % Pitch (rad)
    else
        if (wndspd_fix<=tsr_fix.min_ws_cap)
            % Check decision variables constraints limits
            rotLowLim   = x_constraints.RotSpd(1,1);                   % Rotor rot. (rad/s)
            rotUpLim    = x_constraints.RotSpd(1,1);                   % Rotor rot. (rad/s)
            pitchLowLim = x_constraints.Pitch(1,1);                    % Pitch (rad)
            pitchUpLim  = x_constraints.Pitch(2,1);                    % Pitch (rad)
        else
            % Check decision variables constraints limits
            rotLowLim   = x_constraints.RotSpd(1,1);                   % Rotor rot. (rad/s)
            rotUpLim    = x_constraints.RotSpd(2,1);                   % Rotor rot. (rad/s)
            pitchLowLim = x_constraints.Pitch(1,1);                    % Pitch (rad)
            pitchUpLim  = x_constraints.Pitch(2,1);                    % Pitch (rad)
        end
    end
else
    % Check decision variables constraints limits
    rotLowLim   = x_constraints.RotSpd(1,1);                   % Rotor rot. (rad/s)
    rotUpLim    = x_constraints.RotSpd(2,1);                   % Rotor rot. (rad/s)
    pitchLowLim = x_constraints.Pitch(1,1);                    % Pitch (rad)
    pitchUpLim  = x_constraints.Pitch(2,1);                    % Pitch (rad)
end


Cp            = turbine.Cp([RotSpd,wndspd_fix,Pitch]);
Ct            = turbine.Ct([RotSpd,wndspd_fix,Pitch]);

%% Optimization weights and objective function definition

CP_weight            = opt_weights(1);
CQ_weight            = opt_weights(2);       
CT_weight            = opt_weights(3);       

CP_f = Cp;
CT_f = Ct;
CQ_f = Cp/(RotSpd*turbine.R)*wndspd_fix;

f  = CP_weight*CP_f + CT_weight*CT_f + CQ_weight*CQ_f;

%% Create constraints

g          = create_function_constraints(g_constraints,turbine,RotSpd,wndspd_fix,Pitch);
[lbg,ubg]  = create_boundaries_constraints(g_constraints);

%% Create NLP with options

nlp   = struct;                                    % NLP declaration
nlp.x = [RotSpd;Pitch];                            % decision vars
nlp.f = f;                                         % objective
nlp.g = g;                                         % constraints

options                                 = struct;
options.ipopt.max_iter                  = 3000;
options.ipopt.warm_start_init_point     = 'yes';

if verbose == false
    options.verbose                         = verbose;
    options.print_time                      = verbose;
    options.ipopt.print_level               = verbose;
    options.ipopt.sb                        = 'yes';
else
end

% Create solver instance
F = nlpsol('F','ipopt',nlp,options);

%% Solve the problem using a guess

r = F('x0',optstruct_prev.x,'lam_g0',optstruct_prev.lam_g,'lam_x0',optstruct_prev.lam_x,'lbx',[rotLowLim pitchLowLim],'ubx',[rotUpLim pitchUpLim],'lbg',lbg,'ubg',ubg);

%% Display solution

x_opt = r.x;
res = r;

%disp(F)
%disp(x_opt)

end


function g_function = create_function_constraints(g_constraints,turbine,RotSpd,wndspd_fix,Pitch)

%% Handling constraints
constraints = struct();

% Iterate over fields in g_constraints
fields = fieldnames(g_constraints);
for i = 1:numel(fields)
    field = fields{i};
    
    % Extract the corresponding variable from turbine
    variable = turbine.(field)([RotSpd,wndspd_fix,Pitch]);  % Assuming the field in turbine has the same name as in g_constraints
    
    % Assign to constraints structure
    constraints.(field) = variable;
end

% Initialize empty array for g
g = [];

% Iterate over fields in constraints
fields = fieldnames(constraints);
for i = 1:numel(fields)
    field = fields{i};
    
    % Extract the variable value from constraints
    variable_value = constraints.(field);
    
    % Add the variable value to g
    g = [g; variable_value];
end

g_function = g;

end

function [low_bound_g,up_bound_g] = create_boundaries_constraints(g_constraints)

% Initialize empty arrays for lower and upper bounds
lbg = [];
ubg = [];

% Iterate over fields in g_constraints
fields = fieldnames(g_constraints);

for i = 1:numel(fields)
    field = fields{i};

    % Extract lower and upper bounds from g_constraints
    lb = g_constraints.(field)(1);
    ub = g_constraints.(field)(2);
    
    % Append lower and upper bounds to lbg and ubg arrays
    lbg = [lbg; lb];
    ubg = [ubg; ub];
end

low_bound_g = lbg;
up_bound_g  = ubg;

end