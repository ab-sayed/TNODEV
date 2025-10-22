% Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 21 October 2025
% Last revision: October 2025

%------------- BEGIN CODE --------------

clear all; close all; clc;

%%%  -------------------------------------------------------------------
%  USER CONFIGURATION for spiral – change only the values in this block
%%%  -------------------------------------------------------------------

% ---------- CORA reachability parameters for spiral_non System----------
params.timeStep = 0.01;              % Time step for discretization
params.tFinal = 2;                   % Final time
params.taylorTerms = 2;              % Taylor terms for nonlinear approximation  
params.zonotopeOrder = 20;           % Zonotope order for set representation
params.alg = 'lin';                  % Linear approximation method ('lin' or 'con')
params.tensorOrder = 2;              % Tensor order for nonlinear dynamics

% ---------- Initial set parameters ----------
params.x0 = [2.0; 0.0];              % nODE input set center
params.Initial_radius = 0.2;         % Initial radius

% ---------- System choice for UP_Jacobian_Bounds ----------
system_choice = 15;   % Spiral_non System
% system_choice = 14;   % FPA System

fprintf('=== Starting Verification Analysis ===\n\n');
fprintf('system_choice = %d\n',system_choice);

%% PART 1: CORA computation of Reachable Tube Bounds
fprintf('PART 1: Computing Overall Reachable Tube Bounds...\n');
reachable_tube_bounds = spiral_non_CORA_RT(params);

fprintf('\nPART 1 completed successfully.\n');
% fprintf('Overall reachable tube bounds:\n');
% fprintf('  Lower bounds: [%s]\n', num2str(reachable_tube_bounds.lb', '%.4f '));
% fprintf('  Upper bounds: [%s]\n', num2str(reachable_tube_bounds.ub', '%.4f '));

%% PART 2: Jacobian Bounds Computation
fprintf('\nPART 2: Computing Jacobian Bounds...\n');
a = reachable_tube_bounds.lb;  % Lower bounds from overall tube
b = reachable_tube_bounds.ub;  % Upper bounds from overall tube

[J_lb, J_ub] = Jacob_bounds_element_wise(a, b);

fprintf('\nPART 2 completed successfully.\n');

%% PART 3: Pass the computed Jacobians to UP_Jacobian_Bounds.m
% The function expects: t_init , t_final , x_low , x_up , p_low , p_up

fprintf('\nPART 3: Passing Jacobian bounds via globals (6-inputs)...\n');

% ---- 1. Store the Jacobians in globals (UP_Jacobian_Bounds.m will read them)
global Jacobian_x_low Jacobian_x_up   % will hold ∂f/∂x bounds
Jacobian_x_low = J_lb;                
Jacobian_x_up  = J_ub;

% ---- 2. Call the *documented* 6-inputs
t_init = 0;
t_final = params.tFinal;
x_low  = J_lb;   % state interval (for size only)
x_up   = J_ub;
p_low  = [];                         % autonomous → no parameters
p_up   = [];

% system_choice is still needed inside UP_Jacobian_Bounds and MAIN_CALL.m
global system_choice                 % Let MAIN_CALL.m read system choice
system_choice = system_choice;       % keep the value chosen at the top


[Jx_low,Jx_up,Jp_low,Jp_up] = UP_Jacobian_Bounds(t_init,t_final,...
                                                x_low,x_up,p_low,p_up);  % 6 NEW INPUTS

% ---- 3. (optional) clear globals after use
% clear global Jacobian_x_low Jacobian_x_up

% ---- 4. Show results
fprintf('UP_Jacobian_Bounds returned:\n');
fprintf('Jx_low =\n'); disp(Jx_low);
fprintf('Jx_up  =\n'); disp(Jx_up);
fprintf('Jp_low =\n'); disp(Jp_low);
fprintf('Jp_up  =\n'); disp(Jp_up);

fprintf('\nPART 3 completed successfully.\n');

%% PART 4: Call the nODE reachability analysis function NODE_RA 

fprintf('\nPART 4: Calling the nODE reachability anaylsis methods(single-step and Boundary)...\n');

% % I should add/pass the parameters from here 
% % Build input structure for NODE_RA
% node_params = struct();
% node_params.system_choice      = system_choice;
% node_params.x0                 = params.x0;
% node_params.Initial_radius     = params.Initial_radius;
% node_params.t_final            = params.tFinal;
% node_params.timeStep           = params.timeStep;
% node_params.plot_last_only     = true;           % or false
% node_params.sample_succ_number = 1000;           % optional
% 
% % Set Jacobian bounds (used by UP_Jacobian_Bounds inside TIRA)
% global Jacobian_x_low Jacobian_x_up
% Jacobian_x_low = J_lb;
% Jacobian_x_up  = J_ub;
% 
% % Call NODE_RA
% results = NODE_RA(node_params);

NODE_RA;  % <--- THIS IS THE ONLY CALL

fprintf('\nPART 4 completed successfully.\n');

%% Final Prints and Checks

% % Clean up
clear global Jacobian_x_low Jacobian_x_up


fprintf('\n=== Verification Analysis Complete ===\n');
fprintf('PART 1: Overall reachable tube bounds computed\n');
fprintf('PART 2: Jacobian bounds computed for overall tube bounds\n');
fprintf('PART 3: Jacobian bounds passed to UP_Jacobian_Bounds.m\n');


%------------- END OF CODE --------------