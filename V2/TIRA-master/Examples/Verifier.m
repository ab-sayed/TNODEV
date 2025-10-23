% Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 23 October 2025
% Last revision: October 2025

%------------- BEGIN CODE --------------

clear all; close all; clc;

addpath('../OA_methods')
addpath('../Utilities')

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

% ---- 3. Show results
fprintf('UP_Jacobian_Bounds returned:\n');
fprintf('Jx_low =\n'); disp(Jx_low);
fprintf('Jx_up  =\n'); disp(Jx_up);


fprintf('\nPART 3 completed successfully.\n');

%% PART 4: Call the nODE reachability analysis function NODE_RA 

fprintf('\nPART 4: Calling NODE_RA reachability analysis...\n');

% Build input struct
node_params = struct();
node_params.system_choice      = system_choice;
node_params.x0                 = params.x0;
node_params.Initial_radius     = params.Initial_radius;


node_params.t_final            = 1.0;
node_params.timeStep           = 0.01;

node_params.plot_last_only     = true;
node_params.sample_succ_number = 1000;

% <<< USER-ADJUSTABLE SAFE SET (CHANGE HERE) >>>

% node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)

% Safety verified X_s for all OA methods
% node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)

% Safety violated X_s for some OA methods
node_params.safe_inf = [-1.3; 0.95];   % lower-left corner (x1, x2)
node_params.safe_sup = [0.25; 2.45];   % upper-right corner (x1, x2)
% ---------------------------------------------------------------

% Pass Jacobian bounds globally
global Jacobian_x_low Jacobian_x_up
Jacobian_x_low = J_lb;
Jacobian_x_up  = J_ub;

% CALL FUNCTION
reachability_results = NODE_RA(node_params);

clear global Jacobian_x_low Jacobian_x_up
fprintf('PART 4 completed.\n');

%% PART 5: Safety verification and safe-set plotting 


fprintf('\nPART 5: Safety verification\n');

% ------------------------------------------------------------------
% Retrieve safe region from NODE_RA.m (passed via node_params)
% ------------------------------------------------------------------
if isfield(reachability_results, 'safe_inf') && isfield(reachability_results, 'safe_sup')
    saferegion = interval(reachability_results.safe_inf, reachability_results.safe_sup);
    fprintf('  Safe set loaded: [%.2f, %.2f] × [%.2f, %.2f]\n', ...
            saferegion.inf(1), saferegion.sup(1), saferegion.inf(2), saferegion.sup(2));
else
    error('NODE_RA did not return safe_inf/safe_sup. Did you set node_params.safe_inf/sup in Verifier.m?');
end

% ------------------------------------------------------------------
% Helper: interval subset check
% ------------------------------------------------------------------
interval_is_subset = @(A,B) all(A.inf >= B.inf) && all(A.sup <= B.sup);

% ------------------------------------------------------------------
% Final time step
% ------------------------------------------------------------------
k = size(reachability_results.succ_low_all_methods, 2);

% ------------------------------------------------------------------
% Plot pairs
% ------------------------------------------------------------------
plot_pairs = reachability_results.plot_pairs;

% ------------------------------------------------------------------
% Method names (aligned with checks)
% ------------------------------------------------------------------
method_names = {...
    'Mixed-Monotonicity (Single-step)', ...
    'Sampled-Data Mixed-Monotonicity (Single-step)', ...
    'Mixed-Monotonicity (Boundary)', ...
    'Sampled-Data Mixed-Monotonicity (Boundary)'};

% ------------------------------------------------------------------
% Safety flags
% ------------------------------------------------------------------
safe_flags = true(1,4);

% Print method names once (only if multiple plot pairs)
if size(plot_pairs, 1) > 0
    fprintf('  Method: %s\n', strjoin(method_names, ' | '));
end

for p = 1:size(plot_pairs,1)
    idx1 = plot_pairs(p,1); 
    idx2 = plot_pairs(p,2);

    %% ---- 1. Single-step Mixed-Monotonicity (method 1) -----------------
    I1 = interval( reachability_results.succ_low_all_methods([idx1 idx2], k, 1), ...
                   reachability_results.succ_up_all_methods([idx1 idx2], k, 1) );

    %% ---- 2. Single-step Sampled-Data MM (method 2) -------------------
    I2 = interval( reachability_results.succ_low_all_methods([idx1 idx2], k, 2), ...
                   reachability_results.succ_up_all_methods([idx1 idx2], k, 2) );

    %% ---- 3. Boundary Mixed-Monotonicity (method 1) -------------------
    xb1_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx1,k,1), 1:reachability_results.num_splits);
    yb1_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx2,k,1), 1:reachability_results.num_splits);
    xb1_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx1,k,1),  1:reachability_results.num_splits);
    yb1_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx2,k,1),  1:reachability_results.num_splits);
    I3 = interval([min(xb1_low); min(yb1_low)], [max(xb1_up); max(yb1_up)]);

    %% ---- 4. Boundary Sampled-Data MM (method 2) ----------------------
    xb2_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx1,k,2), 1:reachability_results.num_splits);
    yb2_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx2,k,2), 1:reachability_results.num_splits);
    xb2_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx1,k,2),  1:reachability_results.num_splits);
    yb2_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx2,k,2),  1:reachability_results.num_splits);
    I4 = interval([min(xb2_low); min(yb2_low)], [max(xb2_up); max(yb2_up)]);

    %% ---- Safety checks ------------------------------------------------
    checks = [ interval_is_subset(I1,saferegion), ...
               interval_is_subset(I2,saferegion), ...
               interval_is_subset(I3,saferegion), ...
               interval_is_subset(I4,saferegion) ];
    safe_flags = safe_flags & checks;

    % Pretty printing
    status = {'UNSAFE','SAFE'};
    fprintf('  x%d vs x%d: %s | %s | %s | %s\n', idx1, idx2, ...
            status{checks(1)+1}, status{checks(2)+1}, ...
            status{checks(3)+1}, status{checks(4)+1});
end

%% ---- Final verdict ----------------------------------------------------
if all(safe_flags)
    fprintf('\nALL OVER-APPROXIMATIONS METHODS ARE SAFE!\n');
else
    failed = method_names(~safe_flags);
    fprintf('\nSAFETY VIOLATION DETECTED for these Over-approximation methods: %s\n', strjoin(failed, ', '));
end

fprintf('PART 5 completed.\n');



%% Final Prints and Checks

% % Clean up
clear global Jacobian_x_low Jacobian_x_up


fprintf('\n=== Verification Analysis Complete ===\n');
fprintf('PART 1: Overall reachable tube bounds computed\n');
fprintf('PART 2: Jacobian bounds computed for overall tube bounds\n');
fprintf('PART 3: Jacobian bounds passed to UP_Jacobian_Bounds.m\n');
fprintf('PART 4: Call the nODE reachability analysis function NODE_RA.m\n');
fprintf('PART 5: Safety verification and safe-set plotting using safe set returned by NODE_RA\n');


%------------- END OF CODE --------------