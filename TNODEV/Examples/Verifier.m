% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 24 October 2025
% Last revision: October 2025

%------------- BEGIN CODE --------------

% clear all; close all; clc;
clear all; clc;



%%% ========================================================================
%   Verifier.m - (ADD the dnames of different things done based on verifiy.m)
%   Systems: Spiral_non (2D) | FPA (5D)
%   Tools:   CORA (tube bounds) | TIRA (NODE_RA over-approximations)
%%% =========================================================================

addpath('../OA_methods')
addpath('../Utilities')

%%%  -------------------------------------------------------------------
%  USER CONFIGURATION - EDIT ONLY THIS BLOCK
%%%  -------------------------------------------------------------------

% === 1. SYSTEM SELECTION ===
system_choice = 14;   % FPA System
% system_choice = 15;   % Spiral_non System


% === 2. INITIAL SET ===
% Spiral_non (2D)
x0_spiral     = [2.0; 0.0];            % nODE input set center
radius_spiral = 0.2;                   % Initial radius

% FPA (5D)
x0_fpa     = [0, -0.58587, 0.8, 0.52323, 0.7]';
radius_fpa = 0.01;

% Auto-select based on system_choice
if system_choice == 15
    x0             = x0_spiral;
    Initial_radius = radius_spiral;
    system_name    = 'spiral_non';
elseif system_choice == 14
    x0             = x0_fpa;
    Initial_radius = radius_fpa;
    system_name    = 'fpa';
else
    error('Invalid system_choice. Use 14 (FPA) or 15 (Spiral_non).');
end


% === 3. CORA PARAMETERS (for reachable tube bounds) ===
cora_params.timeStep     = 0.01;   % CORA time step for discretization
cora_params.tFinal       = 2.0;    % CORA Final time (computes full R_tube up to t=2)
cora_params.taylorTerms  = 2;      % Taylor terms for nonlinear approximation  
cora_params.zonotopeOrder = 20;    % Zonotope order for set representation
cora_params.alg          = 'lin';  % Linear approximation
cora_params.tensorOrder  = 2;      % Tensor order for nonlinear dynamics


% === 4. TIRA PARAMETERS (for nODE reachability @NODE_RA.m) ===
t_init_TIRA   = 0;
t_final_TIRA  = 1.0;      % Must match first code → use 1.0
timeStep_TIRA = 0.01;     % Must match first code → use 0.01
node_params.plot_last_only     = true;
node_params.sample_succ_number = 1000;


% === 5. SAFE SET (USER-ADJUSTABLE) ===

%%%%%%%%%%%%% Spiral_non System %%%%%%%%%%%%% 
% Example 1: Safety verified for all OA methods
% node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)

% Example 2: Safety violated for some methods
% node_params.safe_inf = [-1.3; 0.95];   % [x1_low, x2_low]
% node_params.safe_sup = [0.25; 2.45]; % [x1_up,  x2_up]


%%%%%%%%%%%%% FPA System %%%%%%%%%%%%% 
% Example 1: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% node_params.safe_inf = [-1.5; -1];   % [x1_low, x2_low]
% node_params.safe_sup = [0.5; 3]; % [x1_up,  x2_up]

% Example 2: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% node_params.safe_inf = [-1.4; -0.8];   % [x1_low, x2_low]
% node_params.safe_sup = [0.38; 2.55]; % [x1_up,  x2_up]

% Example 3: Safety violated for some methods
node_params.safe_inf = [-1; 0.2];   % [x1_low, x2_low]
node_params.safe_sup = [-0.5; 0.5]; % [x1_up,  x2_up]

% === 6. METHOD NAMES (for output clarity) ===
method_names = {...
    'Mixed-Monotonicity (Single-step)', ...
    'Sampled-Data Mixed-Monotonicity (Single-step)', ...
    'Mixed-Monotonicity (Boundary)', ...
    'Sampled-Data Mixed-Monotonicity (Boundary)'};

% === 7. PLOTTING SELECTOR  === --> [To-Do]
% % Choose which figures to show and save from NODE_RA.m
% plot_config.show_reachability     = true;   % OAs + safe set illustration
% plot_config.show_boundary_points  = true;   % Splitted vertices
% plot_config.show_interval_hull    = true;   % Union of splits
% plot_config.save_figures          = true;   % Save to results_*/ folder

% =========================================================================
fprintf('=== Starting Verification Analysis ===\n');
fprintf('System: %s (choice=%d)\n', system_name, system_choice);
fprintf('Initial set: center = [%s], radius = %.3f\n', ...
        strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
        Initial_radius);
fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);


%% ------------------------------------------------------------------------
%   PART 1: CORA - Compute Reachable Tube Bounds
% -------------------------------------------------------------------------
fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');

% Build full CORA params: CORA settings + initial set
cora_full_params = cora_params;
cora_full_params.x0             = x0;             
cora_full_params.Initial_radius = Initial_radius;  

if system_choice == 15
    reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
elseif system_choice == 14
    reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
end

fprintf('PART 1 completed.\n\n');

%% ------------------------------------------------------------------------
%   PART 2: Compute Jacobian Bounds based on CORA Reachable Tube
% -------------------------------------------------------------------------
fprintf('PART 2: Computing Jacobian Bounds...\n');
a = reachable_tube_bounds.lb;
b = reachable_tube_bounds.ub;

[J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);  % ← Pass system_choice!
% [J_lb, J_ub] = Jacob_bounds_element_wise(a, b);
fprintf('PART 2 completed.\n\n');


%% ------------------------------------------------------------------------
%   PART 3: Pass Jacobians to UP_Jacobian_Bounds.m
% -------------------------------------------------------------------------
fprintf('PART 3: Passing Jacobian bounds via globals...\n');

global Jacobian_x_low Jacobian_x_up system_choice_global
Jacobian_x_low = J_lb;
Jacobian_x_up  = J_ub;
system_choice_global = system_choice;

[Jx_low, Jx_up, ~, ~] = UP_Jacobian_Bounds(...
    t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);

fprintf('PART 3 completed.\n\n');

% fprintf('UP_Jacobian_Bounds returned:\n');
fprintf('Jx_low =\n'); disp(Jx_low);
fprintf('Jx_up  =\n'); disp(Jx_up);


%% ------------------------------------------------------------------------
%   PART 4: Call nODE Reachability Analysis function (NODE_RA.m)
% -------------------------------------------------------------------------
fprintf('PART 4: Calling NODE_RA.m reachability analysis...\n');

% Build NODE_RA input
node_params.system_choice  = system_choice;
node_params.x0             = x0;
node_params.Initial_radius = Initial_radius;
node_params.t_final        = t_final_TIRA;
node_params.timeStep       = timeStep_TIRA;

% Call NODE_RA
reachability_results = NODE_RA(node_params);

clear global Jacobian_x_low Jacobian_x_up system_choice_global
fprintf('PART 4 completed.\n\n');


%% ----------------------------------------------------------------------------------------
%   PART 5: Safety Verification (Checking if the computed OA interval ⊆ safe-set interval)
% -----------------------------------------------------------------------------------------
fprintf('PART 5: Safety verification using safe set...\n');

% Load safe set
if isfield(reachability_results, 'safe_inf') && isfield(reachability_results, 'safe_sup')
    saferegion = interval(reachability_results.safe_inf, reachability_results.safe_sup);
    fprintf('  Safe set loaded: [%.2f, %.2f] × [%.2f, %.2f]\n', ...
            saferegion.inf(1), saferegion.sup(1), saferegion.inf(2), saferegion.sup(2));
else
    error('NODE_RA did not return safe_inf/safe_sup.');
end

% Helper
interval_is_subset = @(A,B) all(A.inf >= B.inf) && all(A.sup <= B.sup);

% Final time index
k = size(reachability_results.succ_low_all_methods, 2);
plot_pairs = reachability_results.plot_pairs;

% Safety flags
safe_flags = true(1,4);

% Print method header
if size(plot_pairs, 1) > 0
    fprintf('  Method: %s\n', strjoin(method_names, ' | '));
end

% Check each projection
for p = 1:size(plot_pairs,1)
    idx1 = plot_pairs(p,1); 
    idx2 = plot_pairs(p,2);

    % 1. Single-step MM
    I1 = interval(reachability_results.succ_low_all_methods([idx1 idx2], k, 1), ...
                  reachability_results.succ_up_all_methods([idx1 idx2], k, 1));

    % 2. Sampled-Data MM
    I2 = interval(reachability_results.succ_low_all_methods([idx1 idx2], k, 2), ...
                  reachability_results.succ_up_all_methods([idx1 idx2], k, 2));

    % 3. Boundary MM
    xb1_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx1,k,1), 1:reachability_results.num_splits);
    yb1_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx2,k,1), 1:reachability_results.num_splits);
    xb1_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx1,k,1),  1:reachability_results.num_splits);
    yb1_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx2,k,1),  1:reachability_results.num_splits);
    I3 = interval([min(xb1_low); min(yb1_low)], [max(xb1_up); max(yb1_up)]);

    % 4. Boundary Sampled-Data
    xb2_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx1,k,2), 1:reachability_results.num_splits);
    yb2_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(idx2,k,2), 1:reachability_results.num_splits);
    xb2_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx1,k,2),  1:reachability_results.num_splits);
    yb2_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(idx2,k,2),  1:reachability_results.num_splits);
    I4 = interval([min(xb2_low); min(yb2_low)], [max(xb2_up); max(yb2_up)]);

    % Safety checks
    checks = [interval_is_subset(I1,saferegion), ...
              interval_is_subset(I2,saferegion), ...
              interval_is_subset(I3,saferegion), ...
              interval_is_subset(I4,saferegion)];
    safe_flags = safe_flags & checks;

    % Print result
    status = {'UNSAFE', 'SAFE'};
    fprintf('  x%d vs x%d: %s | %s | %s | %s\n', idx1, idx2, ...
            status{checks(1)+1}, status{checks(2)+1}, ...
            status{checks(3)+1}, status{checks(4)+1});
end

% Final verdict
if all(safe_flags)
    fprintf('\nALL OVER-APPROXIMATION METHODS ARE SAFE!\n');
else
    failed = method_names(~safe_flags);
    fprintf('\nSAFETY VIOLATION: Failed methods include %s\n', strjoin(failed, ', '));
end

fprintf('PART 5 completed.\n\n');


%% ------------------------------------------------------------------------
%   FINAL SUMMARY
% -------------------------------------------------------------------------
fprintf('=== Verification Analysis Complete ===\n');
fprintf('System: %s\n', upper(system_name));
fprintf('PART 1: CORA tube bounds computed\n');
fprintf('PART 2: Jacobian bounds derived\n');
fprintf('PART 3: Jacobians passed to UP_Jacobian_Bounds.m\n');
fprintf('PART 4: TIRA/NODE_RA reachability completed\n');
fprintf('PART 5: Safety verified against user-defined safe set\n');


%------------- END OF CODE --------------