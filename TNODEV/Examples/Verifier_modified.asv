% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 25 October 2025 (Adapted for iterative verification/falsification/refinement)
% Last revision: October 2025

%------------- BEGIN CODE --------------

clear all; close all; clc;

%%% ========================================================================
%   Verifier.m - Iterative Verification, Falsification, and Refinement Loop
%   Systems: Spiral_non (2D) | FPA (5D)
%   Tools:   CORA (tube bounds) | TIRA (NODE_RA over-approximations)
%%% =========================================================================

addpath('../OA_methods')
addpath('../Utilities')
addpath('./') % Add current directory for utility functions

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

% Example 3: Safety violated for some methods
% node_params.safe_inf = [-1; 1.2];   % [x1_low, x2_low]
% node_params.safe_sup = [0.3; 2]; % [x1_up,  x2_up]


%%%%%%%%%%%%% FPA System %%%%%%%%%%%%% 
% Example 1: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% node_params.safe_inf = [-1.5; -1];   % [x1_low, x2_low]
% node_params.safe_sup = [0.5; 3]; % [x1_up,  x2_up]

% Example 2: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% node_params.safe_inf = [-1.4; -0.8];   % [x1_low, x2_low]
% node_params.safe_sup = [0.38; 2.55]; % [x1_up,  x2_up]

% FPA Example 3: Safety violated for some methods
% node_params.safe_inf = [-1; 0.2];   % [x1_low, x2_low]
% node_params.safe_sup = [-0.5; 0.5]; % [x1_up,  x2_up]


% FPA Example 4: Safety violated for some methods
node_params.safe_inf = [-1.6; 0.9];   % [x1_low, x2_low]
node_params.safe_sup = [0.5; 2]; % [x1_up,  x2_up]


% NOTE: The safe set is defined only for the first two dimensions here.
% The logic below assumes the safe set is defined for the dimensions
% being checked in NODE_RA's plot_pairs.

% === 6. VERIFICATION LOOP PARAMETERS ===
MAX_ITER = 50;
MIN_RADIUS = 1e-4; % Minimum radius for a set to be considered for splitting

% === 7. METHOD NAMES (for output clarity) ===
method_names = {...
    'Mixed-Monotonicity (Single-step)', ...
    'Sampled-Data Mixed-Monotonicity (Single-step)', ...
    'Mixed-Monotonicity (Boundary)', ...
    'Sampled-Data Mixed-Monotonicity (Boundary)'};

% =========================================================================
fprintf('=== Starting Iterative Verification Analysis ===\n');
fprintf('System: %s (choice=%d)\n', system_name, system_choice);
fprintf('Initial set: center = [%s], radius = %.3f\n', ...
        strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
        Initial_radius);
fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);


%% ------------------------------------------------------------------------
%   INITIALIZATION
% -------------------------------------------------------------------------

% Queue of input sets to verify: {x_center, radius}
% Initial set is the first in the queue
verification_queue = {{x0, Initial_radius}};

% Results tracking
num_verified = 0;
num_falsified = 0;
num_unknown = 1;

% Global flag for safety
overall_safe = true;
falsified_example = [];

% Pre-compute CORA bounds once, as they are for the whole input set
fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');
cora_full_params = cora_params;
cora_full_params.x0             = x0;
cora_full_params.Initial_radius = Initial_radius;

if system_choice == 15
    reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
elseif system_choice == 14
    reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
end
fprintf('PART 1 completed.\n\n');

% Pre-compute Jacobian Bounds once, as they depend on the full CORA tube
fprintf('PART 2: Computing Jacobian Bounds...\n');
a = reachable_tube_bounds.lb;
b = reachable_tube_bounds.ub;
[J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
fprintf('PART 2 completed.\n\n');

% Pass Jacobians to UP_Jacobian_Bounds.m (This is typically done once)
fprintf('PART 3: Passing Jacobian bounds via globals...\n');
global Jacobian_x_low Jacobian_x_up system_choice_global
Jacobian_x_low = J_lb;
Jacobian_x_up  = J_ub;
system_choice_global = system_choice;

% NOTE: UP_Jacobian_Bounds is called here but its output is not used
% in the current Verifier.m. The global variables are what matters for NODE_RA.
[~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
fprintf('PART 3 completed.\n\n');


%% ------------------------------------------------------------------------
%   MAIN VERIFICATION LOOP (
%   1. Verification: Uses NODE_RA to compute interval over-approximations of the
%   reachable set at $ t = 1.0 $ using methods like mixed-monotonicity and 
%   sampled-data approaches. Checks if these intervals are contained in the 
%   safe set (safe_inf, safe_sup) for the first two dimensions (or all 
%   dimensions for FPA).  
%   2. Falsification: Checks samples from NODE_RA (samples_final_time) against
%   the safe set to find a counterexample. Relies on sampling rather than
%   adversarial methods --> [I need to do it with adversarial attacks similar to
%   verify.m]
%   3. Refinement: Splits the input interval along the dimension with the 
%   largest radius, creating two new intervals with halved radius in that 
%   dimension.
% -------------------------------------------------------------------------
iter = 0;
while ~isempty(verification_queue) && iter < MAX_ITER && overall_safe

    iter = iter + 1;
    fprintf('--- ITERATION %d: Queue size = %d ---\n', iter, length(verification_queue));

    % Pop the next set from the queue
    current_set = verification_queue{1};
    verification_queue(1) = [];
    x_i = current_set{1};
    r_i = current_set{2};

    % Check if the set is too small to split further
    if max(r_i) < MIN_RADIUS
        fprintf('  Set radius (%.4e) below minimum threshold (%.4e). Skipping refinement.\n', max(r_i), MIN_RADIUS);
        num_unknown = num_unknown + 1;
        continue;
    end

    % ---------------------------------------------------------------------
    % PART 4: Call nODE Reachability Analysis (NODE_RA.m)
    % ---------------------------------------------------------------------
    fprintf('  PART 4: Calling NODE_RA for set with center [%.3f, ...] and radius %.3f\n', x_i(1), r_i);

    % Build NODE_RA input for the current subset
    node_params.system_choice  = system_choice;
    node_params.x0             = x_i;
    node_params.Initial_radius = r_i;
    node_params.t_final        = t_final_TIRA;
    node_params.timeStep       = timeStep_TIRA;

    % Call NODE_RA
    reachability_results = NODE_RA(node_params);

    % Get the final over-approximated interval for all methods (assuming 4 methods)
    k = size(reachability_results.succ_low_all_methods, 2);
    num_methods = size(reachability_results.succ_low_all_methods, 3);
    
    % Get the interval hull of all boundary splits for the boundary methods
    if num_methods >= 3
        % Boundary MM (Method 3)
        xb3_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
        I3_low = min(cat(2, xb3_low{:}), [], 2);
        xb3_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,1),  1:reachability_results.num_splits, 'UniformOutput', false);
        I3_up = max(cat(2, xb3_up{:}), [], 2);
        
        % Boundary Sampled-Data (Method 4)
        xb4_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
        I4_low = min(cat(2, xb4_low{:}), [], 2);
        xb4_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,2),  1:reachability_results.num_splits, 'UniformOutput', false);
        I4_up = max(cat(2, xb4_up{:}), [], 2);
    end
    
    % Aggregate all over-approximations for the safety check
    OA_low = reachability_results.succ_low_all_methods(:, k, :);
    OA_up  = reachability_results.succ_up_all_methods(:, k, :);
    
    % Replace single-step boundary OAs with the hull of all splits
    if num_methods >= 3
        OA_low(:, 1, 3) = I3_low;
        OA_up(:, 1, 3) = I3_up;
    end
    if num_methods >= 4
        OA_low(:, 1, 4) = I4_low;
        OA_up(:, 1, 4) = I4_up;
    end
    
    % ---------------------------------------------------------------------
    % 1. Verification (Check if all OA intervals are fully contained in the safe set)
    % ---------------------------------------------------------------------
    saferegion_inf = node_params.safe_inf;
    saferegion_sup = node_params.safe_sup;
    
    % NOTE: This is a simplification. The safe set is only defined for the
    % first two dimensions in the user's config. We only check those.
    check_dims = size(saferegion_inf, 1);
    
    is_verified = true;
    for m = 1:num_methods
        % Check if OA is subset of safe set for the checked dimensions
        low_check = all(OA_low(1:check_dims, 1, m) >= saferegion_inf);
        up_check = all(OA_up(1:check_dims, 1, m) <= saferegion_sup);
        
        if low_check && up_check
            fprintf('    Method %d (%s) VERIFIED (SAFE).\n', m, method_names{m});
        else
            fprintf('    Method %d (%s) UNKNOWN (violates safe set bounds: low=%d, up=%d).\n', m, method_names{m}, ~low_check, ~up_check);
            is_verified = false;
        end
    end

    if is_verified
        num_verified = num_verified + 1;
        num_unknown = num_unknown - 1;
        fprintf('  Set VERIFIED: All methods are safe for this input set.\n');
        continue; % Move to the next set in the queue
    end

    % ---------------------------------------------------------------------
    % 2. Falsification (Check if any sample violates the safe set)
    % ---------------------------------------------------------------------
    % The user's NODE_RA.m does not return a counterexample, but it samples.
    % We will assume the samples are in reachability_results.samples_final_time
    % and check them against the safe set.
    
    % NOTE: This is a placeholder for a proper falsification check.
    % We use the final time samples from NODE_RA.
    if isfield(reachability_results, 'samples_final_time')
        samples = reachability_results.samples_final_time(1:check_dims, :);
        
        % Check if any sample is outside the safe set
        falsification_check = any(samples < saferegion_inf | samples > saferegion_sup, 1);
        
        if any(falsification_check)
            num_falsified = num_falsified + 1;
            num_unknown = num_unknown - 1;
            overall_safe = false; % Stop the loop
            falsified_example = samples(:, find(falsification_check, 1));
            fprintf('  Set FALSIFIED: Found counterexample at output: [%s]\n', ...
                strjoin(arrayfun(@(x) sprintf('%.3f', x), falsified_example, 'UniformOutput', false), ', '));
            break;
        else
            fprintf('  Falsification: No counterexample found among %d samples.\n', size(samples, 2));
        end
    else
        fprintf('  Falsification: No samples available for check.\n');
    end

    % ---------------------------------------------------------------------
    % 3. Refine input sets (Split the current set and add to queue)
    % ---------------------------------------------------------------------
    
    % Heuristic: Split the dimension with the largest radius
    [max_r, split_dim] = max(r_i);
    
    % New radii
    r_new = r_i;
    r_new(split_dim) = max_r / 2;
    
    % New centers
    x_c1 = x_i;
    x_c1(split_dim) = x_i(split_dim) - max_r / 2;
    
    x_c2 = x_i;
    x_c2(split_dim) = x_i(split_dim) + max_r / 2;
    
    % Add new sets to the queue
    verification_queue{end+1} = {x_c1, r_new};
    verification_queue{end+1} = {x_c2, r_new};
    
    fprintf('  Set REFINED: Split dimension %d (radius %.4e) into two new sets.\n', split_dim, max_r);
    num_unknown = num_unknown - 1 + 2; % Remove one unknown, add two new unknowns
end

% Clear global variables
clear global Jacobian_x_low Jacobian_x_up system_choice_global

%% ------------------------------------------------------------------------
%   FINAL SUMMARY
% -------------------------------------------------------------------------
fprintf('\n=== Verification Analysis Complete ===\n');
fprintf('System: %s\n', upper(system_name));
fprintf('Total Iterations: %d\n', iter);
fprintf('Verified Subsets: %d\n', num_verified);
fprintf('Falsified Subsets: %d\n', num_falsified);
fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue) + num_unknown);

if overall_safe && isempty(verification_queue)
    fprintf('\nVERDICT: FULLY VERIFIED (SAFE) for the initial set.\n');
elseif ~overall_safe
    fprintf('\nVERDICT: FALSIFIED! Counterexample found.\n');
    fprintf('Counterexample output: [%s]\n', ...
        strjoin(arrayfun(@(x) sprintf('%.3f', x), falsified_example, 'UniformOutput', false), ', '));
elseif overall_safe && iter >= MAX_ITER
    fprintf('\nVERDICT: UNKNOWN (Max iterations reached). %d subsets remain in queue.\n', length(verification_queue));
else
    fprintf('\nVERDICT: UNKNOWN (Verification queue is not empty, but loop terminated). %d subsets remain in queue.\n', length(verification_queue));
end

%------------- END OF CODE --------------
