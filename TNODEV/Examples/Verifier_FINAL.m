% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 27 October 2025 (Fixed most-sensitive-input-radius heuristic)
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
addpath('./') % Add current directory for utility functions --> "check if this is needed" 

%%%  -------------------------------------------------------------------
%  USER CONFIGURATION - EDIT ONLY THIS BLOCK
%%%  -------------------------------------------------------------------

% === 1. SYSTEM SELECTION ===
% system_choice = 14;   % FPA System
system_choice = 15;   % Spiral_non System

% === 2. INITIAL SET ===
% Spiral_non (2D)
x0_spiral     = [2.0; 0.0];            % nODE input set center
radius_spiral = 0.2;                   % Initial radius
timeStep_spiral = 0.01;                % CORA and TIRA time step
tFinal_spiral   = 1.0;                 % CORA and TIRA final time

% FPA (5D)
x0_fpa     = [0, -0.58587, 0.8, 0.52323, 0.7]';
radius_fpa = 0.01;
timeStep_fpa = 0.05;                   % CORA and TIRA time step
tFinal_fpa   = 2.0;                    % CORA and TIRA final time

% Auto-select based on system_choice
if system_choice == 15
    x0             = x0_spiral;
    Initial_radius = radius_spiral;
    system_name    = 'spiral_non';
    cora_params.timeStep = timeStep_spiral;
    cora_params.tFinal   = tFinal_spiral;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_spiral;
    timeStep_TIRA        = timeStep_spiral;
elseif system_choice == 14
    x0             = x0_fpa;
    Initial_radius = radius_fpa;
    system_name    = 'fpa';
    cora_params.timeStep = timeStep_fpa;
    cora_params.tFinal   = tFinal_fpa;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_fpa;
    timeStep_TIRA        = timeStep_fpa;
else
    error('Invalid system_choice. Use 14 (FPA) or 15 (Spiral_non).');
end

% === 3. CORA PARAMETERS (for reachable tube bounds) ===
cora_params.taylorTerms  = 2;      % Taylor terms for nonlinear approximation
cora_params.zonotopeOrder = 20;    % Zonotope order for set representation
cora_params.alg          = 'lin';  % Linear approximation
cora_params.tensorOrder  = 2;      % Tensor order for nonlinear dynamics --> "check what will happen if I increase this"

% === 4. TIRA PARAMETERS (for nODE reachability @NODE_RA.m) ===
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
% node_params.safe_sup = [0.3; 2];    % [x1_up, x2_up]

% Example 4: Safety violated for some methods
node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]

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
% node_params.safe_inf = [-1.6; 0.9];   % [x1_low, x2_low]
% node_params.safe_sup = [0.5; 2]; % [x1_up,  x2_up]

% === 6. VERIFICATION LOOP PARAMETERS ===
MAX_ITER = 50;     %--> "check this crtiteria either we conclude based on that or a time out with a defined time for example 45 mins."
MIN_RADIUS = 1e-4; % Minimum radius for a set to be considered for splitting

% === 7. REFINEMENT HEURISTIC CHOICE===
% Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
node_params.input_split_heuristic = 'naive';

% === 8. METHOD NAMES (for output clarity) ===
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

% Start timer for total VERDICT time --> basically the verifier time
tic;

% Queue of input sets to verify: {x_center, radius}
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

if system_choice == 14
    reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
elseif system_choice == 15
    reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
end

fprintf('PART 1 completed.\n\n');

% Pre-compute Jacobian Bounds once, as they depend on the full CORA reachable tube
fprintf('PART 2: Computing Jacobian Bounds...\n');
a = reachable_tube_bounds.lb;
b = reachable_tube_bounds.ub;
[J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
fprintf('PART 2 completed.\n\n');

% Validate the computed Jacobian bounds
n = length(x0); % State dimension
if isempty(J_lb) || isempty(J_ub) || size(J_lb, 1) ~= n || size(J_lb, 2) ~= n || size(J_ub, 1) ~= n || size(J_ub, 2) ~= n
    warning('Invalid Jacobian bounds (J_lb or J_ub empty or wrong dimensions: J_lb=%s, J_ub=%s). Falling back to naive heuristic.', ...
        mat2str(size(J_lb)), mat2str(size(J_ub)));
    node_params.input_split_heuristic = 'naive';
elseif any(isnan(J_lb(:)) | isinf(J_lb(:)) | isnan(J_ub(:)) | isinf(J_ub(:)))
    warning('Jacobian bounds contain NaN or Inf values. Falling back to naive heuristic.');
    node_params.input_split_heuristic = 'naive';
end

% Pass Jacobians to UP_Jacobian_Bounds.m
fprintf('PART 3: Passing Jacobian bounds via globals...\n');
global Jacobian_x_low Jacobian_x_up system_choice_global
Jacobian_x_low = J_lb;
Jacobian_x_up  = J_ub;
system_choice_global = system_choice;

[~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
fprintf('PART 3 completed.\n\n');

%% -----------------------------------------------------------------------------
%   MAIN VERIFICATION LOOP (1. Verification, 2. Falsification, 3. Refinement)
% ----------------------------------------------------------------------------
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
    fprintf('  PART 4: Calling NODE_RA.m for set with center [%.3f, ...] and radius %.3f\n', x_i(1), r_i);

    % Build NODE_RA input for the current subset
    node_params.system_choice  = system_choice;
    node_params.x0             = x_i;
    node_params.Initial_radius = r_i;
    node_params.t_final        = t_final_TIRA;
    node_params.timeStep       = timeStep_TIRA;

    % Call NODE_RA
    reachability_results = NODE_RA(node_params);

    % Get the final over-approximated interval for all methods
    k = size(reachability_results.succ_low_all_methods, 2);
    num_methods = size(reachability_results.succ_low_all_methods, 3);
    
    % Get the interval hull of all boundary splits for the boundary methods
    if num_methods >= 3
        xb3_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
        I3_low = min(cat(2, xb3_low{:}), [], 2);
        xb3_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
        I3_up = max(cat(2, xb3_up{:}), [], 2);
        
        xb4_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
        I4_low = min(cat(2, xb4_low{:}), [], 2);
        xb4_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
        I4_up = max(cat(2, xb4_up{:}), [], 2);
    end
    
    % Aggregate all over-approximations for the safety check
    OA_low = reachability_results.succ_low_all_methods(:, k, :);
    OA_up  = reachability_results.succ_up_all_methods(:, k, :);
    
    if num_methods >= 3
        OA_low(:, 1, 3) = I3_low;
        OA_up(:, 1, 3) = I3_up;
    end
    if num_methods >= 4
        OA_low(:, 1, 4) = I4_low;
        OA_up(:, 1, 4) = I4_up;
    end
    
    % -------------------------------------------------------------------------------
    % 1. Verification (Check if all OA intervals are fully contained in the safe set)
    % -------------------------------------------------------------------------------
    saferegion_inf = node_params.safe_inf;
    saferegion_sup = node_params.safe_sup;
    
    check_dims = size(saferegion_inf, 1);
    
    is_verified = true;
    for m = 1:num_methods
        low_check = all(OA_low(1:check_dims, 1, m) >= saferegion_inf);
        up_check = all(OA_up(1:check_dims, 1, m) <= saferegion_sup);
        
        if low_check && up_check
            fprintf('  Verification: Method %d (%s) VERIFIED (SAFE).\n', m, method_names{m});
        else
            fprintf('  Verification: Method %d (%s) UNKNOWN (violates safe set bounds: low=%d, up=%d).\n', m, method_names{m}, ~low_check, ~up_check);
            is_verified = false;
        end
    end

    if is_verified
        num_verified = num_verified + 1;
        num_unknown = num_unknown - 1;
        fprintf('  Set VERIFIED: All methods are safe for this input set.\n');
        continue;
    end

    % ---------------------------------------------------------------------
    % 2. Falsification (Check if any sample violates the safe set)
    % ---------------------------------------------------------------------
    if isfield(reachability_results, 'samples_final_time')
        samples = reachability_results.samples_final_time(1:check_dims, :);
        
        falsification_check = any(samples < saferegion_inf | samples > saferegion_sup, 1);
        
        if any(falsification_check)
            num_falsified = num_falsified + 1;
            num_unknown = num_unknown - 1;
            overall_safe = false;
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
    % 3. Refine input sets (Split using selected heuristic)
    % ---------------------------------------------------------------------
    
    % Compute heuristic for splitting
    switch node_params.input_split_heuristic
        case 'naive'
            % Split the dimension with the largest radius
            [max_r, split_dim] = max(r_i);
            fprintf('  Refinement: Using naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
        case 'most-sensitive-input-radius'
            % Compute sensitivity using Jacobian bounds
            % Use the maximum absolute Jacobian value as sensitivity
            try
                sens = max(abs(J_lb), abs(J_ub)); % n x n matrix
                sens = mean(sens, 2); % Average over output dimensions (n x 1)
                if length(sens) ~= length(r_i) || any(isnan(sens) | isinf(sens))
                    warning('Invalid sensitivity vector (length=%d, expected=%d, or contains NaN/Inf). Falling back to naive heuristic.', ...
                        length(sens), length(r_i));
                    [max_r, split_dim] = max(r_i);
                    fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
                else
                    heuristic = r_i .* sens; % Scale radius by sensitivity
                    [max_h, split_dim] = max(heuristic);
                    fprintf('  Refinement: Using most-sensitive-input-radius heuristic, splitting dimension %d (heuristic %.4e).\n', split_dim, max_h);
                end
            catch e
                warning('Error in most-sensitive-input-radius heuristic: %s. Falling back to naive heuristic.', e.message);
                [max_r, split_dim] = max(r_i);
                fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
            end
        case 'interval-norm-gradient'
            % Approximate gradient of the output interval norm w.r.t. input dimensions
            try
                grad = zeros(size(x_i));
                delta = 1e-6; % Small perturbation for numerical gradient
                base_low = OA_low(:, 1, 1); % Use first method's output interval
                base_up = OA_up(:, 1, 1);
                base_norm = sum(base_up - base_low); % Interval norm (sum of widths)
                for j = 1:length(x_i)
                    % Perturb the j-th dimension
                    x_pert = x_i;
                    x_pert(j) = x_i(j) + delta;
                    node_params.x0 = x_pert;
                    pert_results = NODE_RA(node_params);
                    pert_low = pert_results.succ_low_all_methods(:, end, 1);
                    pert_up = pert_results.succ_up_all_methods(:, end, 1);
                    pert_norm = sum(pert_up - pert_low);
                    grad(j) = (pert_norm - base_norm) / delta;
                end
                heuristic = abs(grad); % Use absolute gradient
                if any(isnan(heuristic) | isinf(heuristic))
                    warning('Invalid gradient vector (contains NaN/Inf). Falling back to naive heuristic.');
                    [max_r, split_dim] = max(r_i);
                    fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
                else
                    [max_h, split_dim] = max(heuristic);
                    fprintf('  Refinement: Using interval-norm-gradient heuristic, splitting dimension %d (heuristic %.4e).\n', split_dim, max_h);
                end
            catch e
                warning('Error in interval-norm-gradient heuristic: %s. Falling back to naive heuristic.', e.message);
                [max_r, split_dim] = max(r_i);
                fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
            end
        otherwise
            error('Invalid input_split_heuristic. Use ''naive'', ''most-sensitive-input-radius'', or ''interval-norm-gradient''.');
    end
    
    % Validate split_dim
    if ~isscalar(split_dim) || split_dim < 1 || split_dim > length(r_i) || isnan(split_dim) || isinf(split_dim)
        warning('Invalid split_dim (%d). Falling back to naive heuristic.', split_dim);
        [max_r, split_dim] = max(r_i);
        fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
    end
    
    % Perform the split
    r_new = r_i;
    r_new(split_dim) = r_i(split_dim) / 2;
    
    x_c1 = x_i;
    x_c1(split_dim) = x_i(split_dim) - r_i(split_dim) / 2;
    
    x_c2 = x_i;
    x_c2(split_dim) = x_i(split_dim) + r_i(split_dim) / 2;
    
    % Add new sets to the queue
    verification_queue{end+1} = {x_c1, r_new};
    verification_queue{end+1} = {x_c2, r_new};
    
    fprintf('  Set REFINED: Split dimension %d into two new sets.\n', split_dim);
    num_unknown = num_unknown - 1 + 2;
end

% Clear global variables
clear global Jacobian_x_low Jacobian_x_up system_choice_global

%% ------------------------------------------------------------------------
%   FINAL SUMMARY
% -------------------------------------------------------------------------

% Stop VERDICT timer and get elapsed time
elapsed_time = toc;

fprintf('\n=== Verification Analysis Complete ===\n');
fprintf('System: %s\n', upper(system_name));
fprintf('Total Iterations: %d\n', iter);
fprintf('Verified Subsets: %d\n', num_verified);
fprintf('Falsified Subsets: %d\n', num_falsified);
fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue) + num_unknown);
% Logic to print the elapsed time in seconds (if less than 60 seconds) or minutes 
% (if 60 seconds or more) for easier readability
if elapsed_time < 60
    fprintf('Total Time Taken: %.2f seconds\n', elapsed_time);
else
    fprintf('Total Time Taken: %.2f minutes\n', elapsed_time / 60);
end

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