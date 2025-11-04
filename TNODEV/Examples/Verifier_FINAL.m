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

% %------------- BEGIN CODE --------------
% 
% clear all; close all; clc;
% 
% %%% ========================================================================
% %   Verifier.m - Iterative Verification, Falsification, and Refinement Loop
% %   Systems: Spiral_non (2D) | FPA (5D)
% %   Tools:   CORA (tube bounds) | TIRA (NODE_RA over-approximations)
% %%% =========================================================================
% 
% addpath('../OA_methods')
% addpath('../Utilities')
% addpath('./') % Add current directory for utility functions --> "check if this is needed" 
% 
% %%%  -------------------------------------------------------------------
% %  USER CONFIGURATION - EDIT ONLY THIS BLOCK
% %%%  -------------------------------------------------------------------
% 
% % === 1. SYSTEM SELECTION ===
% % system_choice = 14;   % FPA System
% system_choice = 15;   % Spiral_non System
% 
% % === 2. INITIAL SET ===
% % Spiral_non (2D)
% x0_spiral     = [2.0; 0.0];            % nODE input set center
% radius_spiral = 0.2;                   % Initial radius
% timeStep_spiral = 0.01;                % CORA and TIRA time step
% tFinal_spiral   = 1.0;                 % CORA and TIRA final time
% 
% % FPA (5D)
% x0_fpa     = [0, -0.58587, 0.8, 0.52323, 0.7]';
% radius_fpa = 0.01;
% timeStep_fpa = 0.05;                   % CORA and TIRA time step
% tFinal_fpa   = 2.0;                    % CORA and TIRA final time
% 
% % Auto-select based on system_choice
% if system_choice == 15
%     x0             = x0_spiral;
%     Initial_radius = radius_spiral;
%     system_name    = 'spiral_non';
%     cora_params.timeStep = timeStep_spiral;
%     cora_params.tFinal   = tFinal_spiral;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_spiral;
%     timeStep_TIRA        = timeStep_spiral;
% elseif system_choice == 14
%     x0             = x0_fpa;
%     Initial_radius = radius_fpa;
%     system_name    = 'fpa';
%     cora_params.timeStep = timeStep_fpa;
%     cora_params.tFinal   = tFinal_fpa;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_fpa;
%     timeStep_TIRA        = timeStep_fpa;
% else
%     error('Invalid system_choice. Use 14 (FPA) or 15 (Spiral_non).');
% end
% 
% % === 3. CORA PARAMETERS (for reachable tube bounds) ===
% cora_params.taylorTerms  = 2;      % Taylor terms for nonlinear approximation
% cora_params.zonotopeOrder = 20;    % Zonotope order for set representation
% cora_params.alg          = 'lin';  % Linear approximation
% cora_params.tensorOrder  = 2;      % Tensor order for nonlinear dynamics --> "check what will happen if I increase this"
% 
% % === 4. TIRA PARAMETERS (for nODE reachability @NODE_RA.m) ===
% node_params.plot_last_only     = true;
% node_params.sample_succ_number = 1000;
% 
% % === 5. SAFE SET (USER-ADJUSTABLE) ===
% %%%%%%%%%%%%% Spiral_non System %%%%%%%%%%%%% 
% % Example 1: Safety verified for all OA methods
% % node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% % node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)
% 
% % Example 2: Safety violated for some methods
% % node_params.safe_inf = [-1.3; 0.95];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.25; 2.45]; % [x1_up,  x2_up]
% 
% % Example 3: Safety violated for some methods
% node_params.safe_inf = [-1; 1.2];   % [x1_low, x2_low]
% node_params.safe_sup = [0.3; 2];    % [x1_up, x2_up]
% 
% % Example 4: Safety violated for some methods
% % node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]
% 
% %%%%%%%%%%%%% FPA System %%%%%%%%%%%%% 
% % Example 1: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% % node_params.safe_inf = [-1.5; -1];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.5; 3]; % [x1_up,  x2_up]
% 
% % Example 2: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% % node_params.safe_inf = [-1.4; -0.8];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.38; 2.55]; % [x1_up,  x2_up]
% 
% % FPA Example 3: Safety violated for some methods
% % node_params.safe_inf = [-1; 0.2];   % [x1_low, x2_low]
% % node_params.safe_sup = [-0.5; 0.5]; % [x1_up,  x2_up]
% 
% % FPA Example 4: Safety violated for some methods
% % node_params.safe_inf = [-1.6; 0.9];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.5; 2]; % [x1_up,  x2_up]
% 
% % === 6. VERIFICATION LOOP PARAMETERS ===
% MAX_ITER = 50;     %--> "check this crtiteria either we conclude based on that or a time out with a defined time for example 45 mins."
% MIN_RADIUS = 1e-4; % Minimum radius for a set to be considered for splitting
% 
% % === 7. REFINEMENT HEURISTIC CHOICE===
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% node_params.input_split_heuristic = 'most-sensitive-input-radius';
% 
% % === 8. METHOD NAMES (for output clarity) ===
% method_names = {...
%     'Mixed-Monotonicity (Single-step)', ...
%     'Sampled-Data Mixed-Monotonicity (Single-step)', ...
%     'Mixed-Monotonicity (Boundary)', ...
%     'Sampled-Data Mixed-Monotonicity (Boundary)'};
% 
% % =========================================================================
% fprintf('=== Starting Iterative Verification Analysis ===\n');
% fprintf('System: %s (choice=%d)\n', system_name, system_choice);
% fprintf('Initial set: center = [%s], radius = %.3f\n', ...
%         strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
%         Initial_radius);
% fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
% fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);
% 
% %% ------------------------------------------------------------------------
% %   INITIALIZATION
% % -------------------------------------------------------------------------
% 
% % Start timer for total VERDICT time --> basically the verifier time
% tic;
% 
% % Queue of input sets to verify: {x_center, radius}
% verification_queue = {{x0, Initial_radius}};
% 
% % Results tracking
% num_verified = 0;
% num_falsified = 0;
% num_unknown = 1;
% 
% % Global flag for safety
% overall_safe = true;
% falsified_example = [];
% 
% % Pre-compute CORA bounds once, as they are for the whole input set
% fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');
% cora_full_params = cora_params;
% cora_full_params.x0             = x0;
% cora_full_params.Initial_radius = Initial_radius;
% 
% if system_choice == 14
%     reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
% elseif system_choice == 15
%     reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
% end
% 
% fprintf('PART 1 completed.\n\n');
% 
% % Pre-compute Jacobian Bounds once, as they depend on the full CORA reachable tube
% fprintf('PART 2: Computing Jacobian Bounds...\n');
% a = reachable_tube_bounds.lb;
% b = reachable_tube_bounds.ub;
% [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
% fprintf('PART 2 completed.\n\n');
% 
% % Validate the computed Jacobian bounds
% n = length(x0); % State dimension
% if isempty(J_lb) || isempty(J_ub) || size(J_lb, 1) ~= n || size(J_lb, 2) ~= n || size(J_ub, 1) ~= n || size(J_ub, 2) ~= n
%     warning('Invalid Jacobian bounds (J_lb or J_ub empty or wrong dimensions: J_lb=%s, J_ub=%s). Falling back to naive heuristic.', ...
%         mat2str(size(J_lb)), mat2str(size(J_ub)));
%     node_params.input_split_heuristic = 'naive';
% elseif any(isnan(J_lb(:)) | isinf(J_lb(:)) | isnan(J_ub(:)) | isinf(J_ub(:)))
%     warning('Jacobian bounds contain NaN or Inf values. Falling back to naive heuristic.');
%     node_params.input_split_heuristic = 'naive';
% end
% 
% % Pass Jacobians to UP_Jacobian_Bounds.m
% fprintf('PART 3: Passing Jacobian bounds via globals...\n');
% global Jacobian_x_low Jacobian_x_up system_choice_global
% Jacobian_x_low = J_lb;
% Jacobian_x_up  = J_ub;
% system_choice_global = system_choice;
% 
% [~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
% fprintf('PART 3 completed.\n\n');
% 
% %% -----------------------------------------------------------------------------
% %   MAIN VERIFICATION LOOP (1. Verification, 2. Falsification, 3. Refinement)
% % ----------------------------------------------------------------------------
% iter = 0;
% while ~isempty(verification_queue) && iter < MAX_ITER && overall_safe
% 
%     iter = iter + 1;
%     fprintf('--- ITERATION %d: Queue size = %d ---\n', iter, length(verification_queue));
% 
%     % Pop the next set from the queue
%     current_set = verification_queue{1};
%     verification_queue(1) = [];
%     x_i = current_set{1};
%     r_i = current_set{2};
% 
%     % Check if the set is too small to split further
%     if max(r_i) < MIN_RADIUS
%         fprintf('  Set radius (%.4e) below minimum threshold (%.4e). Skipping refinement.\n', max(r_i), MIN_RADIUS);
%         num_unknown = num_unknown + 1;
%         continue;
%     end
% 
%     % ---------------------------------------------------------------------
%     % PART 4: Call nODE Reachability Analysis (NODE_RA.m)
%     % ---------------------------------------------------------------------
%     fprintf('  PART 4: Calling NODE_RA.m for set with center [%.3f, ...] and radius %.3f\n', x_i(1), r_i);
% 
%     % Build NODE_RA input for the current subset
%     node_params.system_choice  = system_choice;
%     node_params.x0             = x_i;
%     node_params.Initial_radius = r_i;
%     node_params.t_final        = t_final_TIRA;
%     node_params.timeStep       = timeStep_TIRA;
% 
%     % Call NODE_RA
%     reachability_results = NODE_RA(node_params);
% 
%     % Get the final over-approximated interval for all methods
%     k = size(reachability_results.succ_low_all_methods, 2);
%     num_methods = size(reachability_results.succ_low_all_methods, 3);
% 
%     % Get the interval hull of all boundary splits for the boundary methods
%     if num_methods >= 3
%         xb3_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_low = min(cat(2, xb3_low{:}), [], 2);
%         xb3_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_up = max(cat(2, xb3_up{:}), [], 2);
% 
%         xb4_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_low = min(cat(2, xb4_low{:}), [], 2);
%         xb4_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_up = max(cat(2, xb4_up{:}), [], 2);
%     end
% 
%     % Aggregate all over-approximations for the safety check
%     OA_low = reachability_results.succ_low_all_methods(:, k, :);
%     OA_up  = reachability_results.succ_up_all_methods(:, k, :);
% 
%     if num_methods >= 3
%         OA_low(:, 1, 3) = I3_low;
%         OA_up(:, 1, 3) = I3_up;
%     end
%     if num_methods >= 4
%         OA_low(:, 1, 4) = I4_low;
%         OA_up(:, 1, 4) = I4_up;
%     end
% 
%     % -------------------------------------------------------------------------------
%     % 1. Verification (Check if all OA intervals are fully contained in the safe set)
%     % -------------------------------------------------------------------------------
%     saferegion_inf = node_params.safe_inf;
%     saferegion_sup = node_params.safe_sup;
% 
%     check_dims = size(saferegion_inf, 1);
% 
%     is_verified = true;
%     for m = 1:num_methods
%         low_check = all(OA_low(1:check_dims, 1, m) >= saferegion_inf);
%         up_check = all(OA_up(1:check_dims, 1, m) <= saferegion_sup);
% 
%         if low_check && up_check
%             fprintf('  Verification: Method %d (%s) VERIFIED (SAFE).\n', m, method_names{m});
%         else
%             fprintf('  Verification: Method %d (%s) UNKNOWN (violates safe set bounds: low=%d, up=%d).\n', m, method_names{m}, ~low_check, ~up_check);
%             is_verified = false;
%         end
%     end
% 
%     if is_verified
%         num_verified = num_verified + 1;
%         num_unknown = num_unknown - 1;
%         fprintf('  Set VERIFIED: All methods are safe for this input set.\n');
%         continue;
%     end
% 
%     % ---------------------------------------------------------------------
%     % 2. Falsification (Check if any sample violates the safe set)
%     % ---------------------------------------------------------------------
%     if isfield(reachability_results, 'samples_final_time')
%         samples = reachability_results.samples_final_time(1:check_dims, :);
% 
%         falsification_check = any(samples < saferegion_inf | samples > saferegion_sup, 1);
% 
%         if any(falsification_check)
%             num_falsified = num_falsified + 1;
%             num_unknown = num_unknown - 1;
%             overall_safe = false;
%             falsified_example = samples(:, find(falsification_check, 1));
%             fprintf('  Set FALSIFIED: Found counterexample at output: [%s]\n', ...
%                 strjoin(arrayfun(@(x) sprintf('%.3f', x), falsified_example, 'UniformOutput', false), ', '));
%             break;
%         else
%             fprintf('  Falsification: No counterexample found among %d samples.\n', size(samples, 2));
%         end
%     else
%         fprintf('  Falsification: No samples available for check.\n');
%     end
% 
%     % ---------------------------------------------------------------------
%     % 3. Refine input sets (Split using selected heuristic)
%     % ---------------------------------------------------------------------
% 
%     % Compute heuristic for splitting
%     switch node_params.input_split_heuristic
%         case 'naive'
%             % Split the dimension with the largest radius
%             [max_r, split_dim] = max(r_i);
%             fprintf('  Refinement: Using naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%         case 'most-sensitive-input-radius'
%             % Compute sensitivity using Jacobian bounds
%             % Use the maximum absolute Jacobian value as sensitivity
%             try
%                 sens = max(abs(J_lb), abs(J_ub)); % n x n matrix
%                 sens = mean(sens, 2); % Average over output dimensions (n x 1)
%                 if length(sens) ~= length(r_i) || any(isnan(sens) | isinf(sens))
%                     warning('Invalid sensitivity vector (length=%d, expected=%d, or contains NaN/Inf). Falling back to naive heuristic.', ...
%                         length(sens), length(r_i));
%                     [max_r, split_dim] = max(r_i);
%                     fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%                 else
%                     heuristic = r_i .* sens; % Scale radius by sensitivity
%                     [max_h, split_dim] = max(heuristic);
%                     fprintf('  Refinement: Using most-sensitive-input-radius heuristic, splitting dimension %d (heuristic %.4e).\n', split_dim, max_h);
%                 end
%             catch e
%                 warning('Error in most-sensitive-input-radius heuristic: %s. Falling back to naive heuristic.', e.message);
%                 [max_r, split_dim] = max(r_i);
%                 fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%             end
%         case 'interval-norm-gradient'
%             % Approximate gradient of the output interval norm w.r.t. input dimensions
%             try
%                 grad = zeros(size(x_i));
%                 delta = 1e-6; % Small perturbation for numerical gradient
%                 base_low = OA_low(:, 1, 1); % Use first method's output interval
%                 base_up = OA_up(:, 1, 1);
%                 base_norm = sum(base_up - base_low); % Interval norm (sum of widths)
%                 for j = 1:length(x_i)
%                     % Perturb the j-th dimension
%                     x_pert = x_i;
%                     x_pert(j) = x_i(j) + delta;
%                     node_params.x0 = x_pert;
%                     pert_results = NODE_RA(node_params);
%                     pert_low = pert_results.succ_low_all_methods(:, end, 1);
%                     pert_up = pert_results.succ_up_all_methods(:, end, 1);
%                     pert_norm = sum(pert_up - pert_low);
%                     grad(j) = (pert_norm - base_norm) / delta;
%                 end
%                 heuristic = abs(grad); % Use absolute gradient
%                 if any(isnan(heuristic) | isinf(heuristic))
%                     warning('Invalid gradient vector (contains NaN/Inf). Falling back to naive heuristic.');
%                     [max_r, split_dim] = max(r_i);
%                     fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%                 else
%                     [max_h, split_dim] = max(heuristic);
%                     fprintf('  Refinement: Using interval-norm-gradient heuristic, splitting dimension %d (heuristic %.4e).\n', split_dim, max_h);
%                 end
%             catch e
%                 warning('Error in interval-norm-gradient heuristic: %s. Falling back to naive heuristic.', e.message);
%                 [max_r, split_dim] = max(r_i);
%                 fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%             end
%         otherwise
%             error('Invalid input_split_heuristic. Use ''naive'', ''most-sensitive-input-radius'', or ''interval-norm-gradient''.');
%     end
% 
%     % Validate split_dim
%     if ~isscalar(split_dim) || split_dim < 1 || split_dim > length(r_i) || isnan(split_dim) || isinf(split_dim)
%         warning('Invalid split_dim (%d). Falling back to naive heuristic.', split_dim);
%         [max_r, split_dim] = max(r_i);
%         fprintf('  Refinement: Fallback to naive heuristic, splitting dimension %d (radius %.4e).\n', split_dim, max_r);
%     end
% 
%     % Perform the split
%     r_new = r_i;
%     r_new(split_dim) = r_i(split_dim) / 2;
% 
%     x_c1 = x_i;
%     x_c1(split_dim) = x_i(split_dim) - r_i(split_dim) / 2;
% 
%     x_c2 = x_i;
%     x_c2(split_dim) = x_i(split_dim) + r_i(split_dim) / 2;
% 
%     % Add new sets to the queue
%     verification_queue{end+1} = {x_c1, r_new};
%     verification_queue{end+1} = {x_c2, r_new};
% 
%     fprintf('  Set REFINED: Split dimension %d into two new sets.\n', split_dim);
%     num_unknown = num_unknown - 1 + 2;
% end
% 
% % Clear global variables
% clear global Jacobian_x_low Jacobian_x_up system_choice_global
% 
% %% ------------------------------------------------------------------------
% %   FINAL SUMMARY
% % -------------------------------------------------------------------------
% 
% % Stop VERDICT timer and get elapsed time
% elapsed_time = toc;
% 
% fprintf('\n=== Verification Analysis Complete ===\n');
% fprintf('System: %s\n', upper(system_name));
% fprintf('Total Iterations: %d\n', iter);
% fprintf('Verified Subsets: %d\n', num_verified);
% fprintf('Falsified Subsets: %d\n', num_falsified);
% fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue) + num_unknown);
% % Logic to print the elapsed time in seconds (if less than 60 seconds) or minutes 
% % (if 60 seconds or more) for easier readability
% if elapsed_time < 60
%     fprintf('Total Time Taken: %.2f seconds\n', elapsed_time);
% else
%     fprintf('Total Time Taken: %.2f minutes\n', elapsed_time / 60);
% end
% 
% if overall_safe && isempty(verification_queue)
%     fprintf('\nVERDICT: FULLY VERIFIED (SAFE) for the initial set.\n');
% elseif ~overall_safe
%     fprintf('\nVERDICT: FALSIFIED! Counterexample found.\n');
%     fprintf('Counterexample output: [%s]\n', ...
%         strjoin(arrayfun(@(x) sprintf('%.3f', x), falsified_example, 'UniformOutput', false), ', '));
% elseif overall_safe && iter >= MAX_ITER
%     fprintf('\nVERDICT: UNKNOWN (Max iterations reached). %d subsets remain in queue.\n', length(verification_queue));
% else
%     fprintf('\nVERDICT: UNKNOWN (Verification queue is not empty, but loop terminated). %d subsets remain in queue.\n', length(verification_queue));
% end
% 
% %------------- END OF CODE --------------

%------------- BEGIN CODE with Plottings control --------------

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
node_params.safe_inf = [-1; 1.2];   % [x1_low, x2_low]
node_params.safe_sup = [0.3; 2];    % [x1_up, x2_up]

% Example 4: Safety violated for some methods
% node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
% node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]

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
node_params.input_split_heuristic = 'interval-norm-gradient';

% === 8. METHOD NAMES (for output clarity) ===
method_names = {...
    'Mixed-Monotonicity (Single-step)', ...
    'Sampled-Data Mixed-Monotonicity (Single-step)', ...
    'Mixed-Monotonicity (Boundary)', ...
    'Sampled-Data Mixed-Monotonicity (Boundary)'};

% === 9. PLOTTING CONTROL (NEW) ===
plot_control.show_figures = true;                   % Show figures during execution
plot_control.save_each_iteration = true;           % Save plots every iteration
plot_control.save_final_only_if_verified = false;    % Save final only if FULLY VERIFIED
plot_control.save_reachability = true;              % Save reachability plots
plot_control.save_boundary = false;                  % Save boundary points
plot_control.save_interval_hull = false;             % Save interval hull

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
    node_params.plot_control = plot_control;    % PASS PLOTTING CONTROL

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

% % Clear global variables
% clear global Jacobian_x_low Jacobian_x_up system_choice_global

%% ------------------------------------------------------------------------
% FINAL PLOTS (only if save_final_only_if_verified = true AND FULLY VERIFIED)
% -------------------------------------------------------------------------
if ~plot_control.save_each_iteration && ...
   plot_control.save_final_only_if_verified && ...
   overall_safe && isempty(verification_queue)

    fprintf('\n=== FULLY VERIFIED -> Generating FINAL plots ===\n');

    % Recompute CORA tube and Jacobian bounds
    cora_full_params = cora_params;
    cora_full_params.x0 = x0;
    cora_full_params.Initial_radius = Initial_radius;
    if system_choice == 14
        reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
    elseif system_choice == 15
        reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
    end
    a = reachable_tube_bounds.lb; b = reachable_tube_bounds.ub;
    [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);

    global Jacobian_x_low Jacobian_x_up system_choice_global
    Jacobian_x_low = J_lb; Jacobian_x_up = J_ub; system_choice_global = system_choice;
    UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);

    % Call `NODE_RA.m` with final flags
    final_params = node_params;
    final_params.x0 = x0;
    final_params.Initial_radius = Initial_radius;
    final_params.plot_control = plot_control;
    final_params.is_final_call = true;
    final_params.fully_verified = true;

    final_results = NODE_RA(final_params);

    fprintf('Final results saved in: %s\n', final_results.save_dir);
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









































































% %------------- BEGIN CODE 2 --------------
% 
% clear all; close all; clc;
% 
% %%% ========================================================================
% %   Verifier_FINAL.m - Iterative Verification, Falsification, and Refinement
% %   Systems: Spiral_non (2D) | FPA (5D)
% %   Tools:   CORA (tube bounds) | TIRA (NODE_RA over-approximations)
% %%% =========================================================================
% 
% addpath('../OA_methods')
% addpath('../Utilities')
% 
% %%%  -------------------------------------------------------------------
% %  USER CONFIGURATION - EDIT ONLY THIS BLOCK
% %%%  -------------------------------------------------------------------
% 
% % === 1. SYSTEM SELECTION ===
% system_choice = 15;   % 15 = Spiral_non | 14 = FPA
% 
% % === 2. INITIAL SET ===
% % Spiral_non (2D)
% x0_spiral     = [2.0; 0.0];
% radius_spiral = 0.2;
% timeStep_spiral = 0.01;
% tFinal_spiral   = 1.0;
% 
% % FPA (5D)
% x0_fpa     = [0, -0.58587, 0.8, 0.52323, 0.7]';
% radius_fpa = 0.01;
% timeStep_fpa = 0.05;
% tFinal_fpa   = 2.0;
% 
% % Auto-select
% if system_choice == 15
%     x0             = x0_spiral;
%     Initial_radius = radius_spiral;
%     system_name    = 'spiral_non';
%     cora_params.timeStep = timeStep_spiral;
%     cora_params.tFinal   = tFinal_spiral;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_spiral;
%     timeStep_TIRA        = timeStep_spiral;
% elseif system_choice == 14
%     x0             = x0_fpa;
%     Initial_radius = radius_fpa;
%     system_name    = 'fpa';
%     cora_params.timeStep = timeStep_fpa;
%     cora_params.tFinal   = tFinal_fpa;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_fpa;
%     timeStep_TIRA        = timeStep_fpa;
% else
%     error('Invalid system_choice. Use 14 (FPA) or 15 (Spiral_non).');
% end
% 
% % === 3. CORA PARAMETERS ===
% cora_params.taylorTerms  = 2;
% cora_params.zonotopeOrder = 20;
% cora_params.alg          = 'lin';
% cora_params.tensorOrder  = 2;
% 
% % === 4. TIRA PARAMETERS ===
% node_params.plot_last_only     = true;
% node_params.sample_succ_number = 1000;
% 
% % === 5. SAFE SET ===
% %%%%%%%%%%%%% Spiral_non System %%%%%%%%%%%%% 
% % Example 1: Safety verified for all OA methods
% % node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% % node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)
% 
% % Example 2: Safety violated for some methods
% % node_params.safe_inf = [-1.3; 0.95];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.25; 2.45]; % [x1_up,  x2_up]
% 
% % Example 3: Safety violated for some methods
% node_params.safe_inf = [-1; 1.2];   % [x1_low, x2_low]
% node_params.safe_sup = [0.3; 2];    % [x1_up, x2_up]
% 
% % Example 4: Safety violated for some methods
% % node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]
% 
% % === 6. VERIFICATION LOOP ===
% MAX_ITER = 50;
% MIN_RADIUS = 1e-4;
% 
% % === 7. REFINEMENT HEURISTIC ===
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% node_params.input_split_heuristic = 'most-sensitive-input-radius';  
% 
% % === 8. METHOD NAMES ===
% method_names = {...
%     'Mixed-Monotonicity (Single-step)', ...
%     'Sampled-Data Mixed-Monotonicity (Single-step)', ...
%     'Mixed-Monotonicity (Boundary)', ...
%     'Sampled-Data Mixed-Monotonicity (Boundary)'};
% 
% % === 9. SAVE FIGURES PER ITERATION? ===
% save_figures_per_iteration = true;
% 
% % === 10. PLOT CONFIG ===
% node_params.plot_config.show_reachability     = true;
% node_params.plot_config.show_boundary_points  = false;
% node_params.plot_config.show_interval_hull    = false;
% node_params.plot_config.save_figures          = true;
% 
% % =========================================================================
% fprintf('=== Starting Iterative Verification Analysis ===\n');
% fprintf('System: %s (choice=%d)\n', system_name, system_choice);
% fprintf('Initial set: center = [%s], radius = %.3f\n', ...
%         strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
%         Initial_radius);
% fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
% fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);
% 
% %% ------------------------------------------------------------------------
% %   INITIALIZATION
% % -------------------------------------------------------------------------
% 
% % Root results folder
% results_root_dir = fullfile(pwd, sprintf('Verifier_Results_%s_%s', system_name, datestr(now,'yyyy-mm-dd_HH-MM-SS')));
% mkdir(results_root_dir);
% 
% node_params.results_root_dir = results_root_dir;
% node_params.save_figures_per_iteration = save_figures_per_iteration;
% 
% verdict_file = fullfile(results_root_dir, 'VERDICT.mat');
% tic;
% 
% verification_queue = {{x0, Initial_radius}};
% 
% num_verified = 0;
% num_falsified = 0;
% overall_safe = true;
% falsified_example = [];
% 
% % PART 1: CORA Reachable Tube
% fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');
% cora_full_params = cora_params;
% cora_full_params.x0 = x0;
% cora_full_params.Initial_radius = Initial_radius;
% 
% if system_choice == 14
%     reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
% elseif system_choice == 15
%     reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
% end
% fprintf('PART 1 completed.\n\n');
% 
% % PART 2: Jacobian Bounds
% fprintf('PART 2: Computing Jacobian Bounds...\n');
% a = reachable_tube_bounds.lb;
% b = reachable_tube_bounds.ub;
% [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
% fprintf('PART 2 completed.\n\n');
% 
% % Validate Jacobian bounds
% n = length(x0);
% if isempty(J_lb) || isempty(J_ub) || size(J_lb,1) ~= n || size(J_ub,1) ~= n
%     warning('Invalid Jacobian bounds. Falling back to naive heuristic.');
%     node_params.input_split_heuristic = 'naive';
% elseif any(isnan(J_lb(:)) | isinf(J_lb(:)) | isnan(J_ub(:)) | isinf(J_ub(:)))
%     warning('Jacobian bounds contain NaN/Inf. Falling back to naive.');
%     node_params.input_split_heuristic = 'naive';
% end
% 
% % PART 3: Pass Jacobian bounds
% fprintf('PART 3: Passing Jacobian bounds via globals...\n');
% global Jacobian_x_low Jacobian_x_up system_choice_global
% Jacobian_x_low = J_lb;
% Jacobian_x_up  = J_ub;
% system_choice_global = system_choice;
% [~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
% fprintf('PART 3 completed.\n\n');
% 
% %% -----------------------------------------------------------------------------
% %   MAIN VERIFICATION LOOP
% % -----------------------------------------------------------------------------
% iter = 0;
% while ~isempty(verification_queue) && iter < MAX_ITER && overall_safe
% 
%     iter = iter + 1;
%     fprintf('--- ITERATION %d: Queue size = %d ---\n', iter, length(verification_queue));
% 
%     current_set = verification_queue{1};
%     verification_queue(1) = [];
%     x_i = current_set{1};
%     r_i = current_set{2};
% 
%     if max(r_i) < MIN_RADIUS
%         fprintf('  Set radius (%.4e) below threshold. Skipping.\n', max(r_i));
%         continue;
%     end
% 
%     % PART 4: Call NODE_RA
%     fprintf('  PART 4: Calling NODE_RA.m for center [%.3f, ...], radius %.3f\n', x_i(1), r_i);
% 
%     node_params.system_choice  = system_choice;
%     node_params.x0             = x_i;
%     node_params.Initial_radius = r_i;
%     node_params.t_final        = t_final_TIRA;
%     node_params.timeStep       = timeStep_TIRA;
% 
%     if save_figures_per_iteration
%         iter_dir = fullfile(results_root_dir, sprintf('Iteration_%03d', iter));
%         mkdir(iter_dir);
%         node_params.iter_save_dir = iter_dir;
%     else
%         node_params.iter_save_dir = '';
%     end
% 
%     reachability_results = NODE_RA(node_params);
% 
%     k = size(reachability_results.succ_low_all_methods, 2);
%     num_methods = size(reachability_results.succ_low_all_methods, 3);
% 
%     % Boundary method interval hulls
%     if num_methods >= 3
%         xb3_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_low = min(cat(2, xb3_low{:}), [], 2);
%         xb3_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_up = max(cat(2, xb3_up{:}), [], 2);
% 
%         xb4_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_low = min(cat(2, xb4_low{:}), [], 2);
%         xb4_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_up = max(cat(2, xb4_up{:}), [], 2);
%     end
% 
%     OA_low = reachability_results.succ_low_all_methods(:, k, :);
%     OA_up  = reachability_results.succ_up_all_methods(:, k, :);
%     if num_methods >= 3
%         OA_low(:, 1, 3) = I3_low;
%         OA_up(:, 1, 3) = I3_up;
%     end
%     if num_methods >= 4
%         OA_low(:, 1, 4) = I4_low;
%         OA_up(:, 1, 4) = I4_up;
%     end
% 
%     % 1. Verification
%     saferegion_inf = node_params.safe_inf;
%     saferegion_sup = node_params.safe_sup;
%     check_dims = min(size(saferegion_inf,1), size(OA_low,1));
% 
%     is_verified = true;
%     for m = 1:num_methods
%         low_check = all(OA_low(1:check_dims, 1, m) >= saferegion_inf);
%         up_check  = all(OA_up(1:check_dims, 1, m)  <= saferegion_sup);
%         if low_check && up_check
%             fprintf('  Verification: Method %d (%s) VERIFIED.\n', m, method_names{m});
%         else
%             fprintf('  Verification: Method %d (%s) UNKNOWN.\n', m, method_names{m});
%             is_verified = false;
%         end
%     end
% 
%     if is_verified
%         num_verified = num_verified + 1;
%         fprintf('  Set VERIFIED: All methods safe.\n');
%         continue;
%     end
% 
%     % 2. Falsification
%     if isfield(reachability_results, 'rand_succ')
%         samples = reachability_results.rand_succ(1:check_dims, :);
%         outside = samples < saferegion_inf | samples > saferegion_sup;
%         falsification_check = any(outside, 1);
% 
%         if any(falsification_check)
%             num_falsified = num_falsified + 1;
%             overall_safe = false;
%             idx = find(falsification_check, 1);
%             falsified_example = samples(:, idx);
%             fprintf('  Set FALSIFIED: Counterexample [%s]\n', ...
%                 strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_example, 'UniformOutput', false), ', '));
%             break;
%         else
%             fprintf('  Falsification: No counterexample in %d samples.\n', size(samples,2));
%         end
%     else
%         fprintf('  Falsification: No samples available.\n');
%     end
% 
%     % 3. Refinement
%     switch node_params.input_split_heuristic
%         case 'naive'
%             [~, split_dim] = max(r_i);
%             fprintf('  Refinement: Naive split on dim %d.\n', split_dim);
%         case 'most-sensitive-input-radius'
%             try
%                 sens = max(abs(J_lb), abs(J_ub));
%                 sens = mean(sens, 2);
%                 heuristic = r_i .* sens;
%                 [~, split_dim] = max(heuristic);
%                 fprintf('  Refinement: Sensitive split on dim %d.\n', split_dim);
%             catch
%                 [~, split_dim] = max(r_i);
%                 fprintf('  Refinement: Fallback to naive split on dim %d.\n', split_dim);
%             end
%         case 'interval-norm-gradient'
%             try
%                 grad = zeros(size(x_i));
%                 delta = 1e-6;
%                 base_norm = sum(OA_up(:,1,1) - OA_low(:,1,1));
%                 for j = 1:length(x_i)
%                     x_pert = x_i; x_pert(j) = x_i(j) + delta;
%                     node_params.x0 = x_pert;
%                     pert_res = NODE_RA(node_params);
%                     pert_norm = sum(pert_res.succ_up_all_methods(:,end,1) - pert_res.succ_low_all_methods(:,end,1));
%                     grad(j) = (pert_norm - base_norm) / delta;
%                 end
%                 [~, split_dim] = max(abs(grad));
%                 fprintf('  Refinement: Gradient split on dim %d.\n', split_dim);
%             catch
%                 [~, split_dim] = max(r_i);
%                 fprintf('  Refinement: Fallback to naive split on dim %d.\n', split_dim);
%             end
%         otherwise
%             [~, split_dim] = max(r_i);
%     end
% 
%     r_new = r_i;
%     r_new(split_dim) = r_i(split_dim)/2;
%     x_c1 = x_i; x_c1(split_dim) = x_i(split_dim) - r_i(split_dim)/2;
%     x_c2 = x_i; x_c2(split_dim) = x_i(split_dim) + r_i(split_dim)/2;
% 
%     verification_queue{end+1} = {x_c1, r_new};
%     verification_queue{end+1} = {x_c2, r_new};
%     fprintf('  Set REFINED: Split dim %d → 2 new sets.\n', split_dim);
% end
% 
% clear global Jacobian_x_low Jacobian_x_up system_choice_global
% 
% %% ------------------------------------------------------------------------
% %   FINAL SUMMARY
% % -------------------------------------------------------------------------
% elapsed_time = toc;
% remaining_unknown = length(verification_queue);
% 
% fprintf('\n=== Verification Complete ===\n');
% fprintf('System: %s\n', upper(system_name));
% fprintf('Iterations: %d\n', iter);
% fprintf('Verified: %d\n', num_verified);
% fprintf('Falsified: %d\n', num_falsified);
% fprintf('Remaining Unknown: %d\n', remaining_unknown);
% fprintf('Time: %.2f min\n', elapsed_time/60);
% 
% if overall_safe && remaining_unknown == 0
%     fprintf('\nVERDICT: FULLY VERIFIED (SAFE)\n');
% elseif ~overall_safe
%     fprintf('\nVERDICT: FALSIFIED\n');
%     fprintf('Counterexample: [%s]\n', strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_example, 'UniformOutput', false), ', '));
% elseif iter >= MAX_ITER
%     fprintf('\nVERDICT: UNKNOWN (Max iterations)\n');
% else
%     fprintf('\nVERDICT: UNKNOWN (Queue not empty)\n');
% end
% 
% % Save verdict
% save(verdict_file, 'num_verified', 'num_falsified', 'remaining_unknown', 'overall_safe', 'falsified_example', 'elapsed_time');
% 
% %------------- END OF CODE 2 --------------




% %------------- BEGIN CODE 3 --------------
% 
% clear all; close all; clc;
% 
% %%% ========================================================================
% %   Verifier.m - Iterative Verification, Falsification, and Refinement Loop
% %   Systems: Spiral_non (2D) | FPA (5D)
% %   Tools:   CORA (tube bounds) | TIRA (NODE_RA over-approximations)
% %%% =========================================================================
% 
% addpath('../OA_methods')
% addpath('../Utilities')
% addpath('./') % Add current directory for utility functions --> "check if this is needed" 
% 
% %%%  -------------------------------------------------------------------
% %  USER CONFIGURATION - EDIT ONLY THIS BLOCK
% %%%  -------------------------------------------------------------------
% 
% % === 1. SYSTEM SELECTION ===
% % system_choice = 14;   % FPA System
% system_choice = 15;   % Spiral_non System
% 
% % === 2. INITIAL SET ===
% % Spiral_non (2D)
% x0_spiral     = [2.0; 0.0];            % nODE input set center
% radius_spiral = 0.2;                   % Initial radius
% timeStep_spiral = 0.01;                % CORA and TIRA time step
% tFinal_spiral   = 1.0;                 % CORA and TIRA final time
% 
% % FPA (5D)
% x0_fpa     = [0, -0.58587, 0.8, 0.52323, 0.7]';
% radius_fpa = 0.01;
% timeStep_fpa = 0.05;                   % CORA and TIRA time step
% tFinal_fpa   = 2.0;                    % CORA and TIRA final time
% 
% % Auto-select based on system_choice
% if system_choice == 15
%     x0             = x0_spiral;
%     Initial_radius = radius_spiral;
%     system_name    = 'spiral_non';
%     cora_params.timeStep = timeStep_spiral;
%     cora_params.tFinal   = tFinal_spiral;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_spiral;
%     timeStep_TIRA        = timeStep_spiral;
% elseif system_choice == 14
%     x0             = x0_fpa;
%     Initial_radius = radius_fpa;
%     system_name    = 'fpa';
%     cora_params.timeStep = timeStep_fpa;
%     cora_params.tFinal   = tFinal_fpa;
%     t_init_TIRA          = 0;
%     t_final_TIRA         = tFinal_fpa;
%     timeStep_TIRA        = timeStep_fpa;
% else
%     error('Invalid system_choice. Use 14 (FPA) or 15 (Spiral_non).');
% end
% 
% % === 3. CORA PARAMETERS (for reachable tube bounds) ===
% cora_params.taylorTerms  = 2;      % Taylor terms for nonlinear approximation
% cora_params.zonotopeOrder = 20;    % Zonotope order for set representation
% cora_params.alg          = 'lin';  % Linear approximation
% cora_params.tensorOrder  = 2;      % Tensor order for nonlinear dynamics --> "check what will happen if I increase this"
% 
% % === 4. TIRA PARAMETERS (for nODE reachability @NODE_RA.m) ===
% node_params.plot_last_only     = true;
% node_params.sample_succ_number = 1000;
% 
% % === 5. SAFE SET (USER-ADJUSTABLE) ===
% %%%%%%%%%%%%% Spiral_non System %%%%%%%%%%%%% 
% % Example 1: Safety verified for all OA methods
% % node_params.safe_inf = [-1.8; 0.6];   % lower-left corner (x1, x2)
% % node_params.safe_sup = [0.7; 2.8];   % upper-right corner (x1, x2)
% 
% % Example 2: Safety violated for some methods
% % node_params.safe_inf = [-1.3; 0.95];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.25; 2.45]; % [x1_up,  x2_up]
% 
% % Example 3: Safety violated for some methods
% node_params.safe_inf = [-1; 1.2];   % [x1_low, x2_low]
% node_params.safe_sup = [0.3; 2];    % [x1_up, x2_up]
% 
% % Example 4: Safety violated for some methods
% % node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]
% 
% %%%%%%%%%%%%% FPA System %%%%%%%%%%%%% 
% % Example 1: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% % node_params.safe_inf = [-1.5; -1];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.5; 3]; % [x1_up,  x2_up]
% 
% % Example 2: Safety verified for all OA methods across the 5 dimensions (3 Figs)
% % node_params.safe_inf = [-1.4; -0.8];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.38; 2.55]; % [x1_up,  x2_up]
% 
% % FPA Example 3: Safety violated for some methods
% % node_params.safe_inf = [-1; 0.2];   % [x1_low, x2_low]
% % node_params.safe_sup = [-0.5; 0.5]; % [x1_up,  x2_up]
% 
% % FPA Example 4: Safety violated for some methods
% % node_params.safe_inf = [-1.6; 0.9];   % [x1_low, x2_low]
% % node_params.safe_sup = [0.5; 2]; % [x1_up,  x2_up]
% 
% % === 6. VERIFICATION LOOP PARAMETERS ===
% MAX_ITER = 50;     %--> "check this crtiteria either we conclude based on that or a time out with a defined time for example 45 mins."
% MIN_RADIUS = 1e-4; % Minimum radius for a set to be considered for splitting
% 
% % === 7. FIGURE SAVING CONFIGURATION ===
% SAVE_FIGURES_PER_ITERATION = true; % TRUE to save figures for each subset, FALSE to save only final verdict figures.
% 
% % === 8. REFINEMENT HEURISTIC CHOICE===
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% node_params.input_split_heuristic = 'interval-norm-gradient';
% 
% % === 9. METHOD NAMES (for output clarity) ===
% method_names = {...
%     'Mixed-Monotonicity (Single-step)', ...
%     'Sampled-Data Mixed-Monotonicity (Single-step)', ...
%     'Mixed-Monotonicity (Boundary)', ...
%     'Sampled-Data Mixed-Monotonicity (Boundary)'};
% 
% % =========================================================================
% fprintf('=== Starting Iterative Verification Analysis ===\n');
% fprintf('System: %s (choice=%d)\n', system_name, system_choice);
% fprintf('Initial set: center = [%s], radius = %.3f\n', ...
%         strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
%         Initial_radius);
% fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
% fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);
% 
% %% ------------------------------------------------------------------------
% %   INITIALIZATION
% % -------------------------------------------------------------------------
% 
% % Start timer for total VERDICT time --> basically the verifier time
% tic;
% 
% % Queue of input sets to verify: {x_center, radius}
% verification_queue = {{x0, Initial_radius}};
% 
% % Results tracking
% num_verified = 0;
% num_falsified = 0;
% num_unknown = 1;
% 
% % Global flag for safety
% overall_safe = true;
% falsified_example = [];
% 
% % Pre-compute CORA bounds once, as they are for the whole input set
% fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');
% cora_full_params = cora_params;
% cora_full_params.x0             = x0;
% cora_full_params.Initial_radius = Initial_radius;
% 
% if system_choice == 14
%     reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
% elseif system_choice == 15
%     reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
% end
% 
% fprintf('PART 1 completed.\n\n');
% 
% % Pre-compute Jacobian Bounds once, as they depend on the full CORA reachable tube
% fprintf('PART 2: Computing Jacobian Bounds...\n');
% a = reachable_tube_bounds.lb;
% b = reachable_tube_bounds.ub;
% [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
% fprintf('PART 2 completed.\n\n');
% 
% % Validate the computed Jacobian bounds
% n = length(x0); % State dimension
% if isempty(J_lb) || isempty(J_ub) || size(J_lb, 1) ~= n || size(J_lb, 2) ~= n || size(J_ub, 1) ~= n || size(J_ub, 2) ~= n
%     warning('Invalid Jacobian bounds (J_lb or J_ub empty or wrong dimensions: J_lb=%s, J_ub=%s). Falling back to naive heuristic.', ...
%         mat2str(size(J_lb)), mat2str(size(J_ub)));
%     node_params.input_split_heuristic = 'naive';
% elseif any(isnan(J_lb(:)) | isinf(J_lb(:)) | isnan(J_ub(:)) | isinf(J_ub(:)))
%     warning('Jacobian bounds contain NaN or Inf values. Falling back to naive heuristic.');
%     node_params.input_split_heuristic = 'naive';
% end
% 
% % Pass Jacobians to UP_Jacobian_Bounds.m
% fprintf('PART 3: Passing Jacobian bounds via globals...\n');
% global Jacobian_x_low Jacobian_x_up system_choice_global
% Jacobian_x_low = J_lb;
% Jacobian_x_up  = J_ub;
% system_choice_global = system_choice;
% 
% [~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
% fprintf('PART 3 completed.\n\n');
% 
% %% -----------------------------------------------------------------------------
% %   MAIN VERIFICATION LOOP (1. Verification, 2. Falsification, 3. Refinement)
% % ----------------------------------------------------------------------------
% iter = 0;
% while ~isempty(verification_queue) && iter < MAX_ITER && overall_safe
% 
%     iter = iter + 1;
%     fprintf('--- ITERATION %d: Queue size = %d ---\n', iter, length(verification_queue));
% 
%     % Pop the next set from the queue
%     current_set = verification_queue{1};
%     verification_queue(1) = [];
%     x_i = current_set{1};
%     r_i = current_set{2};
% 
%     % Check if the set is too small to split further
%     if max(r_i) < MIN_RADIUS
%         fprintf('  Set radius (%.4e) below minimum threshold (%.4e). Skipping refinement.\n', max(r_i), MIN_RADIUS);
%         num_unknown = num_unknown + 1;
%         continue;
%     end
% 
%     % ---------------------------------------------------------------------
%     % PART 4: Call nODE Reachability Analysis (NODE_RA.m)
%     % ---------------------------------------------------------------------
%     fprintf('  PART 4: Calling NODE_RA.m for set with center [%.3f, ...] and radius %.3f\n', x_i(1), r_i);
% 
%     % Build NODE_RA input for the current subset
%     node_params.iter_num           = iter; % Current iteration number
%     node_params.save_figures_iter  = SAVE_FIGURES_PER_ITERATION; % Figure saving flag
%     node_params.system_choice  = system_choice;
%     node_params.x0             = x_i;
%     node_params.Initial_radius = r_i;
%     node_params.t_final        = t_final_TIRA;
%     node_params.timeStep       = timeStep_TIRA;
% 
%     % Call NODE_RA
%     reachability_results = NODE_RA(node_params);
% 
%     % Get the final over-approximated interval for all methods
%     k = size(reachability_results.succ_low_all_methods, 2);
%     num_methods = size(reachability_results.succ_low_all_methods, 3);
% 
%     % Get the interval hull of all boundary splits for the boundary methods
%     if num_methods >= 3
%         xb3_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_low = min(cat(2, xb3_low{:}), [], 2);
%         xb3_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,1), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I3_up = max(cat(2, xb3_up{:}), [], 2);
% 
%         xb4_low = arrayfun(@(i) reachability_results.succ_low_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_low = min(cat(2, xb4_low{:}), [], 2);
%         xb4_up  = arrayfun(@(i) reachability_results.succ_up_boundary{i}(:,k,2), 1:reachability_results.num_splits, 'UniformOutput', false);
%         I4_up = max(cat(2, xb4_up{:}), [], 2);
%     end
% 
%     % Get the final over-approximated interval for all methods
%     I1_low = reachability_results.succ_low_all_methods(:, k, 1);
%     I1_up  = reachability_results.succ_up_all_methods(:, k, 1);
%     I2_low = reachability_results.succ_low_all_methods(:, k, 2);
%     I2_up  = reachability_results.succ_up_all_methods(:, k, 2);
% 
%     % Combine boundary methods into interval hull if they exist
%     if num_methods >= 3
%         I3_low = min(cat(2, xb3_low{:}), [], 2);
%         I3_up = max(cat(2, xb3_up{:}), [], 2);
%         I4_low = min(cat(2, xb4_low{:}), [], 2);
%         I4_up = max(cat(2, xb4_up{:}), [], 2);
%     else
%         I3_low = I1_low; I3_up = I1_up; % Fallback to single-step if boundary methods are not computed
%         I4_low = I2_low; I4_up = I2_up;
%     end
% 
%     % All final intervals
%     all_I_low = [I1_low, I2_low, I3_low, I4_low];
%     all_I_up  = [I1_up, I2_up, I3_up, I4_up];
% 
%     % ---------------------------------------------------------------------
%     % 1. Verification (Check if all methods are safe)
%     % ---------------------------------------------------------------------
%     is_safe = true;
%     for m = 1:num_methods
%         % Check if the over-approximated interval is a subset of the safe set
%         % [I_low, I_up] subset of [safe_inf, safe_sup]
%         if any(all_I_low(:, m) < node_params.safe_inf) || any(all_I_up(:, m) > node_params.safe_sup)
%             is_safe = false;
%             break;
%         end
%     end
% 
%     if is_safe
%         fprintf('  VERIFIED: Set is safe for all %d OA methods. Removing from queue.\n', num_methods);
%         num_verified = num_verified + 1;
%         num_unknown = num_unknown - 1;
%         continue; % Move to the next set in the queue
%     end
% 
%     % ---------------------------------------------------------------------
%     % 2. Falsification (Check for counterexamples)
%     % ---------------------------------------------------------------------
%     % Check if any sampled successor violates the safe set
%     rand_succ = reachability_results.rand_succ;
% 
%     % Check if any column in rand_succ is outside the safe set
%     % (rand_succ < safe_inf) OR (rand_succ > safe_sup)
%     is_falsified = any(any(rand_succ < node_params.safe_inf)) || any(any(rand_succ > node_params.safe_sup));
% 
%     if is_falsified
%         fprintf('  FALSIFIED: Counterexample found! Stopping verification.\n');
%         overall_safe = false;
%         num_falsified = num_falsified + 1;
%         num_unknown = num_unknown - 1;
% 
%         % Find the first counterexample
%         violating_indices = find(any(rand_succ < node_params.safe_inf) | any(rand_succ > node_params.safe_sup), 1);
%         if ~isempty(violating_indices)
%             falsified_example = rand_succ(:, violating_indices);
%         end
%         break; % Exit the while loop
%     end
% 
%     % ---------------------------------------------------------------------
%     % 3. Refinement (Split the input set)
%     % ---------------------------------------------------------------------
% 
%     % Find the dimension to split
%     switch lower(node_params.input_split_heuristic)
%         case 'naive'
%             % Naive: Split along the dimension with the largest radius
%             [~, split_dim] = max(r_i);
%         case 'most-sensitive-input-radius'
%             % TODO: Implement logic based on Jacobian bounds and radius
%             [~, split_dim] = max(r_i); % Fallback to naive
%             warning('Heuristic "most-sensitive-input-radius" not implemented. Falling back to naive.');
%         case 'interval-norm-gradient'
%             % TODO: Implement logic based on interval norm of the gradient
%             [~, split_dim] = max(r_i); % Fallback to naive
%             warning('Heuristic "interval-norm-gradient" not implemented. Falling back to naive.');
%         otherwise
%             [~, split_dim] = max(r_i); % Default to naive
%             warning('Unknown heuristic. Falling back to naive.');
%     end
% 
%     % Split the set
%     r_i_new = r_i;
%     r_i_new(split_dim) = r_i(split_dim) / 2;
% 
%     x_i_1 = x_i;
%     x_i_1(split_dim) = x_i(split_dim) - r_i(split_dim) / 2;
% 
%     x_i_2 = x_i;
%     x_i_2(split_dim) = x_i(split_dim) + r_i(split_dim) / 2;
% 
%     % Add the two new subsets back to the queue
%     verification_queue{end+1} = {x_i_1, r_i_new};
%     verification_queue{end+1} = {x_i_2, r_i_new};
% 
%     fprintf('  Set REFINED: Split dimension %d into two new sets.\n', split_dim);
%     num_unknown = num_unknown - 1 + 2;
% end
% 
% % --- Final Figure Saving ---
% if ~SAVE_FIGURES_PER_ITERATION
%     fprintf('\n--- Final Figure Saving ---\n');
%     % Call NODE_RA one last time with the initial set and iter_num = 0 to save the final figure
%     node_params.iter_num           = 0; % Signal for final figure
%     node_params.save_figures_iter  = true; % Temporarily set to true to trigger saving logic
%     node_params.x0                 = x0;
%     node_params.Initial_radius     = Initial_radius;
%     NODE_RA(node_params);
%     fprintf('Final figure saved.\n');
% end
% 
% % Clear global variables
% clear global Jacobian_x_low Jacobian_x_up system_choice_global
% 
% %% ------------------------------------------------------------------------
% %   FINAL SUMMARY
% % -------------------------------------------------------------------------
% 
% % Stop VERDICT timer and get elapsed time
% elapsed_time = toc;
% 
% fprintf('\n=== Verification Analysis Complete ===\n');
% fprintf('System: %s\n', upper(system_name));
% fprintf('Total Iterations: %d\n', iter);
% fprintf('Verified Subsets: %d\n', num_verified);
% fprintf('Falsified Subsets: %d\n', num_falsified);
% fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue) + num_unknown);
% % Logic to print the elapsed time in seconds (if less than 60 seconds) or minutes 
% % (if 60 seconds or more) for easier readability
% if elapsed_time < 60
%     fprintf('Total Time Taken: %.2f seconds\n', elapsed_time);
% else
%     fprintf('Total Time Taken: %.2f minutes\n', elapsed_time / 60);
% end
% 
% if overall_safe && isempty(verification_queue)
%     fprintf('\nVERDICT: FULLY VERIFIED (SAFE) for the initial set.\n');
% elseif ~overall_safe
%     fprintf('\nVERDICT: FALSIFIED! Counterexample found.\n');
%     fprintf('Counterexample output: [%s]\n', ...
%         strjoin(arrayfun(@(x) sprintf('%.3f', x), falsified_example, 'UniformOutput', false), ', '));
% elseif overall_safe && iter >= MAX_ITER
%     fprintf('\nVERDICT: UNKNOWN (Max iterations reached). %d subsets remain in queue.\n', length(verification_queue));
% elseif overall_safe && ~isempty(verification_queue)
%     fprintf('\nVERDICT: UNKNOWN (Verification queue is not empty, but loop terminated). %d subsets remain in queue.\n', length(verification_queue));
% end
% 
% %------------- END OF CODE 3 --------------

% %------------- BEGIN OF CODE 4 --------------
% 
% %% Verifier_FINAL.m
% % Reachability-based verification using TIRA + CORA over-approximation
% % Supports: FPA (system 14), Spiral_non (system 15)
% % Now with: save_figures_each_iteration flag
% 
% clear; clc; close all;
% tic;
% 
% %% ------------------------------------------------------------------------
% % USER CONFIGURATION
% % -------------------------------------------------------------------------
% 
% % === 1. System Choice ===
% system_choice = 15;  % 14 = FPA, 15 = Spiral_non
% 
% % === 2. Initial Set ===
% x0 = [0.5; 0.5];                    % Center
% Initial_radius = 0.1;               % Radius
% 
% % === 3. Time Horizon ===
% t_final_TIRA = 2.0;                 % TIRA final time
% timeStep_TIRA = 0.05;               % TIRA time step
% 
% % === 4. CORA Parameters ===
% cora_params.tFinal = 2.0;
% cora_params.timeStep = 0.01;
% cora_params.taylorTerms = 5;
% cora_params.zonotopeOrder = 20;
% cora_params.intermediateOrder = 20;
% 
% % === 5. Safe Set (2D only) ===
% safe_inf = [0.3; 0.3];              % Lower bound of safe set
% safe_sup = [0.7; 0.7];              % Upper bound of safe set
% 
% % === 6. Verification Parameters ===
% MAX_ITER = 100;                     % Max refinement iterations
% refinement_threshold = 1e-3;        % Stop if radius < threshold
% 
% % === 7. Save Figures Flag ===
% %   true  → save plots + .mat at EVERY iteration
% %   false → save ONLY at the end IF verdict is FULLY VERIFIED
% save_figures_each_iteration = false;
% 
% % === 8. Plot Last Only (NODE_RA) ===
% plot_last_only = true;
% 
% % === 9. Sample Successors ===
% sample_succ_number = 1000;
% 
% %% ------------------------------------------------------------------------
% % INITIALIZATION
% % -------------------------------------------------------------------------
% global Jacobian_x_low Jacobian_x_up system_choice_global
% system_choice_global = system_choice;
% 
% % Initial queue
% verification_queue = struct('center', x0, 'radius', Initial_radius);
% verified_safe = [];
% falsified_examples = [];
% num_verified = 0;
% num_falsified = 0;
% num_unknown = 0;
% iter = 0;
% overall_safe = true;
% 
% % CORA initial reachable tube
% cora_params.x0 = x0;
% cora_params.Initial_radius = Initial_radius;
% 
% if system_choice == 14
%     reachable_tube_bounds = fpa_CORA_RT(cora_params);
% elseif system_choice == 15
%     reachable_tube_bounds = spiral_non_CORA_RT(cora_params);
% else
%     error('Invalid system_choice');
% end
% 
% a = reachable_tube_bounds.lb;
% b = reachable_tube_bounds.ub;
% [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
% 
% % Validate Jacobian bounds
% n = length(x0);
% if size(J_lb,1) ~= n || any(isnan(J_lb(:)) | isinf(J_lb(:)))
%     error('Invalid Jacobian lower bounds');
% end
% if size(J_ub,1) ~= n || any(isnan(J_ub(:)) | isinf(J_ub(:)))
%     error('Invalid Jacobian upper bounds');
% end
% 
% Jacobian_x_low = J_lb;
% Jacobian_x_up  = J_ub;
% 
% % Precompute UP Jacobian bounds
% t_init_TIRA = 0;
% [~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
% 
% fprintf('Initial Jacobian bounds computed and passed.\n');
% 
% %% ------------------------------------------------------------------------
% % MAIN VERIFICATION LOOP
% % -------------------------------------------------------------------------
% while ~isempty(verification_queue) && iter < MAX_ITER
%     iter = iter + 1;
%     current = verification_queue(1);
%     verification_queue(1) = [];
% 
%     x_i = current.center;
%     r_i = current.radius;
% 
%     fprintf('\n--- ITERATION %d ---\n', iter);
%     fprintf('  Center: [%.4f, %.4f], Radius: %.4f\n', x_i(1), x_i(2), r_i);
% 
%     % Skip if too small
%     if r_i < refinement_threshold
%         verified_safe = [verified_safe; x_i', r_i];
%         num_verified = num_verified + 1;
%         fprintf('  Small set → VERIFIED (radius < %.2e)\n', refinement_threshold);
%         continue;
%     end
% 
%     % Build NODE_RA params
%     node_params.system_choice               = system_choice;
%     node_params.x0                          = x_i;
%     node_params.Initial_radius              = r_i;
%     node_params.t_final                     = t_final_TIRA;
%     node_params.timeStep                    = timeStep_TIRA;
%     node_params.plot_last_only              = plot_last_only;
%     node_params.sample_succ_number          = sample_succ_number;
%     node_params.save_figures_each_iteration = save_figures_each_iteration;
%     node_params.safe_inf                    = safe_inf;
%     node_params.safe_sup                    = safe_sup;
% 
%     % Call NODE_RA
%     try
%         reachability_results = NODE_RA(node_params);
%     catch e
%         fprintf('  NODE_RA failed: %s\n', e.message);
%         num_unknown = num_unknown + 1;
%         continue;
%     end
% 
%     % Extract final reachable set (last time step)
%     if plot_last_only
%         k = size(reachability_results.succ_low_all_methods, 2);
%     else
%         k = length(reachability_results.t_points);
%     end
% 
%     % Check safety for each method
%     safe_methods = true(1, reachability_results.n_methods);
%     for m = 1:reachability_results.n_methods
%         low = reachability_results.succ_low_all_methods(:, k, m);
%         up  = reachability_results.succ_up_all_methods(:, k, m);
% 
%         if any(low > safe_sup) || any(up < safe_inf)
%             safe_methods(m) = false;
%         end
%     end
% 
%     % Verdict
%     if all(safe_methods)
%         verified_safe = [verified_safe; x_i', r_i];
%         num_verified = num_verified + 1;
%         fprintf('  ALL METHODS SAFE → VERIFIED\n');
%     else
%         % Try boundary approach
%         all_boundary_safe = true;
%         for i = 1:reachability_results.num_splits
%             for m = 1:reachability_results.n_methods
%                 low_b = reachability_results.succ_low_boundary{i}(:, k, m);
%                 up_b  = reachability_results.succ_up_boundary{i}(:, k, m);
%                 if any(low_b > safe_sup) || any(up_b < safe_inf)
%                     all_boundary_safe = false;
%                     break;
%                 end
%             end
%             if ~all_boundary_safe, break; end
%         end
% 
%         if all_boundary_safe
%             verified_safe = [verified_safe; x_i', r_i];
%             num_verified = num_verified + 1;
%             fprintf('  BOUNDARY METHODS SAFE → VERIFIED\n');
%         else
%             % Falsified
%             falsified_examples = [falsified_examples; x_i', r_i];
%             num_falsified = num_falsified + 1;
%             overall_safe = false;
%             fprintf('  FALSIFIED! Counterexample found.\n');
% 
%             % Optional: refine or stop
%             if r_i > refinement_threshold
%                 % Split into 4 quadrants
%                 dirs = [1 0; 0 1; -1 0; 0 -1];
%                 for d = 1:4
%                     offset = dirs(d,:) * (r_i / 2);
%                     new_center = x_i + offset';
%                     new_radius = r_i / 2;
%                     if all(new_center - new_radius >= x0 - Initial_radius) && ...
%                        all(new_center + new_radius <= x0 + Initial_radius)
%                         verification_queue(end+1) = struct('center', new_center, 'radius', new_radius);
%                     end
%                 end
%             end
%         end
%     end
% end
% 
% %% ------------------------------------------------------------------------
% % FINAL SUMMARY
% % -------------------------------------------------------------------------
% elapsed_time = toc;
% 
% fprintf('\n=== Verification Analysis Complete ===\n');
% fprintf('System: %s\n', upper(system_names(system_choice)));
% fprintf('Total Iterations: %d\n', iter);
% fprintf('Verified Subsets: %d\n', num_verified);
% fprintf('Falsified Subsets: %d\n', num_falsified);
% fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue) + num_unknown);
% if elapsed_time < 60
%     fprintf('Total Time Taken: %.2f seconds\n', elapsed_time);
% else
%     fprintf('Total Time Taken: %.2f minutes\n', elapsed_time / 60);
% end
% 
% %% ------------------------------------------------------------------------
% % FINAL PLOTS & DATA (only if flag=false AND FULLY VERIFIED)
% % -------------------------------------------------------------------------
% if ~save_figures_each_iteration && overall_safe && isempty(verification_queue)
%     fprintf('\n=== VERDICT: FULLY VERIFIED ===\n');
%     fprintf('Generating FINAL reachability plots and data (original input set)...\n');
% 
%     % Recompute CORA tube and Jacobian bounds
%     cora_full_params = cora_params;
%     cora_full_params.x0 = x0;
%     cora_full_params.Initial_radius = Initial_radius;
% 
%     if system_choice == 14
%         reachable_tube_bounds = fpa_CORA_RT(cora_full_params);
%     elseif system_choice == 15
%         reachable_tube_bounds = spiral_non_CORA_RT(cora_full_params);
%     end
% 
%     a = reachable_tube_bounds.lb;
%     b = reachable_tube_bounds.ub;
%     [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
% 
%     % Pass via global
%     global Jacobian_x_low Jacobian_x_up system_choice_global
%     Jacobian_x_low = J_lb;
%     Jacobian_x_up  = J_ub;
%     system_choice_global = system_choice;
%     UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
% 
%     % Call NODE_RA
%     node_params.x0                          = x0;
%     node_params.Initial_radius              = Initial_radius;
%     node_params.t_final                     = t_final_TIRA;
%     node_params.timeStep                    = timeStep_TIRA;
%     node_params.system_choice               = system_choice;
%     node_params.plot_last_only              = plot_last_only;
%     node_params.sample_succ_number          = sample_succ_number;
%     node_params.save_figures_each_iteration = true;   % Force save
%     node_params.safe_inf                    = safe_inf;
%     node_params.safe_sup                    = safe_sup;
% 
%     final_results = NODE_RA(node_params);
% 
%     fprintf('Final results saved in: %s\n', final_results.save_dir);
%     fprintf('  - reachability_data.mat\n');
%     fprintf('  - reachability_plot_*.png/.eps\n');
%     fprintf('  - boundary_points_*.png/.eps\n');
%     fprintf('  - interval_hull_*.png\n');
% end
% 
% %% ------------------------------------------------------------------------
% % VERDICT
% % -------------------------------------------------------------------------
% if overall_safe && isempty(verification_queue)
%     fprintf('\nVERDICT: FULLY VERIFIED (SAFE) for the initial set.\n');
% elseif ~overall_safe
%     fprintf('\nVERDICT: FALSIFIED! Counterexample found.\n');
%     if ~isempty(falsified_examples)
%         fprintf('Example: center = [%.3f, %.3f], radius = %.3f\n', ...
%             falsified_examples(1,1), falsified_examples(1,2), falsified_examples(1,3));
%     end
% elseif iter >= MAX_ITER
%     fprintf('\nVERDICT: UNKNOWN (max iterations reached).\n');
% else
%     fprintf('\nVERDICT: UNKNOWN (queue not empty).\n');
% end
% 
% %% ------------------------------------------------------------------------
% % CLEAN UP
% % -------------------------------------------------------------------------
% clear global Jacobian_x_low Jacobian_x_up system_choice_global
% 
% %------------- END OF CODE 4 --------------
