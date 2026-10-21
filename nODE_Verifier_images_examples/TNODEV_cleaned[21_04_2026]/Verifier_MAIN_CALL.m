% =========================================================================
% File: Verifier_MAIN_CALL.m
%
% Purpose:
%   Top-level entry point of the TNODEV Toolbox. Performs falsification,
%   iterative verification and refinement of Neural ODE / NNCS systems
%   using interval over-approximation (OA) methods.
%
% Description:
%   This is the only file the end-user is expected to edit. It exposes
%   a small USER CONFIGURATION block (system selection, refinement
%   heuristic, safe set, ...) and dispatches to the appropriate
%   verification pipeline:
%
%     - Pure Neural ODE systems (FPA, Spiral_non) -> internal loop here
%     - Hybrid NNCS systems     (ACC linear/nonlin) -> reach_acc_TNODEV
%
%   Available systems (system_choice):
%     14   FPA            pure NODE,   5D
%     15   Spiral_non     pure NODE,   2D
%     16   MNIST          (handled in a separate folder)
%     17   ACC nonlinear  hybrid NNCS, 8D, tanh activation
%     24   ACC linear     hybrid NNCS, 8D, no activation
%
%   Available reachability OA methods (used internally by NODE_RA_New):
%     1. Mixed-Monotonicity                (Single-step)
%     2. Sampled-Data Mixed-Monotonicity   (Single-step)
%     3. Mixed-Monotonicity                (Boundary)
%     4. Sampled-Data Mixed-Monotonicity   (Boundary)
%
%   Available refinement heuristics:
%     'naive'                          split the dimension with the largest radius
%     'most-sensitive-input-radius'    Jacobian-bound weighted radius
%     'interval-norm-gradient'         finite-difference gradient of |OA(R)|_1
%
% Prerequisites:
%   - CORA      (reachable-tube bounds for nonlinear systems)
%   - NNV 2.0   (used by the ACC hybrid NNCS pipeline)
%
% Outputs:
%   A timestamped folder ./FINAL_RESULTS_<system>_<date>/ containing
%   plots (PNG, PDF, FIG, EPS, high-res PNG) and a .mat data dump.
%
% Part of the TNODEV Toolbox
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
% =========================================================================

%------------- BEGIN CODE --------------
clear all; close all; clc;

addpath('../OA_methods')
addpath('../Utilities')
addpath('./')   % current directory (utility functions)


%% =========================================================================
%   USER CONFIGURATION  --  EDIT ONLY THIS BLOCK
%  =========================================================================

% --- 1. SYSTEM SELECTION -------------------------------------------------
% Uncomment exactly ONE of the lines below.
%
% system_choice = 14;   % FPA           (pure NODE,   5D)
% system_choice = 15;   % Spiral_non    (pure NODE,   2D)
% system_choice = 16;   % MNIST         (handled in a separate folder)
system_choice = 17;     % ACC nonlinear (hybrid NNCS, 8D, tanh activation)
% system_choice = 24;   % ACC linear    (hybrid NNCS, 8D, no activation)


% --- 2. REFINEMENT HEURISTIC ---------------------------------------------
% Options: 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
input_split_heuristic = 'naive';


%% =========================================================================
%   HYBRID NNCS DISPATCHER  (ACC: linear and nonlinear variants)
%  -------------------------------------------------------------------------
%   The ACC benchmark is a hybrid NNCS (Neural ODE plant + neural-network
%   controller). It does NOT use the per-iteration verification loop
%   defined further down in this script. Both the nonlinear (tanh) and
%   the linear ACC variants are routed through reach_acc_TNODEV, which
%   handles its own falsification, refinement and plotting.
%  =========================================================================

if ismember(system_choice, [17, 24])
    if system_choice == 17
        sys_label = 'Adaptive Cruise Control (ACC) - 8D - Nonlinear (tanh)';
    else
        sys_label = 'Adaptive Cruise Control (ACC) - 8D - Linear';
    end

    fprintf('=== Starting Hybrid NNCS Verification Example ===\n');
    fprintf('System:    %s\n',   sys_label);
    fprintf('Heuristic: %s\n\n', input_split_heuristic);

    % Make the Examples folder visible so reach_acc_TNODEV can locate
    % its system-specific .mat files (controller weights, etc.)
    addpath(genpath(fullfile(pwd, 'Examples')));

    % Dispatch (reach_acc_TNODEV handles its own plotting and saving)
    reach_acc_TNODEV(input_split_heuristic, system_choice);

    % Stop execution here: ACC has its own end-to-end pipeline
    return;
end


%% =========================================================================
%   PURE NEURAL ODE SETUP  (FPA / Spiral_non)
%  -------------------------------------------------------------------------
%   Everything below configures the pure-NODE verification pipeline used
%   for system_choice 14 (FPA) and 15 (Spiral_non).
%  =========================================================================

node_params.input_split_heuristic = input_split_heuristic;


% --- 1. INITIAL SET ------------------------------------------------------

% Spiral_non (2D)
x0_spiral       = [2.0; 0.0];
radius_spiral   = 0.2;
timeStep_spiral = 0.01;        % CORA and TIRA time step
tFinal_spiral   = 1.0;         % CORA and TIRA final time

% FPA (5D)
x0_fpa       = [0; -0.58587; 0.8; 0.52323; 0.7];
radius_fpa   = 0.01;
timeStep_fpa = 0.05;
tFinal_fpa   = 2.0;

% Auto-select based on system_choice
if system_choice == 15            % Spiral_non
    x0             = x0_spiral;
    Initial_radius = radius_spiral * ones(length(x0), 1);
    system_name    = 'spiral_non';
    cora_params.timeStep = timeStep_spiral;
    cora_params.tFinal   = tFinal_spiral;
    t_init_TIRA   = 0;
    t_final_TIRA  = tFinal_spiral;
    timeStep_TIRA = timeStep_spiral;

elseif system_choice == 14        % FPA
    x0             = x0_fpa;
    Initial_radius = radius_fpa * ones(length(x0), 1);
    system_name    = 'fpa';
    cora_params.timeStep = timeStep_fpa;
    cora_params.tFinal   = tFinal_fpa;
    t_init_TIRA   = 0;
    t_final_TIRA  = tFinal_fpa;
    timeStep_TIRA = timeStep_fpa;

else
    error('Invalid system_choice: %d. Use 14 (FPA), 15 (Spiral_non), or 17 / 24 (ACC).', ...
          system_choice);
end


% --- 2. CORA PARAMETERS (reachable-tube bounds) --------------------------
cora_params.taylorTerms   = 2;        % Taylor terms for nonlinear approximation
cora_params.zonotopeOrder = 20;       % Zonotope order for set representation
cora_params.alg           = 'lin';    % Linear approximation
cora_params.tensorOrder   = 2;        % --> "check what will happen if I increase this"


% --- 3. TIRA PARAMETERS (NODE_RA_New) ------------------------------------
node_params.system_choice      = system_choice;
node_params.x0                 = x0;
node_params.Initial_radius     = Initial_radius;
node_params.t_final            = t_final_TIRA;
node_params.timeStep           = timeStep_TIRA;
node_params.plot_last_only     = true;
node_params.sample_succ_number = 1000;   % --> check PJ comment about using the falsification to determine this number


% --- 4. SAFE SET (USER-ADJUSTABLE) ---------------------------------------
% --> Remove these examples at the end and leave it for the user to define
%     it, or keep just one default SAFE SET and mention next to it that it
%     is adjustable.

% Make sure the safe-set dimension matches the system dimension
n_sys = length(x0);

if system_choice == 15            % Spiral_non (2D)
    % --- Default ----------------------------------------------------
    %   Example 4: Safety violated for some methods.
    %   To run a different scenario, COMMENT OUT the two lines below and
    %   UNCOMMENT exactly one alternative from the block underneath.
    node_params.safe_inf = [-2.0; 1.2];
    node_params.safe_sup = [ 0.1; 5.0];

    % --- Alternative examples (uncomment one to activate) ----------
    %
    % Example 1: Safety verified for all OA methods
    %   node_params.safe_inf = [-1.8; 0.6];
    %   node_params.safe_sup = [ 0.7; 2.8];
    %
    % Example 2: Safety violated for some methods
    %   node_params.safe_inf = [-1.30; 0.95];
    %   node_params.safe_sup = [ 0.25; 2.45];
    %
    % Example 3: Safety violated for some methods
    %   node_params.safe_inf = [-1.0; 1.2];
    %   node_params.safe_sup = [ 0.3; 2.0];
    %
    % Example 5: Safety violated -- counter-example works here
    %   node_params.safe_inf = [-3.0; 1.5];
    %   node_params.safe_sup = [ 0.0; 3.0];

elseif system_choice == 14        % FPA (5D)
    % --- Default ----------------------------------------------------
    %   FPA Example 6 (PJ): safe without refinement (the simplest case).
    %   To run a different scenario, COMMENT OUT the two lines below and
    %   UNCOMMENT exactly one alternative from the block underneath.
    node_params.safe_inf = [-1.3; -0.7; -0.9; 0.3; 2.3];
    node_params.safe_sup = [-1.2; -0.5; -0.7; 0.4; 2.5];

    % --- Alternative examples (uncomment one to activate) ----------
    %
    % Half-space style: finite x1-x2, unbounded elsewhere.
    % Goes into verification but cannot conclude (only x1-x2 are finite).
    %   node_params.safe_inf = [-1.285; -2;  -Inf; -Inf; -Inf];
    %   node_params.safe_sup = [-0.5;    1;   Inf;  Inf;  Inf];
    %
    % Wide axis-aligned box
    %   node_params.safe_inf = [-1.4; -10; -10; -10; -10];
    %   node_params.safe_sup = [10;    10;  10;   0;  10];
    %
    %   node_params.safe_inf = [-0; -2; -2; 0; -2];
    %   node_params.safe_sup = [-2;  2;  2; 0;  2];
    %
    % Falsified example (intentionally violated)
    %   node_params.safe_inf = [-10; -10; -10; -10; -10];
    %   node_params.safe_sup = [ 10;  10;  10;   0;  10];
    %
    % 1 refinement only
    %   node_params.safe_inf = [-1.30; -0.70; -0.75; 0.35; 2.38];
    %   node_params.safe_sup = [-1.20; -0.55; -0.72; 0.38; 2.48];
    %
    % Refinement iteration ex.1 -- WORKING
    %   naive                       : ~2.07 minutes
    %   most-sensitive-input-radius : ~2.23 minutes
    %   interval-norm-gradient      : ~59.75 seconds
    %   node_params.safe_inf = [-1.29; -0.70; -0.75; 0.35; 2.38];
    %   node_params.safe_sup = [-1.23; -0.59; -0.72; 0.38; 2.46];
    %
    % Refinement iteration ex.2 -- WORKING
    %   naive                       : ~35 minutes  (343 iterations)
    %   most-sensitive-input-radius : ~39.92 min   (343 iterations)
    %   interval-norm-gradient      : did not converge after >12 h / 1850+ iters
    %   node_params.safe_inf = [-1.29; -0.68; -0.74; 0.35; 2.38];
    %   node_params.safe_sup = [-1.23; -0.59; -0.72; 0.37; 2.45];
end

% Validate dimensions
if length(node_params.safe_inf) ~= n_sys
    error('Dimension mismatch: Safe set has %d dims, but system has %d.', ...
          length(node_params.safe_inf), n_sys);
end


% --- 5. VERIFICATION-LOOP PARAMETERS -------------------------------------
MAX_ITER = 5000;     % --> "check this criterion: either we conclude based on
                     %      that or a time-out with a defined time, e.g. 45 min"


% --- 6. PLOTTING CONTROL -------------------------------------------------
% --> Re-check cause of big changes with V1.2 Origine
plot_control.show_figures    = true;     % display figures during execution
plot_control.save_final_plot = true;     % save final figures + .mat to disk

% Style for the verified-subsets plot:
%   'all'        -> draw individual reachable sets for every verified subset (detailed)
%   'union'      -> draw the interval-hull union of the verified reachable subsets (clean)
%   'true_union' -> draw the exact polygonal union of the verified reachable subsets (cleanest)
plot_control.verified_set_style = 'all';


% =========================================================================
fprintf('=== Starting Iterative Verification Analysis ===\n');
fprintf('System: %s (choice=%d)\n', system_name, system_choice);
fprintf('Initial set: center = [%s], radius = %.3f\n', ...
        strjoin(arrayfun(@(x) sprintf('%.3f', x), x0, 'UniformOutput', false), ', '), ...
        max(Initial_radius));
fprintf('TIRA: t = [%.2f, %.2f], step = %.3f\n', t_init_TIRA, t_final_TIRA, timeStep_TIRA);
fprintf('CORA: tFinal = %.2f\n\n', cora_params.tFinal);


%% =========================================================================
%   INITIALIZATION
%  =========================================================================

tic;     % global timer (verdict time only, not plotting)

% Queue of input sets to verify: { {x_center, radius}, ... }
verification_queue = {{x0, Initial_radius}};

% Result counters
num_verified  = 0;
num_falsified = 0;
num_unknown   = 1;

% Global safety flag and falsification artefacts
overall_safe            = true;
falsified_example       = [];
falsified_initial_point = [];
verified_initial_sets   = {};   % stores {center, radius, intersect_low, intersect_up} per verified subset


% --- PART 1: Pre-compute the overall reachable-tube bounds (CORA) --------
fprintf('PART 1: Computing Overall Reachable Tube Bounds (CORA)...\n');
cora_full_params                = cora_params;
cora_full_params.x0             = x0;
cora_full_params.Initial_radius = Initial_radius;

reachable_tube_bounds = compute_CORA_Reachable_Tube(cora_full_params, system_choice);
fprintf('PART 1 completed.\n\n');


% --- PART 2: Pre-compute element-wise Jacobian bounds --------------------
fprintf('PART 2: Computing Jacobian Bounds...\n');
a = reachable_tube_bounds.lb;
b = reachable_tube_bounds.ub;
[J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice);
fprintf('PART 2 completed.\n\n');


% --- PART 3: Push Jacobians + system_choice into globals -----------------
% These globals are read by the UP_* sensitivity / Jacobian routines.
fprintf('PART 3: Passing Jacobian bounds via globals...\n');
global Jacobian_x_low Jacobian_x_up system_choice
Jacobian_x_low = J_lb;
Jacobian_x_up  = J_ub;
system_choice  = system_choice;   % preserved on purpose (interaction with global)

[~, ~, ~, ~] = UP_Jacobian_Bounds(t_init_TIRA, t_final_TIRA, J_lb, J_ub, [], []);
fprintf('PART 3 completed.\n\n');


%% =========================================================================
%   MAIN VERIFICATION LOOP  (1. Falsification, 2. Verification, 3. Refinement)
%  =========================================================================

% --- PART 4: Initial falsification attempt -------------------------------
fprintf('PART 4: Initial Falsification Attempt...\n');
check_dims = length(node_params.safe_inf);

% Sample uniformly inside the FULL initial input set
num_falsification_samples = 1000;
initial_samples = node_params.x0 - node_params.Initial_radius + ...
                  2 * node_params.Initial_radius .* ...
                      rand(length(node_params.x0), num_falsification_samples);

% Get the active ODE solver from the project-wide solver settings
final_samples = zeros(check_dims, num_falsification_samples);
global g_sensitivity_bounds_method;
parameters  = get_solver_parameters(length(node_params.x0), 0, g_sensitivity_bounds_method);
ode_solver  = parameters.ode_solver;
ode_options = parameters.ode_options;

% System_description reads system_choice from the global scope, so we
% redeclare it here before constructing the closure below.
global system_choice;

% Simulate every initial sample to t_final
for i = 1:num_falsification_samples
    x0_rand = initial_samples(:, i);
    [~, x_traj] = ode_solver(@(t,x) System_description(t, x, []), ...
                             [t_init_TIRA, t_final_TIRA], x0_rand, ode_options);
    final_samples(:, i) = x_traj(end, 1:check_dims)';
end

% Counter-example check: any final sample outside the safe set?
violation_indices = find(any(final_samples < node_params.safe_inf | ...
                             final_samples > node_params.safe_sup, 1));

if ~isempty(violation_indices)
    first_violation_idx     = violation_indices(1);
    overall_safe            = false;
    falsified_example       = final_samples(:, first_violation_idx);
    falsified_initial_point = initial_samples(:, first_violation_idx);
    fprintf('  FALSIFIED! Counter-example found from initial sampling.\n');
    fprintf('  Initial point: [%s]\n', num2str(falsified_initial_point', '%.4f '));
    fprintf('  Final point:   [%s]\n', num2str(falsified_example',       '%.4f '));
else
    fprintf('  No counter-example found in initial falsification.\n');
end
fprintf('PART 4 completed.\n\n');


% --- PART 5: Verification + refinement loop ------------------------------
fprintf('PART 5: Verification and Refinement based Heuristics Loop...\n');

iter = 0;
if overall_safe   % only enter the loop if we have not already falsified
    while ~isempty(verification_queue) && iter < MAX_ITER
        iter = iter + 1;
        fprintf('--- ITERATION %d: Queue size = %d ---\n', iter, length(verification_queue));

        % Pop the next input set from the queue
        current_set = verification_queue{1};
        verification_queue(1) = [];
        x_i = current_set{1};
        r_i = current_set{2};

        % Normalise r_i to a column vector matching x_i
        if isscalar(r_i)
            r_i = r_i * ones(length(x_i), 1);
        else
            r_i = r_i(:);
        end

        fprintf('  Calling NODE_RA_New for set with center [%s] and radius %.4f\n', ...
                num2str(x_i', '%.4f '), max(r_i));

        % Build the per-subset NODE_RA_New input
        node_params_subset                = node_params;
        node_params_subset.x0             = x_i;
        node_params_subset.Initial_radius = r_i;

        % Call NODE_RA_New on this subset
        reachability_results = NODE_RA_New(node_params_subset);


        % --- 1. Verification check -----------------------------------
        % Check whether the intersection of all 4 OAs is contained in
        % the safe set. The intersection is now computed inside
        % NODE_RA_New (avoids recomputation here).
        intersect_low = reachability_results.intersect_low;
        intersect_up  = reachability_results.intersect_up;

        low_ok = all(intersect_low >= node_params.safe_inf);
        up_ok  = all(intersect_up  <= node_params.safe_sup);

        if low_ok && up_ok
            fprintf('  PROVEN SAFE: Intersection of all 4 OAs is within the safe set.\n');
            % Store {center, radius, intersect_low, intersect_up} so the
            % final plotting stage can reuse the bounds without recomputing
            verified_initial_sets{end+1} = {x_i, r_i, intersect_low, intersect_up};
            num_verified = num_verified + 1;
            num_unknown  = num_unknown - 1;
            continue;
        end

        fprintf('  NOT PROVEN: Intersection of all 4 OAs violates safe set. Refining...\n');


        % --- 2. Refinement: pick a dimension to split ----------------
        split_dim = [];
        switch node_params.input_split_heuristic
            case 'naive'
                % Split the dimension with the largest current radius
                [max_radius, split_dim] = max(r_i);
                fprintf('  Refinement: Using naive heuristic, splitting dimension %d (radius %.4e).\n', ...
                        split_dim, max_radius);

            case 'most-sensitive-input-radius'
                % Score = local Jacobian magnitude weighted by input radius
                try
                    [J_lb_loc, J_ub_loc] = Jacob_bounds_element_wise(x_i - r_i, x_i + r_i, system_choice);

                    absJ = max(abs(J_lb_loc), abs(J_ub_loc));   % (n x n)
                    sens = max(absJ, [], 1);                    % (1 x n)
                    sens = max(sens, 1e-10);                    % avoid zero

                    scores = r_i .* sens;
                    [~, split_dim] = max(scores);

                    fprintf(' Refinement: Using most-sensitive-input-radius heuristic, splitting dimension %d (score = %.4e)\n', ...
                            split_dim, max(scores));    % Check if I can remove max(scores)

                catch ME
                    [~, split_dim] = max(r_i);
                    fprintf(' Refinement: Fallback to naive heuristic (Jacob failed: %s), splitting dimension %d\n', ...
                            ME.message, split_dim);
                end

            case 'interval-norm-gradient'
                % Numerical gradient of |R(x)|_1 w.r.t. each input dimension
                try
                    base_norm = sum(intersect_up - intersect_low);
                    if base_norm <= 0
                        throw(MException('', 'Base reachable set has zero or negative width'));   % Re-check this exception again
                    end

                    grad  = zeros(size(x_i));
                    delta = 1e-6;

                    for j = 1:length(x_i)
                        x_pert    = x_i;
                        x_pert(j) = x_i(j) + delta;

                        node_params.x0             = x_pert;
                        node_params.Initial_radius = r_i;
                        pert_results = NODE_RA_New(node_params);

                        p_low = pert_results.intersect_low;
                        p_up  = pert_results.intersect_up;

                        pert_norm = sum(p_up - p_low);
                        grad(j)   = (pert_norm - base_norm) / delta;
                    end

                    heuristic = abs(grad);
                    if any(isnan(heuristic) | isinf(heuristic) | heuristic <= 0)
                        throw(MException('', 'Invalid gradient'));
                    end

                    [~, split_dim] = max(heuristic);
                    fprintf(' Refinement: Using interval-norm-gradient heuristic, splitting dimension %d (grad = %.4e)\n', ...
                            split_dim, max(heuristic));     % Check if I can remove max(heuristic)

                catch ME
                    [~, split_dim] = max(r_i);
                    fprintf(' Refinement: Fallback to naive heuristic (gradient failed: %s), splitting dimension %d\n', ...
                            ME.message, split_dim);
                end

            otherwise
                error('Invalid input_split_heuristic: %s', node_params.input_split_heuristic);
        end


        % --- 3. Perform the split (replace the parent set by 2 children)
        r_new            = r_i;
        r_new(split_dim) = r_i(split_dim) / 2;

        x_c1            = x_i;
        x_c1(split_dim) = x_i(split_dim) - r_i(split_dim) / 2;

        x_c2            = x_i;
        x_c2(split_dim) = x_i(split_dim) + r_i(split_dim) / 2;

        verification_queue{end+1} = {x_c1, r_new};
        verification_queue{end+1} = {x_c2, r_new};

        fprintf('  Set REFINED: Split dimension %d into two new sets.\n', split_dim);
        num_unknown = num_unknown - 1 + 2;
    end
end
fprintf('PART 5 completed.\n\n');

% Stop the verdict timer here: plotting time is reported separately and
% does not affect the verification verdict.
elapsed_time = toc;


%% =========================================================================
%   PART 6 : FINAL PLOTTING AND SAVING
%  =========================================================================

if plot_control.save_final_plot
    fprintf('PART 6: Generating Final_Entire_input_set_Verification_Plot and Saving Data...\n');

    % --- Create the timestamped results folder (one per run) ------------
    folder_name = sprintf('FINAL_RESULTS_%s_%s', system_name, datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
    save_dir    = fullfile(pwd, folder_name);
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    fprintf('  Created results folder:\n    %s\n', save_dir);

    % --- Recompute the OA on the FULL initial set for the final figure --
    % --> Maybe check if I can pass the one directly from above !!
    node_params_full                = node_params;
    node_params_full.x0             = x0;
    node_params_full.Initial_radius = Initial_radius;
    res_full = NODE_RA_New(node_params_full);

    % --- Choose which 2D projections to plot ----------------------------
    if system_choice == 14            % FPA (5D): three projections
        plot_pairs = {[1 2], [3 4], [4 5]};
    else                              % default: only (x1, x2)
        plot_pairs = {[1 2]};
    end


    % =====================================================================
    %  Loop over the requested 2D projections
    % =====================================================================
    for p_idx = 1:length(plot_pairs)
        dims = plot_pairs{p_idx};
        px = dims(1);
        py = dims(2);

        fprintf('  Generating Figure Set %d for x%d vs x%d...\n', p_idx, px, py);

        % Unrefined OA (rectangle in the chosen projection)
        unrefined_low = res_full.intersect_low;
        unrefined_up  = res_full.intersect_up;
        rect_unrefined = [unrefined_low(px), unrefined_low(py), ...
                          unrefined_up(px) - unrefined_low(px), ...
                          unrefined_up(py) - unrefined_low(py)];

        % Safe-set rectangle (clamped to finite values for plotting)
        % --> Main usage is to ensure X_s is plotted correctly by MATLAB,
        %     even if your safety definition includes Inf.
        % /--> I will probably remove it later as it adds complexity.
        sx_inf = max(node_params.safe_inf(px));   sx_sup = min(node_params.safe_sup(px));
        sy_inf = max(node_params.safe_inf(py));   sy_sup = min(node_params.safe_sup(py));
        safe_x = [sx_inf, sx_sup, sx_sup, sx_inf, sx_inf];
        safe_y = [sy_inf, sy_inf, sy_sup, sy_sup, sy_inf];


        % =================================================================
        % FIGURE 1 : Verified RS union  vs.  Entire Initial input set OA
        % =================================================================

        fig = figure('Color', 'w', 'Visible', plot_control.show_figures, 'Position', [100 100 900 700]);
        hold on; grid on; box on;
        set(gca, 'FontSize', 18, 'TickLabelInterpreter', 'latex');
        k = 1;        % final time index

        % 1. Sampled trajectories from the FULL initial set
        plot(final_samples(px,:), final_samples(py,:), 'k.', 'MarkerSize', 5, ...
             'DisplayName', 'Sampled Trajectories');

        % 2. Unrefined / Un-VERIFIED reachable set (red dashed)
        rectangle('Position', rect_unrefined, ...
                  'EdgeColor', [0.85 0.33 0.1], 'LineWidth', 2, 'LineStyle', '--');
        h_unrefined = plot(NaN, NaN, '--', 'Color', [0.85 0.33 0.1], 'LineWidth', 2, ...
                           'DisplayName', 'Unrefined OA (Entire Set)');

        % 3. Verified subsets (purple)
        h_int = [];

        % --> check which initialisation is correct for the union variables
        % union_low = Inf(length(x0), 1);
        % union_up  = -Inf(length(x0), 1);
        union_low = [Inf;  Inf];      % 2D projections
        union_up  = [-Inf; -Inf];

        % polyshape accumulator for the 'true_union' style
        total_union_poly = polyshape();

        if ~isempty(verified_initial_sets)
            fprintf('  Computing reachable sets for %d verified subsets...\n', length(verified_initial_sets));

            for i = 1:length(verified_initial_sets)
                set_info = verified_initial_sets{i};
                int_low  = set_info{3};
                int_up   = set_info{4};

                switch plot_control.verified_set_style
                    case 'all'
                        % Plot every subset individually
                        rect_dims = [int_low(px), int_low(py), ...
                                     int_up(px) - int_low(px), int_up(py) - int_low(py)];
                        rectangle('Position', rect_dims, ...
                                  'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);

                    case 'true_union'
                        % Accumulate exact polygonal union
                        x_corners = [int_low(px), int_up(px),  int_up(px),  int_low(px)];
                        y_corners = [int_low(py), int_low(py), int_up(py),  int_up(py)];
                        new_poly  = polyshape(x_corners, y_corners);
                        total_union_poly = union(total_union_poly, new_poly);

                    otherwise   % 'union' / interval hull
                        union_low = min(union_low, [int_low(px); int_low(py)]);
                        union_up  = max(union_up,  [int_up(px);  int_up(py)]);
                end
            end

            % Draw the union (only relevant for 'union' / 'true_union')
            if strcmp(plot_control.verified_set_style, 'union')
                rect_dims = [union_low(1), union_low(2), ...
                             union_up(1) - union_low(1), union_up(2) - union_low(2)];
                rectangle('Position', rect_dims, ...
                          'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 2);

            elseif strcmp(plot_control.verified_set_style, 'true_union')
                h_poly = plot(total_union_poly, ...
                              'FaceColor', [0.4940 0.1840 0.5560], 'FaceAlpha', 0.6, ...
                              'EdgeColor', 'none');
            end

            % Dummy handle for the legend
            switch plot_control.verified_set_style
                case 'union'
                    legend_name = 'Union (Interval Hull) of Verified Reachable Sets';
                case 'true_union'
                    legend_name = 'True Union of Verified Reachable Sets';
                otherwise
                    legend_name = 'Verified Reachable Sets';
            end
            h_int = plot(NaN, NaN, '-', 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2, ...
                         'DisplayName', legend_name);
        end

        % 4. Safe set (green)
        h_safe = plot(safe_x, safe_y, 'Color', [0 0.8 0], 'LineWidth', 2, ...
                      'DisplayName', 'Safe Set');

        % 5. Counter-example (only if falsification succeeded)
        h_fals = [];
        if ~overall_safe
            plot(falsified_example(px), falsified_example(py), 'rX', ...
                 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Counter-Example');
        end

        axis equal;
        xlabel(['$x_' num2str(px) '$'], 'Interpreter', 'latex', 'FontSize', 20);
        ylabel(['$x_' num2str(py) '$'], 'Interpreter', 'latex', 'FontSize', 20);
        title(['Reachable set OA vs Verified Safe Subsets ($x_' num2str(px) ', x_' num2str(py) '$)'], ...
              'Interpreter', 'latex', 'FontSize', 18);

        % --- Legend assembly --------------------------------------------
        legend_entries = h_unrefined;
        legend_labels  = {'Entire Initial input set'};

        if ~isempty(h_int)
            legend_entries(end+1) = h_int;
            legend_labels{end+1}  = 'Union of verified subsets OAs';
        end

        legend_entries(end+1) = h_safe;
        legend_labels{end+1}  = 'Safe Set';

        if ~overall_safe && ~isempty(h_fals)
            legend_entries(end+1) = h_fals;
            legend_labels{end+1}  = 'Counter-Example';
        end

        legend(legend_entries, legend_labels, 'Location', 'northeast', 'Interpreter', 'latex');
        set(legend, 'FontSize', 10, 'FontWeight', 'bold', 'Box', 'on');

        % --- Save Figure 1 ----------------------------------------------
        filename_str         = sprintf('Final_Entire_input_set_Verification_Plot_x%d_x%d', px, py);
        plot_filename_entire = fullfile(save_dir, filename_str);

        saveas(gcf, [plot_filename_entire, '.png']);
        saveas(gcf, [plot_filename_entire, '.pdf']);
        saveas(gcf, [plot_filename_entire, '.fig']);
        print(gcf, [plot_filename_entire, '.eps'], '-depsc');             % vector EPS for LaTeX
        print(gcf, [plot_filename_entire, '_highres'], '-dpng', '-r600'); % 600 DPI PNG
        fprintf('Final_Entire_input_set_Verification_Plots saved as (PNG, PDF, FIG, EPS, high-res PNG) in:\n %s\n', ...
                plot_filename_entire);


        % =================================================================
        % FIGURE 2 : Refined initial subsets  vs.  resulting Verified OAs
        % =================================================================

        fig = figure('Color', 'w', 'Visible', plot_control.show_figures, 'Position', [100 100 900 700]);
        hold on; grid on; box on;
        set(gca, 'FontSize', 18, 'TickLabelInterpreter', 'latex');

        % 1. [INPUT] Verified initial subsets (filled blue)
        if ~isempty(verified_initial_sets)
            for i = 1:length(verified_initial_sets)
                set_info = verified_initial_sets{i};
                center   = set_info{1};
                radius   = set_info{2};
                rect_pos = [center(px) - radius(px), center(py) - radius(py), ...
                            2 * radius(px), 2 * radius(py)];
                rectangle('Position', rect_pos, ...
                          'EdgeColor', 'b', 'FaceColor', [0.8 0.8 1]);
            end
        end
        h_ver_init = patch(NaN, NaN, 'b', 'FaceAlpha', 0.3, ...
                           'DisplayName', 'Verified Initial Subsets');

        % 2. [INPUT] Original full initial input set boundary (dashed black)
        initial_inf  = x0 - Initial_radius;
        initial_dims = (x0 + Initial_radius) - initial_inf;
        rectangle('Position', [initial_inf(px), initial_inf(py), ...
                               initial_dims(px), initial_dims(py)], ...
                  'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
        h_init = plot(NaN, NaN, 'k--', 'LineWidth', 2, ...
                      'DisplayName', 'Original Initial Set');

        % 3. [OUTPUT] Unrefined reachable set (dashed red)
        rectangle('Position', rect_unrefined, ...
                  'EdgeColor', [0.85 0.33 0.1], 'LineWidth', 2, 'LineStyle', '--');
        h_unrefined_2 = plot(NaN, NaN, '--', 'Color', [0.85 0.33 0.1], 'LineWidth', 2, ...
                             'DisplayName', 'Entire Initial input set');

        % 4. [OUTPUT] Verified reachable sets (purple)
        % Reuse the union polyshape / per-subset boxes already computed above
        if ~isempty(verified_initial_sets)
            switch plot_control.verified_set_style
                case 'true_union'
                    h_poly_2 = plot(total_union_poly, ...
                                    'FaceColor', [0.4940 0.1840 0.5560], 'FaceAlpha', 0.6, ...
                                    'EdgeColor', 'none');

                case 'all'
                    for i = 1:length(verified_initial_sets)
                        set_info = verified_initial_sets{i};
                        int_low  = set_info{3};
                        int_up   = set_info{4};
                        rect_dims = [int_low(px), int_low(py), ...
                                     int_up(px) - int_low(px), int_up(py) - int_low(py)];
                        rectangle('Position', rect_dims, ...
                                  'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
                    end

                otherwise   % 'union' / interval hull
                    rect_dims = [union_low(1), union_low(2), ...
                                 union_up(1) - union_low(1), union_up(2) - union_low(2)];
                    rectangle('Position', rect_dims, ...
                              'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 2);
            end
        end
        h_ver_rs_2 = plot(NaN, NaN, '-', 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2, ...
                          'DisplayName', 'Verified Reachable Sets');

        % Sampled trajectories from the FULL initial set
        plot(final_samples(px,:), final_samples(py,:), 'k.', 'MarkerSize', 5, ...
             'DisplayName', 'Sampled Trajectories');

        % 5. Safe set (green)
        h_safe_2 = plot(safe_x, safe_y, 'Color', [0 0.8 0], 'LineWidth', 2, ...
                        'DisplayName', 'Safe Set');

        % 6. Falsified points (input + output)
        h_fals_in  = [];
        h_fals_out = [];
        if ~overall_safe
            h_fals_in  = plot(falsified_initial_point(1), falsified_initial_point(2), ...
                              'bx', 'MarkerSize', 12, 'LineWidth', 2, ...
                              'DisplayName', 'Falsifying Input');
            h_fals_out = plot(falsified_example(1), falsified_example(2), ...
                              'rX', 'MarkerSize', 12, 'LineWidth', 2, ...
                              'DisplayName', 'Falsifying Output');
        end

        axis equal;
        xlabel(['$x_' num2str(px) '$'], 'Interpreter', 'latex', 'FontSize', 20);
        ylabel(['$x_' num2str(py) '$'], 'Interpreter', 'latex', 'FontSize', 20);
        title(['Global View: Refined Initial input set \& Resulting Refined Safe subsets OAs ($x_' ...
               num2str(px) ', x_' num2str(py) '$)'], 'Interpreter', 'latex', 'FontSize', 18);

        % Legend 2
        legend_entries_2 = [h_init, h_ver_init, h_unrefined_2, h_ver_rs_2, h_safe_2];
        legend_labels_2  = {'Original Initial input Set', 'Verified Initial Subsets', ...
                            'Entire Initial input set OA', ...
                            'Union of verified subsets OAs', 'Safe Set'};

        if ~overall_safe
            if ~isempty(h_fals_in)
                legend_entries_2(end+1) = h_fals_in;
                legend_labels_2{end+1}  = 'Falsifying Input';
            end
            if ~isempty(h_fals_out)
                legend_entries_2(end+1) = h_fals_out;
                legend_labels_2{end+1}  = 'Falsifying Output';
            end
        end

        legend(legend_entries_2, legend_labels_2, 'Location', 'northeast', 'Interpreter', 'latex');
        set(legend, 'FontSize', 10, 'FontWeight', 'bold', 'Box', 'on');

        % --- Save Figure 2 ----------------------------------------------
        filename_str          = sprintf('Refined_Safe_Subsets_Plot_x%d_x%d', px, py);
        plot_filename_refined = fullfile(save_dir, filename_str);

        saveas(gcf, [plot_filename_refined, '.png']);
        saveas(gcf, [plot_filename_refined, '.pdf']);
        saveas(gcf, [plot_filename_refined, '.fig']);
        print(gcf, [plot_filename_refined, '.eps'], '-depsc');              % vector EPS for LaTeX
        print(gcf, [plot_filename_refined, '_highres'], '-dpng', '-r600');  % 600 DPI PNG
        fprintf('Refined_Safe_Subsets_Plots saved as (PNG, PDF, FIG, EPS, high-res PNG) in:\n %s\n', ...
                plot_filename_refined);
    end

    % --- Save the per-run .mat data dump --------------------------------
    filename_base = sprintf('reachability_data_sys%d_%s', system_choice, datestr(now, 'yyyy-mm-dd'));

    save(fullfile(save_dir, [filename_base '.mat']), ...
         'verified_initial_sets', ...
         'falsified_example', 'falsified_initial_point', ...
         'final_samples', ...
         'res_full', ...
         'overall_safe', '-v7.3');

    fprintf('Final .mat data saved to: %s.mat\n', filename_base);

    % Also save a simpler summary file --> ##Check how to save it as .txt file
    % save(fullfile(save_dir, 'verification_summary.mat'), ...
    %      'verified_initial_sets', 'falsified_example', ...
    %      'falsified_initial_point', 'verdict_string', '-v7.3');
    % fprintf('  Saved summary to: verification_summary.mat\n');

    fprintf('PART 6 completed.\n\n');
end


%% =========================================================================
%   FINAL VERDICT & SUMMARY
%  =========================================================================

clear global Jacobian_x_low Jacobian_x_up system_choice_global

% --- Final verdict -------------------------------------------------------
is_fully_verified = false;
verdict_string    = '';

if ~overall_safe
    verdict_string = 'FALSIFIED! Counter-example found.';
elseif isempty(verification_queue)
    % Empty queue + no falsification = the entire input set is safe
    is_fully_verified = true;
    verdict_string    = 'FULLY VERIFIED (SAFE) for the initial input set.';
else
    % Non-empty queue: we hit MAX_ITER, i.e. timeout
    verdict_string = sprintf('UNKNOWN (Timeout: %d subsets remain unverified).', ...
                             length(verification_queue));
end


% --- Final summary -------------------------------------------------------
fprintf('\n\n=================================================================\n');
fprintf('               Verification Analysis Complete\n');
fprintf('=================================================================\n\n');
fprintf('--- Overall Summary ---\n');
fprintf('System:                    %s\n', upper(system_name));
fprintf('Total Iterations:          %d\n', iter);
fprintf('Verified Subsets:          %d\n', num_verified);
fprintf('Falsified Subsets:         %d\n', num_falsified);
fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue));   % Re-check if this is reading the correct number

% Print elapsed time in seconds (<60 s) or minutes (>=60 s) for readability
if elapsed_time < 60
    fprintf('Total Time Taken:          %.2f seconds\n', elapsed_time);
else
    fprintf('Total Time Taken:          %.2f minutes\n', elapsed_time / 60);
end

fprintf('\nVERDICT: %s\n', verdict_string);
if ~overall_safe
    fprintf('  Initial point: [%s]\n  Final point:   [%s]\n', ...
            strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_initial_point, 'UniformOutput', false), ', '), ...
            strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_example,       'UniformOutput', false), ', '));
end

fprintf('\n=================================================================\n');

%------------- END OF CODE --------------
