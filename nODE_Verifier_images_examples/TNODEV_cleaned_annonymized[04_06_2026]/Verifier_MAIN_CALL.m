% =========================================================================
% File: Verifier_MAIN_CALL.m
%
% Purpose:
%   Top-level entry point of the TNODEV Toolbox. Performs falsification,
%   verification and refinement of Pure neural ODE / neural ODE control
%   systems, and supports GNODEs.
%
% Description:
%   This is the only file the end-user is expected to edit. It exposes
%   a small USER CONFIGURATION block (system selection, refinement
%   heuristic, safe set, ...) and dispatches to the appropriate
%   verification pipeline:
%
%     - Pure neural ODE systems (Spiral, FPA, Cartpole)
%     - Hybrid neural ODE control systems (ACC) 
%     - GNODE Classification (MNIST)
%
%
%   Available systems:             
%     Spiral_linear
%     Spiral_non 
%     FPA 
%     Cartpole   
%     ACC linear      
%     ACC nonlinear
%     MNIST  
%
%   Available refinement heuristics:
%     'naive'                          
%     'most-sensitive-input-radius'    
%     'interval-norm-gradient'         
%
% Prerequisites:
%   - CORA      (used for reachable-tube for Pure neural ODE)
%   - NNV 2.0   (used for Hybrid neural ODE + GNODE)
%
% Part of the TNODEV Toolbox
% Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
clear all; close all; clc;

this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, 'OA_methods'));
addpath(fullfile(this_dir, 'Utilities'));
addpath(this_dir);   % current directory 


%% =========================================================================
%   USER CONFIGURATION  --  EDIT ONLY THIS BLOCK
%  =========================================================================

% --- 1. SYSTEM SELECTION -------------------------------------------------
% Uncomment exactly ONE of the lines below.
%
% system_choice = 21;   % Spiral_linear (pure nODE,   2D)
% system_choice = 15;   % Spiral_non    (pure nODE,   2D)
% system_choice = 14;   % FPA           (pure nODE,   5D)
% system_choice = 22;   % Cartpole      (pure nODE,   12D)
% system_choice = 17;   % ACC nonlinear (hybrid nODE, 8D, tanh activation)
% system_choice = 24;   % ACC linear    (hybrid nODE, 8D, no activation)
system_choice = 16;   % MNIST         (GNODE)

% --- 2. REFINEMENT HEURISTIC ---------------------------------------------
% Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
input_split_heuristic = 'most-sensitive-input-radius';

%% =========================================================================
%   HYBRID neural ODE Control System DISPATCHER  (ACC)
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
    addpath(genpath(fullfile(this_dir, 'Examples')));

    % Dispatch (reach_acc_TNODEV handles its own plotting and saving)
    reach_acc_TNODEV(input_split_heuristic, system_choice);

    % Stop execution here as ACC has its own end-to-end pipeline
    return;
end


%% =========================================================================
%   GNODE CLASSIFICATION DISPATCHER  (MNIST)
%  =========================================================================

if system_choice == 16
    fprintf('=== Starting MNIST GNODE Classification Verification ===\n');

    % Which network(s) to run: 'tiny' (CNODE_S) | 'medium' (CNODE_M) | 'both'
    mnist_network = 'both';

    % Locate the MNIST subfolder relative to THIS script's own location so
    % the dispatch is independent of the current working directory.
    candidates = { fullfile(this_dir, 'Examples', 'MNIST'), ...
                   fullfile(this_dir, 'MNIST') };
    mnist_dir  = '';
    for c = 1:numel(candidates)
        if exist(candidates{c}, 'dir')
            mnist_dir = candidates{c};
            break;
        end
    end
    if isempty(mnist_dir)
        error('MNIST folder not found. Looked in:\n  %s\n  %s', ...
              candidates{1}, candidates{2});
    end
    addpath(mnist_dir);

    % The runners read the data files by bare filename (e.g.
    % 't10k-images.idx3-ubyte'), which resolves against the current folder, so
    % we cd into the MNIST folder while they run and restore the original
    % directory afterwards (including on error).
    original_dir = pwd;
    cd(mnist_dir);
    try
        switch mnist_network
            case 'tiny'
                fprintf('Network: CNODE_S (tiny)\n\n');
                run_mnist_smoketest_expm_tiny();
            case 'medium'
                fprintf('Network: CNODE_M (medium)\n\n');
                run_mnist_smoketest_expm_medium();
            case 'both'
                fprintf('Network: CNODE_S (tiny), then CNODE_M (medium)\n\n');
                run_mnist_smoketest_expm_tiny();
                run_mnist_smoketest_expm_medium();
            otherwise
                error('Invalid mnist_network: %s', mnist_network);
        end
    catch ME
        cd(original_dir);
        rethrow(ME);
    end
    cd(original_dir);

    return;
end


%% =========================================================================
%   PURE NEURAL ODE SETUP  (Spital / FPA / Cartpole)
%  =========================================================================

node_params.input_split_heuristic = input_split_heuristic;


% --- 1. INITIAL SET ------------------------------------------------------

% Spiral_non (2D)
x0_spiral       = [2.0; 0.0];
radius_spiral   = 0.2;
timeStep_spiral = 0.01;        
tFinal_spiral   = 1.0;        

% Spiral_linear (2D) 
x0_spiral_lin     = [2.0; 0.0];
radius_spiral_lin = 0.2;
timeStep_spiral_lin = 0.01;
tFinal_spiral_lin   = 1.0;

% FPA (5D)
x0_fpa       = [0; -0.58587; 0.8; 0.52323; 0.7];
radius_fpa   = 0.01;
timeStep_fpa = 0.05;
tFinal_fpa   = 2.0;

% Cartpole (12D)
x0_cartpole     = [0; 0; 0.001; 0; 0; 0; 0; 0; 0; 0; 0; 0];   
radius_cartpole = 1e-4;                                      
timeStep_cartpole = 0.02;
tFinal_cartpole   = 0.2;                                    

% Auto-select based on system_choice
if system_choice == 15
    x0             = x0_spiral;
    Initial_radius = radius_spiral * ones(length(x0), 1);
    system_name    = 'spiral_non';
    cora_params.timeStep = timeStep_spiral;
    cora_params.tFinal   = tFinal_spiral;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_spiral;
    timeStep_TIRA        = timeStep_spiral;
elseif system_choice == 14
    x0             = x0_fpa;
    Initial_radius = radius_fpa * ones(length(x0), 1);
    system_name    = 'fpa';
    cora_params.timeStep = timeStep_fpa;
    cora_params.tFinal   = tFinal_fpa;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_fpa;
    timeStep_TIRA        = timeStep_fpa;
elseif system_choice == 21
    x0             = x0_spiral_lin;
    Initial_radius = radius_spiral_lin * ones(length(x0), 1);
    system_name    = 'spiral_linear';
    cora_params.timeStep = timeStep_spiral_lin;
    cora_params.tFinal   = tFinal_spiral_lin;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_spiral_lin;
    timeStep_TIRA        = timeStep_spiral_lin;
elseif system_choice == 22
    x0             = x0_cartpole;
    Initial_radius = radius_cartpole * ones(length(x0), 1);
    system_name    = 'cartpole';
    cora_params.timeStep = timeStep_cartpole;
    cora_params.tFinal   = tFinal_cartpole;
    t_init_TIRA          = 0;
    t_final_TIRA         = tFinal_cartpole;
    timeStep_TIRA        = timeStep_cartpole;

else
    error('Invalid system_choice: %d. Use 14 (FPA), 15 (Spiral_non), 21 (Spiral_lin), 22 (Cartpole).', ...
          system_choice);
end


% --- 2. CORA PARAMETERS (reachable-tube bounds) --------------------------
cora_params.taylorTerms   = 2;       
cora_params.zonotopeOrder = 20;       
cora_params.alg           = 'lin';    
cora_params.tensorOrder   = 2;       


% --- 3. TIRA PARAMETERS (NODE_RA_New) ------------------------------------
node_params.system_choice      = system_choice;
node_params.x0                 = x0;
node_params.Initial_radius     = Initial_radius;
node_params.t_final            = t_final_TIRA;
node_params.timeStep           = timeStep_TIRA;
node_params.plot_last_only     = true;
node_params.sample_succ_number = 1000;   


% --- 4. SAFE SET (USER-ADJUSTABLE) ---------------------------------------

% Make sure the safe-set dimension matches the system dimension
n_sys = length(x0);

if system_choice == 15            % Spiral_non (2D)
    node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
    node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]

elseif system_choice == 21        % Spiral_linear (2D)
    node_params.safe_inf = [-2; 1.2];   % [x1_low, x2_low]
    node_params.safe_sup = [0.1; 5];    % [x1_up, x2_up]

elseif system_choice == 14        % FPA (5D)
    node_params.safe_inf = [-1.29; -0.7;-0.75;0.35;2.38]; % [x1_low, x2_low, ...]
    node_params.safe_sup = [-1.23;-0.59;-0.72;0.38;2.46];  % [x1_up,  x2_up, ...]

elseif system_choice == 22        % Cartpole (12D)
    node_params.safe_inf = [
        -0.015;   
        -0.110;  
        -0.015;   
        -0.050;    
        -0.2 * ones(8,1) 
        ];

    node_params.safe_sup = [
        0.015;   
        0.060;   
        0.020;  
        0.160;   
        0.2 * ones(8,1) 
        ];
end

% Validate dimensions
if length(node_params.safe_inf) ~= n_sys
    error('Dimension mismatch: Safe set has %d dims, but system has %d.', ...
          length(node_params.safe_inf), n_sys);
end


% --- 5. VERIFICATION-LOOP PARAMETERS -------------------------------------
MAX_ITER = 5000;    
TIMEOUT_SECONDS = 120 * 60; % Wall-clock timeout T_max for the verification & refinement loop ONLY (does not include the CORA / Jacobian / falsification preamble). Set to `Inf` to disable the timeout and recover budget-only behavior.


% --- 6. PLOTTING CONTROL -------------------------------------------------
plot_control.show_figures = false;    % Show figures during execution
plot_control.save_final_plot = true; % Simplified: save if verified or falsified
% New Selector:
plot_control.verified_set_style = 'true_union'; 
% Options: 
%   'all'   -> Plot individual reachable sets for every verified subset (Detailed)
%   'union --> Interval Hull' -> Plot the union of all verified reachable subsets (Clean)
%   'true_union' -> Plot filled polygon of the exact geometric union (Cleanest)

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

tic;     % global timer (verdict time only)

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
verified_initial_sets   = {};  % stores {center, radius, intersect_low, intersect_up} per verified subset


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

% Simulate trajectories for these samples
final_samples = zeros(check_dims, num_falsification_samples);
global g_sensitivity_bounds_method;
parameters  = get_solver_parameters(length(node_params.x0), 0, g_sensitivity_bounds_method);
ode_solver  = parameters.ode_solver;
ode_options = parameters.ode_options;

% System_description reads system_choice from the global scope, so we
% redeclare it here before constructing the closure below.
global system_choice;
% Creates the function handle for the ODE solver
ode_func = @(t,x) System_description(t,x,[]);


% Simulate every initial sample to t_final
for i = 1:num_falsification_samples
    x0_rand = initial_samples(:, i);
    [~, x_traj] = ode_solver(@(t,x) System_description(t, x, []), ...
                             [t_init_TIRA, t_final_TIRA], x0_rand, ode_options);
    final_samples(:, i) = x_traj(end, 1:check_dims)';
end

% Counter-example check to see if any sample violates the safe set
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


% --- PART 5: Verification and refinement loop ------------------------------
fprintf('PART 5: Verification and Refinement based Heuristics Loop...\n');

host_system_choice = system_choice;
host_Jx_low        = Jacobian_x_low;
host_Jx_up         = Jacobian_x_up;

% Initialize / reuse parallel pool
fprintf('  Initializing Parallel Pool...\n');
p = gcp('nocreate');
if isempty(p)
    p = parpool;
end
worker_load_factor = 5;
batch_size         = p.NumWorkers * worker_load_factor;
fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
        p.NumWorkers, batch_size, worker_load_factor);

batch_iter              = 0;
total_subsets_processed = 0;
total_explosions        = 0;

% '' (empty) means the loop ended because the queue is empty (i.e., SAFE).
loop_stop_reason = ''

% Loop-only wall clock. Separated from the run-wide `tic` above so that
% TIMEOUT_SECONDS bounds the verification & refinement loop only, matching
% the paper's T_max (the CORA/Jacobian/falsification preamble is excluded).
loop_timer = tic;

if overall_safe   % only enter the loop if we have not already falsified
    while ~isempty(verification_queue) ...
            && total_subsets_processed < MAX_ITER ...
            && toc(loop_timer) < TIMEOUT_SECONDS

        batch_iter = batch_iter + 1;

        % Pop up to batch_size subsets
        remaining_budget = MAX_ITER - total_subsets_processed;
        num_tasks        = min([length(verification_queue), batch_size, remaining_budget]);

        current_batch         = verification_queue(1:num_tasks);
        verification_queue(1:num_tasks) = [];

        fprintf('--- BATCH %d: Processing %d parallel sub-regions (Queue remaining: %d, Subsets processed so far: %d/%d, Loop elapsed: %.1f/%.0f min) ---\n', ...
                batch_iter, num_tasks, length(verification_queue), total_subsets_processed, MAX_ITER, ...
                toc(loop_timer)/60, TIMEOUT_SECONDS/60);

        % Broadcast variables required by every worker
        node_params_batch = node_params;
        safe_inf_local    = node_params.safe_inf;
        safe_sup_local    = node_params.safe_sup;
        heuristic_local   = node_params.input_split_heuristic;
        sc_local          = host_system_choice;
        Jx_low_local      = host_Jx_low;
        Jx_up_local       = host_Jx_up;

        results = cell(1, num_tasks);

        % =========================================================
        %                PARALLEL COMPUTATION BLOCK
        % =========================================================
        parfor idx = 1:num_tasks

            % Re-seed globals on this worker (parfor workers do NOT inherit
            % the host's globals; NODE_RA_New + the heuristics rely on
            % system_choice and Jacobian_x_low/up).
            seed_globals_for_worker(sc_local, Jx_low_local, Jx_up_local); 

            % --- Unpack the queued subset ---
            task = current_batch{idx};
            x_i  = task{1};
            r_i  = task{2};
            if isscalar(r_i)
                r_i = r_i * ones(length(x_i), 1);
            else
                r_i = r_i(:);
            end

            % --- Build the per-subset NODE_RA_New input ---
            sub_params                = node_params_batch;
            sub_params.x0             = x_i;
            sub_params.Initial_radius = r_i;

            % --- Defaults for the result struct ---
            is_safe        = false;
            had_explosion  = false;
            split_dim      = 1;
            int_low_out    = NaN(length(x_i), 1);
            int_up_out     = NaN(length(x_i), 1);

            % --- Outer try/catch: CORA/TIRA explosion handling ----------
            try
                % 1. Reachability for this subset
                rr          = NODE_RA_New(sub_params);
                int_low_out = rr.intersect_low;
                int_up_out  = rr.intersect_up;

                % 2. Safety check
                low_ok = all(int_low_out >= safe_inf_local);
                up_ok  = all(int_up_out  <= safe_sup_local);

                if low_ok && up_ok
                    is_safe = true;
                else
                    % 3. Refinement heuristic (only when not provably safe)
                    switch heuristic_local
                        case 'naive'
                            [~, split_dim] = max(r_i);

                        case 'most-sensitive-input-radius'
                            try
                                [J_lb_loc, J_ub_loc] = Jacob_bounds_element_wise(x_i - r_i, x_i + r_i, sc_local);
                                absJ   = max(abs(J_lb_loc), abs(J_ub_loc));
                                sens   = max(absJ, [], 1);
                                sens   = max(sens, 1e-10);
                                scores = r_i .* sens(:);
                                [~, split_dim] = max(scores);
                            catch
                                [~, split_dim] = max(r_i);
                            end

                        case 'interval-norm-gradient'
                            try
                                base_norm = sum(int_up_out - int_low_out);
                                if base_norm <= 0
                                    error('NODE_RA_New_pf:invalid_base', ...
                                          'Base reachable set has zero or negative width');
                                end
                                grad  = zeros(size(x_i));
                                delta = 1e-6;
                                for j = 1:length(x_i)
                                    x_pert    = x_i;
                                    x_pert(j) = x_i(j) + delta;
                                    pp        = sub_params;
                                    pp.x0     = x_pert;
                                    pr        = NODE_RA_New(pp);
                                    pn        = sum(pr.intersect_up - pr.intersect_low);
                                    grad(j)   = (pn - base_norm) / delta;
                                end
                                hv = abs(grad);
                                if any(isnan(hv) | isinf(hv) | hv <= 0)
                                    error('NODE_RA_New_pf:invalid_gradient', ...
                                          'Invalid gradient');
                                end
                                [~, split_dim] = max(hv);
                            catch
                                [~, split_dim] = max(r_i);
                            end

                        otherwise
                            error('Invalid input_split_heuristic: %s', heuristic_local);
                    end
                end

            catch
                % CORA reachable-set explosion or TIRA failure: mark this
                % subset as unsafe so the reducer splits it on the next batch.
                is_safe        = false;
                had_explosion  = true;
                [~, split_dim] = max(r_i);   % naive fallback split dimension
            end

            % --- Pack the worker result ---
            results{idx} = struct('is_safe',       is_safe, ...
                                  'had_explosion', had_explosion, ...
                                  'split_dim',     split_dim, ...
                                  'x_i',           x_i, ...
                                  'r_i',           r_i, ...
                                  'int_low',       int_low_out, ...
                                  'int_up',        int_up_out);
        end
        % =========================================================

        % --- Sequential reducer ----------------------------------------
        batch_safe       = 0;
        batch_unsafe     = 0;
        batch_explosions = 0;
        for idx = 1:num_tasks
            res = results{idx};
            if res.had_explosion
                batch_explosions = batch_explosions + 1;
            end
            if res.is_safe
                batch_safe = batch_safe + 1;
                verified_initial_sets{end+1} = {res.x_i, res.r_i, res.int_low, res.int_up}; 
                num_verified = num_verified + 1;
                num_unknown  = num_unknown - 1;
            else
                batch_unsafe = batch_unsafe + 1;
                sd          = res.split_dim;
                r_new       = res.r_i;
                r_new(sd)   = res.r_i(sd) / 2;
                x_c1        = res.x_i; x_c1(sd) = res.x_i(sd) - res.r_i(sd) / 2;
                x_c2        = res.x_i; x_c2(sd) = res.x_i(sd) + res.r_i(sd) / 2;
                verification_queue{end+1} = {x_c1, r_new}; 
                verification_queue{end+1} = {x_c2, r_new}; 
                num_unknown = num_unknown - 1 + 2;
            end
        end
        total_explosions        = total_explosions        + batch_explosions;
        total_subsets_processed = total_subsets_processed + num_tasks;

        fprintf('  -> Batch %d complete: %d safe, %d unsafe (%d CORA explosions, spawned %d new sets).\n', ...
                batch_iter, batch_safe, batch_unsafe, batch_explosions, batch_unsafe * 2);
    end

     % Record which limit ended the loop (checked in the same priority order
     % as the verdict logic below).
    if ~isempty(verification_queue)
        if toc(loop_timer) >= TIMEOUT_SECONDS
            loop_stop_reason = 'timeout';
        elseif total_subsets_processed >= MAX_ITER
            loop_stop_reason = 'budget';
        end
    end
end

% Capture the loop-only elapsed time for reporting.
loop_elapsed_time = toc(loop_timer);

% Keep `iter` for final summary reporting.
iter = total_subsets_processed;

if total_explosions > 0
    fprintf('[diagnostic] Total CORA/TIRA explosions across run: %d\n', total_explosions);
end
fprintf('PART 5 completed.\n\n');

% Stop VERDICT timer and get elapsed time 
elapsed_time = toc;


%% =========================================================================
%   PART 6 : FINAL PLOTTING AND SAVING
%  =========================================================================

if plot_control.save_final_plot
    fprintf('PART 6: Generating Final_Entire_input_set_Verification_Plot and Saving Data...\n');

    % % Create a single results folder
    %=====================================================================
    % CREATE THE RESULTS FOLDER ONCE
    % =====================================================================
    folder_name = sprintf('FINAL_RESULTS_%s_%s', system_name, datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
    save_dir = fullfile(pwd, folder_name);
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    fprintf('  Created results folder:\n    %s\n', save_dir);
    
    % Calculate the R_s OA for the initial input set one last time 
    node_params_full = node_params;
    node_params_full.x0 = x0;
    node_params_full.Initial_radius = Initial_radius;
    res_full = NODE_RA_New(node_params_full);

    % --- Determine Projections based on System ---
    if system_choice == 14 % FPA (5D)
        % Plot requested projections: (x1,x2), (x3,x4), (x4,x5)
        plot_pairs = {[1 2], [3 4], [4 5]};

    elseif system_choice == 22 % Cartpole (12D)
        % Physical states only: cart (pos,vel) and pole (angle,rate)
        plot_pairs = {[1 2], [3 4]};

    else
        % Default 2D projection (x1,x2)
        plot_pairs = {[1 2]};
    end

    % --- Loop over requested projections ---
    for p_idx = 1:length(plot_pairs)
        dims = plot_pairs{p_idx};
        px = dims(1); py = dims(2);

        fprintf('  Generating Figure Set %d for x%d vs x%d...\n', p_idx, px, py);
    
        % Unrefined Rect Projection
        unrefined_low = res_full.intersect_low;
        unrefined_up  = res_full.intersect_up;
        
        rect_unrefined = [unrefined_low(px), unrefined_low(py), ...
                          unrefined_up(px)-unrefined_low(px), unrefined_up(py)-unrefined_low(py)];

        % Safe Set Projection (Clamp Inf for plotting) --> Main Usage is to ensure X_s is
        % plotted correctly by MATLAB
        sx_inf = max(node_params.safe_inf(px)); sx_sup = min(node_params.safe_sup(px));
        sy_inf = max(node_params.safe_inf(py)); sy_sup = min(node_params.safe_sup(py));
        safe_x = [sx_inf, sx_sup, sx_sup, sx_inf, sx_inf];
        safe_y = [sy_inf, sy_inf, sy_sup, sy_sup, sy_inf];

        % ===========================================================================================================
        % FIGURE 1: Verified Reachable Set (Union of Verified Subsets or all of them) vs. Entire Initial input set OA 
        % ===========================================================================================================
    
        fig = figure('Color', 'w', 'Visible', plot_control.show_figures, 'Position', [100 100 900 700]);
        hold on; grid on; box on;
        set(gca, 'FontSize', 18, 'TickLabelInterpreter', 'latex');
        k = 1;

        % 1. Black sampled points (from the full initial set)
        plot(final_samples(px,:), final_samples(py,:), 'k.', 'MarkerSize', 5, 'DisplayName', 'Sampled Trajectories');

        % 2. Plot the Unsafe/Un-VERIFIED Reachable Set (i.e. Entire Initial input set) in Red
              
        % Draw it in Red with a dashed line to indicate "Unsafe/Un-VERIFIED"
        rectangle('Position', rect_unrefined, 'EdgeColor', [0.85 0.33 0.1], 'LineWidth', 2, 'LineStyle', '--');
        h_unrefined = plot(NaN,NaN,'--','Color',[0.85 0.33 0.1],'LineWidth',2, 'DisplayName', 'Unrefined OA (Entire Set)');

        % 3. Plot the Reachable Sets of the VERIFIED subsets
    
        h_int = []; 

        union_low = [Inf; Inf];  % For 2D projections
        union_up = [-Inf; -Inf]; % For 2D projections
    
        % Prepare for True Union
        total_union_poly = polyshape();

        if ~isempty(verified_initial_sets)
            fprintf('  Computing reachable sets for %d verified subsets...\n', length(verified_initial_sets));
        
            for i = 1:length(verified_initial_sets)
                % Extract subset data
                set_info = verified_initial_sets{i};
                x_sub      = set_info{1}; 
                r_sub      = set_info{2};  
                int_low    = set_info{3};  
                int_up     = set_info{4};  
                               
                % Logic for different plotting styles
                if strcmp(plot_control.verified_set_style, 'all')
                    % MODE: ALL - Plot each subset individually
                    rect_dims = [int_low(px), int_low(py), int_up(px)-int_low(px), int_up(py)-int_low(py)];
                    rectangle('Position', rect_dims, 'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
                
                elseif strcmp(plot_control.verified_set_style, 'true_union')
                    % MODE: TRUE_UNION  
                    x_corners = [int_low(px), int_up(px), int_up(px), int_low(px)];
                    y_corners = [int_low(py), int_low(py), int_up(py), int_up(py)];
                    new_poly = polyshape(x_corners, y_corners);
                    % Merge into the total union
                    total_union_poly = union(total_union_poly, new_poly);    
                
                else
                    % MODE: UNION/Interval Hull 
                    union_low = min(union_low, [int_low(px); int_low(py)]);
                    union_up  = max(union_up,  [int_up(px);  int_up(py)]);
                end
            end
        
            % % Handle drawing for 'union' mode or just creating the legend handle
            if strcmp(plot_control.verified_set_style, 'union')
                 rect_dims = [union_low(1), union_low(2), union_up(1)-union_low(1), union_up(2)-union_low(2)];
                 rectangle('Position', rect_dims, 'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 2);

            elseif strcmp(plot_control.verified_set_style, 'true_union')
                 h_poly = plot(total_union_poly, 'FaceColor', [0.4940 0.1840 0.5560], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
            end
        
            % Create a dummy handle for the legend
            legend_name = 'Verified Reachable Sets';
            if strcmp(plot_control.verified_set_style, 'union')
                legend_name = 'Union (Interval Hull) of Verified Reachable Sets';
            end
    
            if strcmp(plot_control.verified_set_style, 'true_union')
                legend_name = 'True Union of Verified Reachable Sets'; 
            end
            
            h_int = plot(NaN,NaN,'-','Color',[0.4940 0.1840 0.5560],'LineWidth',2, 'DisplayName', legend_name);
        end
    
        % 4. Safe set (thick green)
        h_safe = plot(safe_x, safe_y, 'Color',[0 0.8 0], 'LineWidth',2, 'DisplayName','Safe Set');
    
        % 5. Falsified point (if any)
        h_fals = [];
        if ~overall_safe
            plot(falsified_example(px), falsified_example(py), 'rX', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Counter-Example');
        end
    
        axis equal;
        xlabel(['$x_' num2str(px) '$'], 'Interpreter','latex','FontSize',20); 
        ylabel(['$x_' num2str(py) '$'], 'Interpreter','latex','FontSize',20);
        title(['Reachable set OA vs Verified Safe Subsets ($x_' num2str(px) ', x_' num2str(py) '$)'], 'Interpreter','latex','FontSize',18);
    

        % 6. Legend Logic
        legend_entries = [h_unrefined];
        legend_labels = {'Entire Initial input set OA'};
        
        if ~isempty(h_int)
            legend_entries = [legend_entries, h_int];
            legend_labels = [legend_labels, {'Union of verified subsets OAs'}];
        end
    
        legend_entries = [legend_entries, h_safe];
        legend_labels = [legend_labels, {'Safe Set'}];
    
        if ~overall_safe && ~isempty(h_fals)
            legend_entries(end+1) = h_fals;
            legend_labels{end+1} = 'Counter-Example';
        end
    
        legend(legend_entries, legend_labels, 'Location', 'northeast', 'Interpreter', 'latex');
        set(legend, 'FontSize', 10, 'FontWeight', 'bold', 'Box', 'on')
    
        % plot_filename_entire = fullfile(save_dir, 'Final_Entire_input_set_Verification_Plot_x%d_x%d', px, py');
        % Dynamic plot
        % 1. Create the dynamic filename string first using sprintf
        filename_str = sprintf('Final_Entire_input_set_Verification_Plot_x%d_x%d', px, py);

        % 2. Combine with the directory path
        plot_filename_entire = fullfile(save_dir, filename_str);

        % Save in multiple formats
        saveas(gcf, [plot_filename_entire, '.png']);
        saveas(gcf, [plot_filename_entire, '.pdf']);
        saveas(gcf, [plot_filename_entire, '.fig']);
        print(gcf, [plot_filename_entire, '.eps'], '-depsc'); % Vector EPS for LaTeX
        print(gcf, [plot_filename_entire, '_highres'], '-dpng', '-r600'); % 600 DPI PNG
        fprintf('Final_Entire_input_set_Verification_Plots saved as(PNG, PDF, FIG, EPS, high-res PNG) in:\n %s\n', plot_filename_entire);
        % fprintf('Final_Entire_input_set_Verification_Plots saved for x%d vs x%d.\n', px, py);
    
        % ================================================================================
        % FIGURE 2: Final Refined Plotting: Initial input set vs. Output reachable set OAs
        % ================================================================================
    
        % Create a new figure with a larger size
        fig = figure('Color', 'w', 'Visible', plot_control.show_figures, 'Position', [100 100 900 700]);
        hold on; grid on; box on;
        set(gca, 'FontSize', 18, 'TickLabelInterpreter', 'latex');
    
        % 1. [INPUT] Plot the initial sets that were proven safe (in blue)
    
        if ~isempty(verified_initial_sets)
            for i = 1:length(verified_initial_sets)
                set_info = verified_initial_sets{i};
                center = set_info{1}; radius = set_info{2};
                rect_pos = [center(px)-radius(px), center(py)-radius(py), 2*radius(px), 2*radius(py)];
                rectangle('Position', rect_pos, 'EdgeColor', 'b', 'FaceColor', [0.8 0.8 1]);
            end
        end
        h_ver_init = patch(NaN, NaN, 'b', 'FaceAlpha', 0.3, 'DisplayName', 'Verified Initial Subsets');

    
        % 2. [INPUT] Plot the original full initial input set boundary (dashed black)
        
        initial_inf = x0 - Initial_radius;
        initial_dims = (x0 + Initial_radius) - initial_inf;
        rectangle('Position', [initial_inf(px), initial_inf(py), initial_dims(px), initial_dims(py)], 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
        h_init = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Original Initial Set');


        % 3. [OUTPUT] Unrefined Reachable Set (Red Dashed)
        
        rectangle('Position', rect_unrefined, 'EdgeColor', [0.85 0.33 0.1], 'LineWidth', 2, 'LineStyle', '--');
        h_unrefined_2 = plot(NaN,NaN,'--','Color',[0.85 0.33 0.1],'LineWidth',2, 'DisplayName', 'Entire Initial input set OA');


        % 4. [OUTPUT] Verified Reachable Sets (Purple)
        % We directly re-use the calculation from Fig 1 logic (computed above)
        
        if ~isempty(verified_initial_sets)
            if strcmp(plot_control.verified_set_style, 'true_union')
                % Use the poly we calculated in Fig 1
                h_poly_2 = plot(total_union_poly, 'FaceColor', [0.4940 0.1840 0.5560], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
                
            elseif strcmp(plot_control.verified_set_style, 'all')
                % Re-loop to plot individual RS rectangles using stored bounds
                for i = 1:length(verified_initial_sets)
                    set_info = verified_initial_sets{i};
                    int_low  = set_info{3};
                    int_up   = set_info{4};
                    rect_dims = [int_low(px), int_low(py), ...
                                 int_up(px)-int_low(px), int_up(py)-int_low(py)];
                    rectangle('Position', rect_dims, 'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
                end 
            else
                 % Union mode (using bounds calculated in Fig 1 block)
                 rect_dims = [union_low(1), union_low(2), union_up(1)-union_low(1), union_up(2)-union_low(2)];
                 rectangle('Position', rect_dims, 'EdgeColor', [0.4940 0.1840 0.5560], 'LineWidth', 2);
            end
        end
        h_ver_rs_2 = plot(NaN,NaN,'-','Color',[0.4940 0.1840 0.5560],'LineWidth',2, 'DisplayName', 'Verified Reachable Sets'); % Union of Verified Subsets OAs
        
        % Black sampled points (from the full initial set)
        plot(final_samples(px,:), final_samples(py,:), 'k.', 'MarkerSize', 5, 'DisplayName', 'Sampled Trajectories');

        % 5. [OUTPUT] Safe Set (Green)
        h_safe_2 = plot(safe_x, safe_y, 'Color',[0 0.8 0], 'LineWidth',2, 'DisplayName','Safe Set');

    
        % 6. Falsified Points (Input and Output if applicable)
        h_fals_in = []; h_fals_out = [];
        if ~overall_safe
            % Input Point
            h_fals_in = plot(falsified_initial_point(1), falsified_initial_point(2), 'bx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Falsifying Input');
            % Output Point
            h_fals_out = plot(falsified_example(1), falsified_example(2), 'rX', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Falsifying Output');
        end
    

        axis equal;
        xlabel(['$x_' num2str(px) '$'], 'Interpreter','latex','FontSize',20); 
        ylabel(['$x_' num2str(py) '$'], 'Interpreter','latex','FontSize',20);
        title(['Global View: Refined Initial input set \& Resulting Refined Safe subsets OAs ($x_' num2str(px) ', x_' num2str(py) '$)'],'Interpreter','latex','FontSize',18);
        

        % Legend 2
        legend_entries_2 = [h_init, h_ver_init, h_unrefined_2, h_ver_rs_2, h_safe_2];
        legend_labels_2 = {'Original Initial input Set', 'Verified Initial Subsets', 'Entire Initial input set OA', 'Union of verified subsets OAs', 'Safe Set'};
    
        if ~overall_safe
            if ~isempty(h_fals_in), legend_entries_2(end+1) = h_fals_in; legend_labels_2{end+1} = 'Falsifying Input'; end
            if ~isempty(h_fals_out), legend_entries_2(end+1) = h_fals_out; legend_labels_2{end+1} = 'Falsifying Output'; end
        end
    
        legend(legend_entries_2, legend_labels_2, 'Location', 'northeast', 'Interpreter', 'latex');
        set(legend, 'FontSize', 10, 'FontWeight', 'bold', 'Box', 'on')
        
        % plot_filename_refined = fullfile(save_dir, 'Refined_Safe_Subsets_Plot_x%d_x%d', px, py');
        % Dynamic plot
        % 1. Create the dynamic filename string first using sprintf
        filename_str = sprintf('Refined_Safe_Subsets_Plot_x%d_x%d', px, py);
        
        % 2. Combine with the directory path
        plot_filename_refined = fullfile(save_dir, filename_str);

        % Save in multiple formats
        saveas(gcf, [plot_filename_refined, '.png']);
        saveas(gcf, [plot_filename_refined, '.pdf']);
        saveas(gcf, [plot_filename_refined, '.fig']);
        print(gcf, [plot_filename_refined, '.eps'], '-depsc'); % Vector EPS for LaTeX
        print(gcf, [plot_filename_refined, '_highres'], '-dpng', '-r600'); % 600 DPI PNG
        fprintf('Refined_Safe_Subsets_Plots saved as(PNG, PDF, FIG, EPS, high-res PNG) in:\n %s\n', plot_filename_refined);
    end

        % Save the detailed .mat file
        filename_base = sprintf('reachability_data_sys%d_%s', system_choice, datestr(now, 'yyyy-mm-dd'));
       
     
        save(fullfile(save_dir, [filename_base '.mat']), ...
            'verified_initial_sets', ...
            'falsified_example', 'falsified_initial_point', ...
            'final_samples', ...
            'res_full', ... 
            'overall_safe', '-v7.3');
    
        fprintf('Final .mat data saved to: %s.mat\n', filename_base);
    
        fprintf('PART 6 completed.\n\n');
end

%% =========================================================================
%   FINAL VERDICT & SUMMARY
%  =========================================================================

clear global Jacobian_x_low Jacobian_x_up system_choice_global

% --- Final Verdict ---
is_fully_verified = false;
verdict_string = '';

if ~overall_safe
    verdict_string = 'FALSIFIED! Counter-example found.';
else
    % Simplified clear Logic:
    % If queue is empty
    if isempty(verification_queue)
        is_fully_verified = true;
        verdict_string = 'FULLY VERIFIED (SAFE) for the initial input set.';
    else
        % If queue is not empty
        verdict_string = sprintf('UNKNOWN (Timeout: %d subsets remain unverified).', length(verification_queue));
    end
end


% --- Final Summary ---
fprintf('\n\n=================================================================\n');
fprintf('               Verification Analysis Complete\n');
fprintf('=================================================================\n\n');
fprintf('--- Overall Summary ---\n');
fprintf('System: %s\n', upper(system_name));
fprintf('Total Iterations: %d\n', iter);
fprintf('Verified Subsets: %d\n', num_verified);
fprintf('Falsified Subsets: %d\n', num_falsified);
fprintf('Remaining Unknown Subsets: %d\n', length(verification_queue)); 

% Logic to print the elapsed time in seconds (if less than 60 seconds) or minutes 
% (if 60 seconds or more) for easier readability
if elapsed_time < 60
    fprintf('Total Time Taken: %.2f seconds\n', elapsed_time);
else
    fprintf('Total Time Taken: %.2f minutes\n', elapsed_time / 60);
end

fprintf('\nVERDICT: %s\n', verdict_string);
if ~overall_safe, fprintf('  Initial point: [%s]\n  Final point:   [%s]\n', strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_initial_point, 'UniformOutput', false), ', '), strjoin(arrayfun(@(x) sprintf('%.4f', x), falsified_example, 'UniformOutput', false), ', ')); end

fprintf('\n=================================================================\n');


%% =========================================================================
%   LOCAL HELPER FUNCTION (used by the parfor body in PART 5)
%  =========================================================================
function seed_globals_for_worker(sc, Jx_low, Jx_up)
    global system_choice Jacobian_x_low Jacobian_x_up
    system_choice  = sc;
    Jacobian_x_low = Jx_low;
    Jacobian_x_up  = Jx_up;
end

%------------- END OF CODE --------------
