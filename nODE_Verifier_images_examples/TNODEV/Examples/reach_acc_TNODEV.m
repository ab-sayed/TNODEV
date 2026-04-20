% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 21 February 2026
% Last update: 20 April 2026
% Last revision: October 2025

%% Working code for a single call without refinements
% function reach_acc_TNODEV()
% 
%     % Reachability analysis of Adaptive Cruise Control using TNODEV
%     % for the continuous plant and NNV for the discrete controller.
% 
%     %% Load objects
% 
%     % Load controller
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     % --- MODIFICATION 1: Setup TNODEV instead of NonLinearODE ---
%     ode_params = struct();
%     ode_params.system_choice = 17; % ACC tanh plant in System_description
%     ode_params.t_final = 0.1;      % Control Period is 0.1s
%     ode_params.timeStep = 10;      % Discretization steps for TNODEV
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
%     % ------------------------------------------------------------
% 
%     %% Reachability analysis
% 
%     % Set reachability options
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11]; 
%     v_ego = [30 30.2];
% 
%     % initial state set
%     lb = [xlead(1);v_lead(1);0;x_ego(1);v_ego(1);0;0;-2]; %lower bound
%     ub = [xlead(2);v_lead(2);0;x_ego(2);v_ego(2);0;0;-2]; % upper bound
%     X0 = Star(lb,ub); 
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                 1 0 0 -1 0 0 0 0;
%                 0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); % vset and tgap
% 
%     % Perform reachability analysis
%     trajR = X0;
%     trajU = [];
%     N = 50;
% 
%     % --- Start Full Verification Timer ---
%     t_full = tic;
% 
%     % --- Start Computation Timer (Reachability Only) ---
%     t_reach = tic;
%     for k=1:N
%         fprintf('Computing step %d / %d...\n', k, N);
% 
%         % --- MODIFICATION 2: TNODEV Continuous Reachability ---
% 
%         % 1. Extract the Interval Hull from X0 (X0 might be an array of Stars)
%         lb_all = zeros(states, length(X0));
%         ub_all = zeros(states, length(X0));
%         for s_idx = 1:length(X0)
%             [lb_all(:, s_idx), ub_all(:, s_idx)] = X0(s_idx).getRanges();
%         end
%         % Get the absolute min and max across all possible split sets
%         lb_current = min(lb_all, [], 2);
%         ub_current = max(ub_all, [], 2);
% 
%         % 2. Setup TNODEV parameters for this step
%         ode_params.x0 = (lb_current + ub_current) / 2;
%         ode_params.Initial_radius = (ub_current - lb_current) / 2;
% 
%         % 3. Run Interval Reachability based on on `NODE_RA_new.m`
%         tnode_res = NODE_RA_New(ode_params);
% 
%         % 4. Convert TNODEV output back into a Star set
%         X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%         % ------------------------------------------------------
% 
%         trajR = [trajR X0]; % Keep track of trajectory of NNCS
% 
%         % Controller Step (NNV)
%         ppp = X0.affineMap(map_mat,[]);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
%         trajU = [trajU Rc];
% 
%         % State Update
%         x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%         X0 = X0.affineMap(C(1:6,:),[]); % Get set for variables 1 to 6
%         X0 = X0.concatenate(Rc); % Add state/input 7 (a_ego)
%         X0 = X0.concatenate(x08);
%     end
%     rT = toc(t_reach); % End Reachability Timer
% 
%     %% Extract Sets and Safety Specification Check
%     t_gap = 1.4;
%     D_default = 10;
%     alp = 1;
%     outAll = [];
%     safe_dis = [];
% 
%     is_safe = true; % Assume safe until proven otherwise
% 
%     for i=1:length(trajR)
%         % Map to relative distance and safe distance Stars
%         % trajR(i) is guaranteed to be a single Star here due to the TNODEV Interval Hull
%         rel_dist_star = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
%         safe_dist_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%         outAll = [outAll rel_dist_star]; 
%         safe_dis = [safe_dis safe_dist_star]; 
% 
%         % Mathematical Safety Check (rel_dist > safe_dist)
%         [min_rel_dist, ~] = rel_dist_star.getRanges();
%         [~, max_safe_dist] = safe_dist_star.getBox();
% 
%         if min_rel_dist <= max_safe_dist
%             is_safe = false;
%         end
%     end
% 
%     % --- End Full Verification Timer ---
%     total_time = toc(t_full);
% 
%     % Save results
%     save('results_tanhplant_TNODEV.mat','rT',"trajR","trajU");
% 
%     %% Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (TNODEV)\n');
%     fprintf('=================================================\n');
%     if is_safe
%         fprintf('Result: SAFE (Relative distance is always > Safe distance)\n');
%     else
%         fprintf('Result: UNSAFE (Safety violation detected)\n');
%     end
%     fprintf('Reachability Loop Time: %.4f seconds\n', rT);
%     fprintf('Full Verification Time: %.4f seconds\n', total_time);
%     fprintf('=================================================\n\n');
% 
%     %% Visualize results
%     times = 0:0.1:0.1*N; % to plot in x-axis (time)
% 
%     % Create figure
%     f = figure;
%     hold on;
%     pb = plot(0,85,'m');
%     pr = plot(0,85,'k');
%     Star.plotRanges_2D(outAll,1,times,'k');
%     hold on;
%     Star.plotRanges_2D(safe_dis,1,times,'m');
% 
%     % Enhance figure for paper
%     ax = gca; % Get current axis
%     ax.XAxis.FontSize = 15; % Set font size of axis
%     ax.YAxis.FontSize = 15;
%     xlabel('Time (s)');
%     ylabel('Distance (m)')
%     ylim([40 110])
%     legend([pr,pb],{'rel dist','safe dist'},"Location","best",'FontSize',14);
% 
%     % Save figure locally as both PNG and PDF
%     fprintf('Saving figures to your current directory...\n');
%     exportgraphics(f, 'acc_tnodev.pdf', 'ContentType', 'vector');
%     saveas(f, 'acc_tnodev.png');
%     fprintf('Done!\n');
% 
% end

%% Added interval hull prints after the nODE RA

% function reach_acc_TNODEV()
% 
%     % Reachability analysis of Adaptive Cruise Control using TNODEV
%     % for the continuous plant and NNV for the discrete controller.
% 
%     %% Load objects
% 
%     % Load controller
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     % Setup TNODEV instead of NonLinearODE
%     ode_params = struct();
%     ode_params.system_choice = 17; % ACC tanh plant in System_description
%     ode_params.t_final = 0.1;      % Control Period is 0.1s
%     ode_params.timeStep = 10;      % Discretization steps for TNODEV
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     %% Reachability analysis
% 
%     % Set reachability options
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11]; 
%     v_ego = [30 30.2];
% 
%     % initial state set
%     lb = [xlead(1);v_lead(1);0;x_ego(1);v_ego(1);0;0;-2]; %lower bound
%     ub = [xlead(2);v_lead(2);0;x_ego(2);v_ego(2);0;0;-2]; % upper bound
%     X0 = Star(lb,ub); 
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                 1 0 0 -1 0 0 0 0;
%                 0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); % vset and tgap
% 
%     % Perform reachability analysis
%     trajR = X0;
%     trajU = [];
%     N = 50; % 50 steps total (i.e., 5 seconds: N*ode_params.t_final)
% 
%     % Arrays to track Interval Hull Measurements (for saving purposes)
%     hull_max_widths = zeros(1, N);
%     hull_volumes = zeros(1, N);
% 
%     % --- NEW: Setup CORA Parameters ---
%     global Jacobian_x_low Jacobian_x_up system_choice
%     system_choice = 17; % Ensure global system_choice is set
% 
%     cora_params.timeStep      = 0.05; 
%     cora_params.tFinal        = ode_params.t_final; % 0.1s
%     cora_params.taylorTerms   = 3;
%     cora_params.zonotopeOrder = 10;
%     cora_params.alg           = 'lin';
%     cora_params.tensorOrder   = 2;
%     % ----------------------------------
% 
%     % --- Start Full Verification Timer ---
%     t_full = tic;
% 
%     % --- Start Computation Timer (Reachability Only) ---
%     t_reach = tic;
%     for k=1:N
%         fprintf('Computing step %d / %d...\n', k, N);
% 
%         % 1. Extract the Interval Hull from X0 (X0 might be an array of Stars)
%         lb_all = zeros(states, length(X0));
%         ub_all = zeros(states, length(X0));
%         for s_idx = 1:length(X0)
%             % [lb_all(:, s_idx), ub_all(:, s_idx)] = X0(s_idx).getBox();
%             B = X0(s_idx).getBox();
%             lb_all(:, s_idx) = B.lb;
%             ub_all(:, s_idx) = B.ub;
%         end
%         % Get the absolute min and max across all possible split sets
%         lb_current = min(lb_all, [], 2);
%         ub_current = max(ub_all, [], 2);
% 
%         % 2. Setup TNODEV parameters for this step
%         ode_params.x0 = (lb_current + ub_current) / 2;
%         ode_params.Initial_radius = (ub_current - lb_current) / 2;
% 
%         % ---> NEW: Compute CORA RT and Push Jacobians <---
%         cora_params.x0 = ode_params.x0;
%         cora_params.Initial_radius = ode_params.Initial_radius;
% 
%         % Step A: Get Reachable Tube from CORA
%         rt_bounds = acc_CORA_RT(cora_params);
% 
%         % Step B: Compute Jacobians over that Tube
%         [J_lb, J_ub] = Jacob_bounds_element_wise(rt_bounds.lb, rt_bounds.ub, system_choice);
% 
%         % Step C: Push to Global Workspace for TIRA
%         Jacobian_x_low = J_lb;
%         Jacobian_x_up  = J_ub;
%         % -------------------------------------------------
% 
%         % 3. Run Interval Reachability based on NODE_RA_new.m
%         tnode_res = NODE_RA_New(ode_params);
% 
%         % --- NEW: Print the Interval Hull metrics dynamically ---
%         % Calculate width (UB - LB) for all 8 dimensions
%         current_widths = tnode_res.intersect_up - tnode_res.intersect_low;
% 
%         % Store and print max uncertainty width and total volume
%         hull_max_widths(k) = max(current_widths);
%         hull_volumes(k) = prod(current_widths); 
% 
%         % fprintf('  -> Hull Max Width: %.4f | Hull Volume: %e\n', hull_max_widths(k), hull_volumes(k));
%         % --------------------------------------------------------
% 
%         % 4. Convert TNODEV output back into a Star set
%         X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%         trajR = [trajR X0]; % Keep track of trajectory of NNCS
% 
%         % Controller Step (NNV)
%         ppp = X0.affineMap(map_mat,[]);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
%         trajU = [trajU Rc];
% 
%         % State Update
%         x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%         X0 = X0.affineMap(C(1:6,:),[]); % Get set for variables 1 to 6
%         X0 = X0.concatenate(Rc); % Add state/input 7 (a_ego)
%         X0 = X0.concatenate(x08);
%     end
%     rT = toc(t_reach); % End Reachability Timer
% 
%     %% Extract Sets and Safety Specification Check
%     t_gap = 1.4;
%     D_default = 10;
%     alp = 1;
%     outAll = [];
%     safe_dis = [];
% 
%     is_safe = true; % Assume safe until proven otherwise
% 
%     for i=1:length(trajR)
%         % Map to relative distance and safe distance Stars
%         % trajR(i) is guaranteed to be a single Star here due to the TNODEV Interval Hull
%         rel_dist_star = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
%         safe_dist_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%         outAll = [outAll rel_dist_star]; 
%         safe_dis = [safe_dis safe_dist_star]; 
% 
%         % Mathematical Safety Check (rel_dist > safe_dist)
%         % [min_rel_dist, ~] = rel_dist_star.getBox();
%         % [~, max_safe_dist] = safe_dist_star.getBox();
% 
%         % Proper getBox() syntax for safety check:
%         B_rel = rel_dist_star.getBox();
%         min_rel_dist = B_rel.lb;
% 
%         B_safe = safe_dist_star.getBox();
%         max_safe_dist = B_safe.ub;
% 
%         if min_rel_dist <= max_safe_dist
%             is_safe = false;
%         end
%     end
% 
%     % --- End Full Verification Timer ---
%     total_time = toc(t_full);
% 
%     % Save results 
%     save('results_tanhplant_TNODEV.mat','rT',"trajR","trajU", "hull_max_widths", "hull_volumes");
% 
%     %% Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (TNODEV)\n');
%     fprintf('=================================================\n');
%     if is_safe
%         fprintf('Result: SAFE (Relative distance is always > Safe distance)\n');
%     else
%         fprintf('Result: UNSAFE (Safety violation detected)\n');
%     end
%     fprintf('Reachability Loop Time: %.4f seconds\n', rT);
%     fprintf('Full Verification Time: %.4f seconds\n', total_time);
%     % fprintf('Final Hull Max Width: %.4f\n', hull_max_widths(end));
%     fprintf('=================================================\n\n');
% 
%     %% Visualize results
%     times = 0:0.1:0.1*N; % to plot in x-axis (time)
% 
%     % Create Main Distance Figure
%     f = figure;
%     hold on;
%     pb = plot(0,85,'m');
%     pr = plot(0,85,'k');
%     Star.plotRanges_2D(outAll,1,times,'k');
%     hold on;
%     Star.plotRanges_2D(safe_dis,1,times,'m');
% 
%     % Enhance figure for paper
%     ax = gca;
%     ax.XAxis.FontSize = 15;
%     ax.YAxis.FontSize = 15;
%     xlabel('Time (s)');
%     ylabel('Distance (m)');
%     ylim([40 110]);
%     legend([pr,pb],{'rel dist','safe dist'},"Location","best",'FontSize',14);
% 
%     % Save figure locally
%     fprintf('Saving figures to your current directory...\n');
%     exportgraphics(f, 'acc_tnodev_distance.pdf', 'ContentType', 'vector');
%     saveas(f, 'acc_tnodev_distance.png');
%     fprintf('Done!\n');
% 
% end


%% Code with refinements + 3 refinement optimization + parfor loop for the refinements

% % Changed `getRanges()` to `getBox()` as PJ mentioned on his email on
% 23/02/2026 and his `PJ_test_TIRA_calls.m`

% function reach_acc_TNODEV()
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
% 
%     %% 1. Configuration
%     % Select heuristic: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
%     heuristic_choice = 'naive'; 
% 
%     N = 50; % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations
% 
%     %% 2. Load objects
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = 17; 
%     ode_params.t_final = 0.1;      
%     ode_params.timeStep = 10;      
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); 
% 
%     %% 3. Initial Setup
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11]; 
%     v_ego = [30 30.2];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2]; 
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2]; 
% 
%     % Queue stores cells of {lb, ub}
%     queue = {};
%     queue{end+1} = {lb_init, ub_init};
% 
%     final_result = 'SAFE'; 
% 
%     %% --- OPTIMIZATION START ---
%     fprintf('\nInitializing Parallel Pool...\n');
%     p = gcp('nocreate'); 
%     if isempty(p)
%         % Start pool utilizing physical cores
%         p = parpool; 
%     end 
% 
%     % Define Load Factor (The Tweak)
%     % A factor of 3 to 5 is usually the "sweet spot" for uneven tasks.
%     worker_load_factor = 5;   % Max 8 based on the processor cores on Dell OptiPlex 7071 tower
% 
%     % Set Batch Size
%     batch_size = p.NumWorkers * worker_load_factor;
% 
%     fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%             p.NumWorkers, batch_size, worker_load_factor);
%     % --- OPTIMIZATION END ---
% 
%     t_full = tic;
%     iter = 1;
% 
%     %% 4. Main Parallel Refinement Loop (Batch Processing)
%     while ~isempty(queue)
% 
%         if iter > MAX_ITER
%             fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%             final_result = 'UNKNOWN (Wrapping effect dominated)';
%             break;
%         end
% 
%         % --- Extract strictly up to 'batch_size' tasks from the queue ---
%         num_tasks = min(length(queue), batch_size);
%         current_batch = queue(1:num_tasks);
%         queue(1:num_tasks) = []; % Remove extracted elements from queue
% 
%         fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                 iter, num_tasks, length(queue));
% 
%         % Preallocate results array for parfor
%         results = cell(1, num_tasks);
% 
%         % =========================================================
%         % PARALLEL COMPUTATION BLOCK
%         % =========================================================
%         parfor idx = 1:num_tasks
%             task_data = current_batch{idx};
%             lb_curr = task_data{1};
%             ub_curr = task_data{2};
% 
%             x_i = (lb_curr + ub_curr) / 2;
%             r_i = (ub_curr - lb_curr) / 2;
% 
%             X0 = Star(lb_curr, ub_curr);
%             is_subregion_safe = true;
% 
%             % Local copy of ode_params for the worker
%             local_ode_params = ode_params;
% 
%             % Track trajectory locally
%             trajR_local = X0;
% 
%             % --- N-Step Reachability Loop ---
%             for k = 1:N
%                 % 1. Extract Interval Hull
%                 lb_all = zeros(states, length(X0));
%                 ub_all = zeros(states, length(X0));
%                 for s_idx = 1:length(X0)
%                     % [lb_all(:, s_idx), ub_all(:, s_idx)] = X0(s_idx).getBox();
%                     B = X0(s_idx).getBox();
%                     lb_all(:, s_idx) = B.lb;
%                     ub_all(:, s_idx) = B.ub;
%                 end
%                 lb_hull = min(lb_all, [], 2);
%                 ub_hull = max(ub_all, [], 2);
% 
%                 % 2. TNODEV Step
%                 local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                 local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
%                 tnode_res = NODE_RA_New(local_ode_params);
% 
%                 % 3. Check Safety
%                 X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                 t_gap = 1.4; D_default = 10; alp = 1;
%                 rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                 safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%                 % [min_rel_dist, ~] = rel_dist_star.getBox();
%                 % [~, max_safe_dist] = safe_dist_star.getBox();
%                 % 
%                 % if min_rel_dist <= max_safe_dist
%                 %     is_subregion_safe = false;
%                 %     break; % Stop simulating this N-step loop early
%                 % end
% 
%                 % --- PROPER getBox() SYNTAX FOR SAFETY CHECK ---
%                 B_rel = rel_dist_star.getBox();
%                 min_rel_dist = B_rel.lb;
% 
%                 B_safe = safe_dist_star.getBox();
%                 max_safe_dist = B_safe.ub;
% 
%                 if min_rel_dist <= max_safe_dist
%                     is_subregion_safe = false;
%                     break; % Stop simulating, region is unsafe
%                 end
% 
%                 % Add to trajectory
%                 trajR_local = [trajR_local X0_new];
% 
%                 % 4. Controller Step & State Update (if safe)
%                 X0 = X0_new;
%                 ppp = X0.affineMap(map_mat,[]);
%                 Uin = U_fix.concatenate(ppp);
%                 Rc = net.reach(Uin);
% 
%                 x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                 X0 = X0.affineMap(C(1:6,:),[]); 
%                 X0 = X0.concatenate(Rc); 
%                 X0 = X0.concatenate(x08);
%             end
% 
%             % Map trajectory to Distance stars ONLY if it survived all N steps
%             outAll_local = [];
%             safe_dis_local = [];
%             if is_subregion_safe
%                 for i=1:length(trajR_local)
%                     rel_dist_star = trajR_local(i).affineMap([1 0 0 -1 0 0 0 0], []);
%                     safe_dist_star = trajR_local(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%                     outAll_local = [outAll_local rel_dist_star]; 
%                     safe_dis_local = [safe_dis_local safe_dist_star]; 
%                 end
%             end
% 
%             % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
%             split_dim = 1; 
%             if ~is_subregion_safe
%                 switch heuristic_choice
%                     case 'naive'
%                         [~, split_dim] = max(r_i);
% 
%                     case 'most-sensitive-input-radius'
%                         try
%                             [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, 17);
%                             absJ = max(abs(J_lb), abs(J_ub));
%                             sens = max(absJ, [], 1); 
%                             sens = max(sens, 1e-10);
%                             scores = r_i(:)' .* sens;
%                             [~, split_dim] = max(scores);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
% 
%                     case 'interval-norm-gradient'
%                         try
%                             % Get base norm for the initial box (1 step)
%                             base_p = local_ode_params;
%                             base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                             base_res = NODE_RA_New(base_p);
%                             base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                             grad = zeros(states, 1);
%                             delta = 1e-6;
%                             for j = 1:states
%                                 x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                 p_pert = base_p; p_pert.x0 = x_pert;
%                                 p_res = NODE_RA_New(p_pert);
%                                 pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                 grad(j) = (pert_norm - base_norm) / delta;
%                             end
%                             heuristic = abs(grad);
%                             [~, split_dim] = max(heuristic);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
%                 end
%             end
% 
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, 'split_dim', split_dim, 'lb', lb_curr, 'ub', ub_curr);
%         end
%         % =========================================================
% 
%         % --- Process Parallel Results & Rebuild Queue ---
%         unsafe_count = 0;
%         for idx = 1:num_tasks
%             res = results{idx};
%             if ~res.is_safe
%                 unsafe_count = unsafe_count + 1;
%                 split_dim = res.split_dim;
%                 lb_c = res.lb;
%                 ub_c = res.ub;
% 
%                 mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                 lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                 lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                 % Add new splits to the queue
%                 queue{end+1} = {lb_1, ub_1};
%                 queue{end+1} = {lb_2, ub_2};
%             end
%         end
% 
%         fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
%                 iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);
% 
%         iter = iter + 1;
%     end
% 
%     total_time = toc(t_full);
% 
%     %% Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%     fprintf('=================================================\n');
%     fprintf('Heuristic Used: %s\n', heuristic_choice);
%     fprintf('Final Result: %s\n', final_result);
%     fprintf('Total Batches Processed: %d\n', iter - 1);
%     fprintf('Full Verification Time: %.4f seconds\n', total_time);
%     fprintf('=================================================\n\n');
% 
% end

%% A version to be executed from Verifier_FINAL.m |---> Last working version as of 19/04/2026

% function reach_acc_TNODEV(heuristic_choice)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
% 
%     %% 1. Configuration 
% 
%     N = 50; % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations
% 
%     %% 2. Load objects
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = 17; 
%     ode_params.t_final = 0.1;      
%     ode_params.timeStep = 10;      
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); 
% 
%     %% 3. Initial Setup
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11]; 
%     v_ego = [30 30.2];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2]; 
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2]; 
% 
%     % Queue stores cells of {lb, ub}
%     queue = {};
%     queue{end+1} = {lb_init, ub_init};
% 
%     final_result = 'SAFE'; 
% 
%     %% --- OPTIMIZATION START ---
%     fprintf('\nInitializing Parallel Pool...\n');
%     p = gcp('nocreate'); 
%     if isempty(p)
%         % Start pool utilizing physical cores
%         p = parpool; 
%     end 
% 
%     % Define Load Factor (The Tweak)
%     % A factor of 3 to 5 is usually the "sweet spot" for uneven tasks.
%     worker_load_factor = 5;   % Max 8 based on the processor cores on Dell OptiPlex 7071 tower
% 
%     % Set Batch Size
%     batch_size = p.NumWorkers * worker_load_factor;
% 
%     fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%             p.NumWorkers, batch_size, worker_load_factor);
%     % --- OPTIMIZATION END ---
% 
%     t_full = tic;
%     iter = 1;
% 
%     %% 4. Main Parallel Refinement Loop (Batch Processing)
%     while ~isempty(queue)
% 
%         if iter > MAX_ITER
%             fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%             final_result = 'UNKNOWN (Wrapping effect dominated)';
%             break;
%         end
% 
%         % --- Extract strictly up to 'batch_size' tasks from the queue ---
%         num_tasks = min(length(queue), batch_size);
%         current_batch = queue(1:num_tasks);
%         queue(1:num_tasks) = []; % Remove extracted elements from queue
% 
%         fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                 iter, num_tasks, length(queue));
% 
%         % Preallocate results array for parfor
%         results = cell(1, num_tasks);
% 
%         % =========================================================
%         % PARALLEL COMPUTATION BLOCK
%         % =========================================================
%         parfor idx = 1:num_tasks
%             task_data = current_batch{idx};
%             lb_curr = task_data{1};
%             ub_curr = task_data{2};
% 
%             x_i = (lb_curr + ub_curr) / 2;
%             r_i = (ub_curr - lb_curr) / 2;
% 
%             X0 = Star(lb_curr, ub_curr);
%             is_subregion_safe = true;
% 
%             % Local copy of ode_params for the worker
%             local_ode_params = ode_params;
% 
%             % Track trajectory locally
%             trajR_local = X0;
% 
%             % --- N-Step Reachability Loop ---
%             for k = 1:N
%                 % 1. Extract Interval Hull
%                 lb_all = zeros(states, length(X0));
%                 ub_all = zeros(states, length(X0));
%                 for s_idx = 1:length(X0)
%                     % [lb_all(:, s_idx), ub_all(:, s_idx)] = X0(s_idx).getBox();
%                     B = X0(s_idx).getBox();
%                     lb_all(:, s_idx) = B.lb;
%                     ub_all(:, s_idx) = B.ub;
%                 end
%                 lb_hull = min(lb_all, [], 2);
%                 ub_hull = max(ub_all, [], 2);
% 
%                 % 2. TNODEV Step
%                 local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                 local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
%                 tnode_res = NODE_RA_New(local_ode_params);
% 
%                 % 3. Check Safety
%                 X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                 t_gap = 1.4; D_default = 10; alp = 1;
%                 rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                 safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%                 % [min_rel_dist, ~] = rel_dist_star.getBox();
%                 % [~, max_safe_dist] = safe_dist_star.getBox();
%                 % 
%                 % if min_rel_dist <= max_safe_dist
%                 %     is_subregion_safe = false;
%                 %     break; % Stop simulating this N-step loop early
%                 % end
% 
%                 % --- PROPER getBox() SYNTAX FOR SAFETY CHECK ---
%                 B_rel = rel_dist_star.getBox();
%                 min_rel_dist = B_rel.lb;
% 
%                 B_safe = safe_dist_star.getBox();
%                 max_safe_dist = B_safe.ub;
% 
%                 if min_rel_dist <= max_safe_dist
%                     is_subregion_safe = false;
%                     break; % Stop simulating, region is unsafe
%                 end
% 
%                 % Add to trajectory
%                 trajR_local = [trajR_local X0_new];
% 
%                 % 4. Controller Step & State Update (if safe)
%                 X0 = X0_new;
%                 ppp = X0.affineMap(map_mat,[]);
%                 Uin = U_fix.concatenate(ppp);
%                 Rc = net.reach(Uin);
% 
%                 x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                 X0 = X0.affineMap(C(1:6,:),[]); 
%                 X0 = X0.concatenate(Rc); 
%                 X0 = X0.concatenate(x08);
%             end
% 
%             % Map trajectory to Distance stars ONLY if it survived all N steps
%             outAll_local = [];
%             safe_dis_local = [];
%             if is_subregion_safe
%                 for i=1:length(trajR_local)
%                     rel_dist_star = trajR_local(i).affineMap([1 0 0 -1 0 0 0 0], []);
%                     safe_dist_star = trajR_local(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%                     outAll_local = [outAll_local rel_dist_star]; 
%                     safe_dis_local = [safe_dis_local safe_dist_star]; 
%                 end
%             end
% 
%             % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
%             split_dim = 1; 
%             if ~is_subregion_safe
%                 switch heuristic_choice
%                     case 'naive'
%                         [~, split_dim] = max(r_i);
% 
%                     case 'most-sensitive-input-radius'
%                         try
%                             [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, 17);
%                             absJ = max(abs(J_lb), abs(J_ub));
%                             sens = max(absJ, [], 1); 
%                             sens = max(sens, 1e-10);
%                             scores = r_i(:)' .* sens;
%                             [~, split_dim] = max(scores);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
% 
%                     case 'interval-norm-gradient'
%                         try
%                             % Get base norm for the initial box (1 step)
%                             base_p = local_ode_params;
%                             base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                             base_res = NODE_RA_New(base_p);
%                             base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                             grad = zeros(states, 1);
%                             delta = 1e-6;
%                             for j = 1:states
%                                 x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                 p_pert = base_p; p_pert.x0 = x_pert;
%                                 p_res = NODE_RA_New(p_pert);
%                                 pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                 grad(j) = (pert_norm - base_norm) / delta;
%                             end
%                             heuristic = abs(grad);
%                             [~, split_dim] = max(heuristic);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
%                 end
%             end
% 
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, 'split_dim', split_dim, 'lb', lb_curr, 'ub', ub_curr);
%         end
%         % =========================================================
% 
%         % --- Process Parallel Results & Rebuild Queue ---
%         unsafe_count = 0;
%         for idx = 1:num_tasks
%             res = results{idx};
%             if ~res.is_safe
%                 unsafe_count = unsafe_count + 1;
%                 split_dim = res.split_dim;
%                 lb_c = res.lb;
%                 ub_c = res.ub;
% 
%                 mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                 lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                 lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                 % Add new splits to the queue
%                 queue{end+1} = {lb_1, ub_1};
%                 queue{end+1} = {lb_2, ub_2};
%             end
%         end
% 
%         fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
%                 iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);
% 
%         iter = iter + 1;
%     end
% 
%     total_time = toc(t_full);
% 
%     %% Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%     fprintf('=================================================\n');
%     fprintf('Heuristic Used: %s\n', heuristic_choice);
%     fprintf('Final Result: %s\n', final_result);
%     fprintf('Total Batches Processed: %d\n', iter - 1);
%     fprintf('Full Verification Time: %.4f seconds\n', total_time);
%     fprintf('=================================================\n\n');
% 
% end

%% Added the linear ACC exampless

% function reach_acc_TNODEV(heuristic_choice, system_choice)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %                      (defaults to 17 for backward compatibility)
% 
%     if nargin < 2 || isempty(system_choice)
%         system_choice = 17;   % default: nonlinear NODE
%     end
%     if ~ismember(system_choice, [17, 24])
%         error('reach_acc_TNODEV: system_choice must be 17 (nonlinear) or 24 (linear), got %d.', system_choice);
%     end
% 
%     switch system_choice
%         case 17, sys_name = 'Nonlinear NODE';
%         case 24, sys_name = 'Linear NODE';
%     end
% 
%     % Note: system_choice is a broadcast variable inside the parfor loop;
%     % every worker sees the same scalar value assigned at function entry.
% 
%     %% 1. Configuration
% 
%     N = 50;        % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations
% 
%     %% 2. Load objects
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = system_choice;   % was hardcoded to 17
%     ode_params.t_final = 0.1;
%     ode_params.timeStep = 10;
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]);
% 
%     %% 3. Initial Setup
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11];
%     v_ego = [30 30.2];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2];
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2];
% 
%     % Queue stores cells of {lb, ub}
%     queue = {};
%     queue{end+1} = {lb_init, ub_init};
% 
%     final_result = 'SAFE';
% 
%     %% --- OPTIMIZATION START ---
%     fprintf('\nInitializing Parallel Pool...\n');
%     p = gcp('nocreate');
%     if isempty(p)
%         % Start pool utilizing physical cores
%         p = parpool;
%     end
% 
%     % Define Load Factor (The Tweak)
%     % A factor of 3 to 5 is usually the "sweet spot" for uneven tasks.
%     worker_load_factor = 5;   % Max 8 based on the processor cores on Dell OptiPlex 7071 tower
% 
%     % Set Batch Size
%     batch_size = p.NumWorkers * worker_load_factor;
% 
%     fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%             p.NumWorkers, batch_size, worker_load_factor);
%     % --- OPTIMIZATION END ---
% 
%     t_full = tic;
%     iter = 1;
% 
%     %% 4. Main Parallel Refinement Loop (Batch Processing)
%     while ~isempty(queue)
% 
%         if iter > MAX_ITER
%             fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%             final_result = 'UNKNOWN (Wrapping effect dominated)';
%             break;
%         end
% 
%         % --- Extract strictly up to 'batch_size' tasks from the queue ---
%         num_tasks = min(length(queue), batch_size);
%         current_batch = queue(1:num_tasks);
%         queue(1:num_tasks) = []; % Remove extracted elements from queue
% 
%         fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                 iter, num_tasks, length(queue));
% 
%         % Preallocate results array for parfor
%         results = cell(1, num_tasks);
% 
%         % =========================================================
%         % PARALLEL COMPUTATION BLOCK
%         % =========================================================
%         parfor idx = 1:num_tasks
%             task_data = current_batch{idx};
%             lb_curr = task_data{1};
%             ub_curr = task_data{2};
% 
%             x_i = (lb_curr + ub_curr) / 2;
%             r_i = (ub_curr - lb_curr) / 2;
% 
%             X0 = Star(lb_curr, ub_curr);
%             is_subregion_safe = true;
% 
%             % Local copy of ode_params for the worker
%             local_ode_params = ode_params;
% 
%             % Track trajectory locally
%             trajR_local = X0;
% 
%             % --- N-Step Reachability Loop ---
%             for k = 1:N
%                 % 1. Extract Interval Hull
%                 lb_all = zeros(states, length(X0));
%                 ub_all = zeros(states, length(X0));
%                 for s_idx = 1:length(X0)
%                     B = X0(s_idx).getBox();
%                     lb_all(:, s_idx) = B.lb;
%                     ub_all(:, s_idx) = B.ub;
%                 end
%                 lb_hull = min(lb_all, [], 2);
%                 ub_hull = max(ub_all, [], 2);
% 
%                 % 2. TNODEV Step
%                 local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                 local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
%                 tnode_res = NODE_RA_New(local_ode_params);
% 
%                 % 3. Check Safety
%                 X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                 t_gap = 1.4; D_default = 10; alp = 1;
%                 rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                 safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%                 % --- PROPER getBox() SYNTAX FOR SAFETY CHECK ---
%                 B_rel = rel_dist_star.getBox();
%                 min_rel_dist = B_rel.lb;
% 
%                 B_safe = safe_dist_star.getBox();
%                 max_safe_dist = B_safe.ub;
% 
%                 if min_rel_dist <= max_safe_dist
%                     is_subregion_safe = false;
%                     break; % Stop simulating, region is unsafe
%                 end
% 
%                 % Add to trajectory
%                 trajR_local = [trajR_local X0_new];
% 
%                 % 4. Controller Step & State Update (if safe)
%                 X0 = X0_new;
%                 ppp = X0.affineMap(map_mat,[]);
%                 Uin = U_fix.concatenate(ppp);
%                 Rc = net.reach(Uin);
% 
%                 x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                 X0 = X0.affineMap(C(1:6,:),[]);
%                 X0 = X0.concatenate(Rc);
%                 X0 = X0.concatenate(x08);
%             end
% 
%             % Map trajectory to Distance stars ONLY if it survived all N steps
%             outAll_local = [];
%             safe_dis_local = [];
%             if is_subregion_safe
%                 for i=1:length(trajR_local)
%                     rel_dist_star = trajR_local(i).affineMap([1 0 0 -1 0 0 0 0], []);
%                     safe_dist_star = trajR_local(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%                     outAll_local = [outAll_local rel_dist_star];
%                     safe_dis_local = [safe_dis_local safe_dist_star];
%                 end
%             end
% 
%             % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
%             split_dim = 1;
%             if ~is_subregion_safe
%                 switch heuristic_choice
%                     case 'naive'
%                         [~, split_dim] = max(r_i);
% 
%                     case 'most-sensitive-input-radius'
%                         try
%                             [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, system_choice);
%                             absJ = max(abs(J_lb), abs(J_ub));
%                             sens = max(absJ, [], 1);
%                             sens = max(sens, 1e-10);
%                             scores = r_i(:)' .* sens;
%                             [~, split_dim] = max(scores);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
% 
%                     case 'interval-norm-gradient'
%                         try
%                             % Get base norm for the initial box (1 step)
%                             base_p = local_ode_params;
%                             base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                             base_res = NODE_RA_New(base_p);
%                             base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                             grad = zeros(states, 1);
%                             delta = 1e-6;
%                             for j = 1:states
%                                 x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                 p_pert = base_p; p_pert.x0 = x_pert;
%                                 p_res = NODE_RA_New(p_pert);
%                                 pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                 grad(j) = (pert_norm - base_norm) / delta;
%                             end
%                             heuristic = abs(grad);
%                             [~, split_dim] = max(heuristic);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
%                 end
%             end
% 
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, 'split_dim', split_dim, 'lb', lb_curr, 'ub', ub_curr);
%         end
%         % =========================================================
% 
%         % --- Process Parallel Results & Rebuild Queue ---
%         unsafe_count = 0;
%         for idx = 1:num_tasks
%             res = results{idx};
%             if ~res.is_safe
%                 unsafe_count = unsafe_count + 1;
%                 split_dim = res.split_dim;
%                 lb_c = res.lb;
%                 ub_c = res.ub;
% 
%                 mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                 lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                 lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                 % Add new splits to the queue
%                 queue{end+1} = {lb_1, ub_1};
%                 queue{end+1} = {lb_2, ub_2};
%             end
%         end
% 
%         fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
%                 iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);
% 
%         iter = iter + 1;
%     end
% 
%     total_time = toc(t_full);
% 
%     %% ---------------------------------------------------------------
%     %  PLOTTING PASS: single-pass simulation from the original initial set
%     %  Used only to produce the distance-over-time figure. The refinement
%     %  verdict above is what determines safety; this is just visualization.
%     %  ---------------------------------------------------------------
%     fprintf('\nRunning single-pass plotting trajectory for visualization...\n');
%     X0_plot = Star(lb_init, ub_init);
%     trajR_plot = X0_plot;
% 
%     plot_ode_params = ode_params;
% 
%     for k = 1:N
%         lb_all = zeros(states, length(X0_plot));
%         ub_all = zeros(states, length(X0_plot));
%         for s_idx = 1:length(X0_plot)
%             B = X0_plot(s_idx).getBox();
%             lb_all(:, s_idx) = B.lb;
%             ub_all(:, s_idx) = B.ub;
%         end
%         lb_hull = min(lb_all, [], 2);
%         ub_hull = max(ub_all, [], 2);
% 
%         plot_ode_params.x0 = (lb_hull + ub_hull) / 2;
%         plot_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
%         tnode_res = NODE_RA_New(plot_ode_params);
% 
%         X0_plot = Star(tnode_res.intersect_low, tnode_res.intersect_up);
%         trajR_plot = [trajR_plot X0_plot];
% 
%         ppp = X0_plot.affineMap(map_mat, []);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
% 
%         x08 = X0_plot.affineMap([0 0 0 0 0 0 0 1], []);
%         X0_plot = X0_plot.affineMap(C(1:6, :), []);
%         X0_plot = X0_plot.concatenate(Rc);
%         X0_plot = X0_plot.concatenate(x08);
%     end
% 
%     % Extract distance bounds from the trajectory
%     t_gap = 1.4; D_default = 10; alp = 1;
%     M = length(trajR_plot);
%     rel_dist_min  = zeros(1, M);
%     rel_dist_max  = zeros(1, M);
%     safe_dist_min = zeros(1, M);
%     safe_dist_max = zeros(1, M);
%     for i = 1:M
%         rel_dist_star  = trajR_plot(i).affineMap([1 0 0 -1 0 0 0 0], []);
%         safe_dist_star = trajR_plot(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%         B_rel  = rel_dist_star.getBox();
%         B_safe = safe_dist_star.getBox();
%         rel_dist_min(i)  = B_rel.lb;
%         rel_dist_max(i)  = B_rel.ub;
%         safe_dist_min(i) = B_safe.lb;
%         safe_dist_max(i) = B_safe.ub;
%     end
% 
%     % Render the discrete-box figure (matches the reference figure you shared)
%     times = 0:0.1:0.1*N;
%     f = figure('Color', 'w'); hold on;
%     for i = 1:(length(times)-1)
%         t_start = times(i);
%         t_width = times(i+1) - times(i);
% 
%         y_safe_min = safe_dist_min(i);
%         y_safe_max = safe_dist_max(i);
%         rectangle('Position', [t_start, y_safe_min, t_width, y_safe_max - y_safe_min], ...
%                   'EdgeColor', 'm', 'LineWidth', 0.5);
% 
%         y_rel_min = rel_dist_min(i);
%         y_rel_max = rel_dist_max(i);
%         rectangle('Position', [t_start, y_rel_min, t_width, y_rel_max - y_rel_min], ...
%                   'EdgeColor', 'k', 'LineWidth', 0.5);
%     end
% 
%     % Dummy handles for the legend
%     pr = plot(NaN, NaN, 'k', 'LineWidth', 1.5);
%     pb = plot(NaN, NaN, 'm', 'LineWidth', 1.5);
% 
%     ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
%     xlabel('Time (s)',    'FontSize', 16, 'FontWeight', 'bold');
%     ylabel('Distance (m)','FontSize', 16, 'FontWeight', 'bold');
%     ylim([40 110]);
%     legend([pr, pb], {'rel dist', 'safe dist'}, 'Location', 'best', 'FontSize', 14);
% 
%     % Save per-system, per-heuristic so nonlinear and linear runs don't overwrite
%     fig_stem = sprintf('acc_sys%d_%s', system_choice, heuristic_choice);
%     exportgraphics(f, [fig_stem '.pdf'], 'ContentType', 'vector');
%     saveas(f, [fig_stem '.png']);
%     fprintf('Saved distance plot: %s.png / .pdf\n', fig_stem);
% 
% 
%     %% Persist results to disk (before printing, so a print error can't lose them)
%     try
%         save_filename = sprintf('results_acc_sys%d_%s.mat', system_choice, heuristic_choice,datestr(now, 'yyyymmdd_HHMMSS'));
%         save(save_filename, ...
%              'total_time', 'final_result', 'iter', ...
%              'heuristic_choice', 'system_choice', 'sys_name', ...
%              'N', 'MAX_ITER', 'batch_size');
%         fprintf('Results saved to: %s\n', save_filename);
%     catch ME
%         warning('reach_acc_TNODEV: failed to save results (%s)', ME.message);
%     end
% 
%     %% Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%     fprintf('=================================================\n');
%     fprintf('System:          %s (choice=%d)\n', sys_name, system_choice);
%     fprintf('Heuristic Used:  %s\n', heuristic_choice);
%     fprintf('Final Result:    %s\n', final_result);
%     fprintf('Total Batches:   %d\n', iter - 1);
%     fprintf('Full Time:       %.4f seconds\n', total_time);
%     fprintf('=================================================\n\n');
% 
% end


%% Adding plotting selector for both single and refined passes

% function reach_acc_TNODEV(heuristic_choice, system_choice, plot_mode)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %   plot_mode        : 'single_pass' | 'refined_union' | 'none'
%     %                      (defaults to 'single_pass')
%     %
%     % Outputs are saved under:
%     %   ./results/acc_sys<ID>_<heuristic>_<timestamp>/
%     %     - results.mat      (verdict, timings, config)
%     %     - distance.pdf     (vector figure)
%     %     - distance.png     (raster figure)
% 
%     if nargin < 2 || isempty(system_choice)
%         system_choice = 17;   % default: nonlinear NODE
%     end
%     if nargin < 3 || isempty(plot_mode)
%         plot_mode = 'single_pass';
%     end
%     if ~ismember(system_choice, [17, 24])
%         error('reach_acc_TNODEV: system_choice must be 17 (nonlinear) or 24 (linear), got %d.', system_choice);
%     end
%     if ~ismember(plot_mode, {'single_pass', 'refined_union', 'none'})
%         error('reach_acc_TNODEV: plot_mode must be ''single_pass'', ''refined_union'', or ''none''.');
%     end
% 
%     switch system_choice
%         case 17, sys_name = 'Nonlinear NODE';
%         case 24, sys_name = 'Linear NODE';
%     end
% 
%     % Note: system_choice is a broadcast variable inside the parfor loop;
%     % every worker sees the same scalar value assigned at function entry.
% 
%     %% 1. Configuration
% 
%     N = 50;         % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations
% 
%     %% 2. Load objects
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = system_choice;
%     ode_params.t_final = 0.1;
%     ode_params.timeStep = 10;
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]);
% 
%     %% 3. Initial Setup
%     xlead = [90 110];
%     % v_lead = [32 32.2]; % orig.
%     % v_lead = [31, 33];
%     v_lead=[31.8, 32.4];
% 
%     x_ego = [10 11];
%     % v_ego = [30 30.2]; % orig.
%     % v_ego = [29, 31];
%     v_ego=[29.8, 30.4];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2];
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2];
% 
%     % Queue stores cells of {lb, ub}
%     queue = {};
%     queue{end+1} = {lb_init, ub_init};
% 
%     final_result = 'SAFE';
% 
%     %% 3b. Prepare output folder (unique per run via timestamp)
%     timestamp = datestr(now, 'yyyymmdd_HHMMSS');
%     run_name = sprintf('acc_sys%d_%s_%s', system_choice, heuristic_choice, timestamp);
%     out_dir = fullfile(pwd, 'results', run_name);
%     if ~exist(out_dir, 'dir')
%         mkdir(out_dir);
%     end
%     fprintf('Output folder: %s\n', out_dir);
% 
%     %% --- OPTIMIZATION START ---
%     fprintf('\nInitializing Parallel Pool...\n');
%     p = gcp('nocreate');
%     if isempty(p)
%         p = parpool;
%     end
% 
%     worker_load_factor = 5;
%     batch_size = p.NumWorkers * worker_load_factor;
% 
%     fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%             p.NumWorkers, batch_size, worker_load_factor);
%     % --- OPTIMIZATION END ---
% 
%     t_full = tic;
%     iter = 1;
% 
%     %% 4. Main Parallel Refinement Loop (Batch Processing)
% 
%     % For plot_mode == 'refined_union', we need to collect the per-cell
%     % trajectories. For other modes we skip this to avoid memory overhead.
%     collect_trajectories = strcmp(plot_mode, 'refined_union');
%     verified_trajectories = {};   % will hold cell arrays of Star objects
% 
%     while ~isempty(queue)
% 
%         if iter > MAX_ITER
%             fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%             final_result = 'UNKNOWN (Wrapping effect dominated)';
%             break;
%         end
% 
%         num_tasks = min(length(queue), batch_size);
%         current_batch = queue(1:num_tasks);
%         queue(1:num_tasks) = [];
% 
%         fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                 iter, num_tasks, length(queue));
% 
%         results = cell(1, num_tasks);
% 
%         % =========================================================
%         % PARALLEL COMPUTATION BLOCK
%         % =========================================================
%         parfor idx = 1:num_tasks
%             task_data = current_batch{idx};
%             lb_curr = task_data{1};
%             ub_curr = task_data{2};
% 
%             x_i = (lb_curr + ub_curr) / 2;
%             r_i = (ub_curr - lb_curr) / 2;
% 
%             X0 = Star(lb_curr, ub_curr);
%             is_subregion_safe = true;
% 
%             local_ode_params = ode_params;
%             trajR_local = X0;
% 
%             for k = 1:N
%                 lb_all = zeros(states, length(X0));
%                 ub_all = zeros(states, length(X0));
%                 for s_idx = 1:length(X0)
%                     B = X0(s_idx).getBox();
%                     lb_all(:, s_idx) = B.lb;
%                     ub_all(:, s_idx) = B.ub;
%                 end
%                 lb_hull = min(lb_all, [], 2);
%                 ub_hull = max(ub_all, [], 2);
% 
%                 local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                 local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
%                 tnode_res = NODE_RA_New(local_ode_params);
% 
%                 X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                 t_gap = 1.4; D_default = 10; alp = 1; % orig
%                 % t_gap = 2.0; D_default = 20; alp = 1; 
%                 rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                 safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%                 B_rel = rel_dist_star.getBox();
%                 min_rel_dist = B_rel.lb;
% 
%                 B_safe = safe_dist_star.getBox();
%                 max_safe_dist = B_safe.ub;
% 
%                 if min_rel_dist <= max_safe_dist
%                     is_subregion_safe = false;
%                     break;
%                 end
% 
%                 trajR_local = [trajR_local X0_new];
% 
%                 X0 = X0_new;
%                 ppp = X0.affineMap(map_mat,[]);
%                 Uin = U_fix.concatenate(ppp);
%                 Rc = net.reach(Uin);
% 
%                 x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                 X0 = X0.affineMap(C(1:6,:),[]);
%                 X0 = X0.concatenate(Rc);
%                 X0 = X0.concatenate(x08);
%             end
% 
%             % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
%             split_dim = 1;
%             if ~is_subregion_safe
%                 switch heuristic_choice
%                     case 'naive'
%                         [~, split_dim] = max(r_i);
% 
%                     case 'most-sensitive-input-radius'
%                         try
%                             [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, system_choice);
%                             absJ = max(abs(J_lb), abs(J_ub));
%                             sens = max(absJ, [], 1);
%                             sens = max(sens, 1e-10);
%                             scores = r_i(:)' .* sens;
%                             [~, split_dim] = max(scores);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
% 
%                     case 'interval-norm-gradient'
%                         try
%                             base_p = local_ode_params;
%                             base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                             base_res = NODE_RA_New(base_p);
%                             base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                             grad = zeros(states, 1);
%                             delta = 1e-6;
%                             for j = 1:states
%                                 x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                 p_pert = base_p; p_pert.x0 = x_pert;
%                                 p_res = NODE_RA_New(p_pert);
%                                 pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                 grad(j) = (pert_norm - base_norm) / delta;
%                             end
%                             heuristic = abs(grad);
%                             [~, split_dim] = max(heuristic);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
%                 end
%             end
% 
%             % Save worker results (conditionally include trajectory)
%             if collect_trajectories && is_subregion_safe
%                 results{idx} = struct('is_safe', is_subregion_safe, ...
%                                       'split_dim', split_dim, ...
%                                       'lb', lb_curr, 'ub', ub_curr, ...
%                                       'traj', {trajR_local});
%             else
%                 results{idx} = struct('is_safe', is_subregion_safe, ...
%                                       'split_dim', split_dim, ...
%                                       'lb', lb_curr, 'ub', ub_curr, ...
%                                       'traj', {{}});
%             end
%         end
%         % =========================================================
% 
%         % --- Process Parallel Results & Rebuild Queue ---
%         unsafe_count = 0;
%         for idx = 1:num_tasks
%             res = results{idx};
%             if ~res.is_safe
%                 unsafe_count = unsafe_count + 1;
%                 split_dim = res.split_dim;
%                 lb_c = res.lb;
%                 ub_c = res.ub;
% 
%                 mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                 lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                 lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                 queue{end+1} = {lb_1, ub_1};
%                 queue{end+1} = {lb_2, ub_2};
%             else
%                 % Collect verified-cell trajectory if requested
%                 if collect_trajectories && ~isempty(res.traj)
%                     verified_trajectories{end+1} = res.traj;
%                 end
%             end
%         end
% 
%         fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
%                 iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);
% 
%         iter = iter + 1;
%     end
% 
%     total_time = toc(t_full);
% 
%     %% 5. Plotting pass (user-selectable mode)
%     fig_path = '';
%     try
%         switch plot_mode
%             case 'single_pass'
%                 fig_path = local_plot_single_pass(out_dir, ...
%                     lb_init, ub_init, N, states, ode_params, ...
%                     map_mat, U_fix, net, C, sys_name, final_result);
% 
%             case 'refined_union'
%                 if isempty(verified_trajectories)
%                     fprintf('[plot] No verified trajectories to plot (verdict = %s). ', final_result);
%                     fprintf('Falling back to single_pass.\n');
%                     fig_path = local_plot_single_pass(out_dir, ...
%                         lb_init, ub_init, N, states, ode_params, ...
%                         map_mat, U_fix, net, C, sys_name, final_result);
%                 else
%                     fig_path = local_plot_refined_union(out_dir, ...
%                         verified_trajectories, N, sys_name, final_result);
%                 end
% 
%             case 'none'
%                 fprintf('[plot] Skipped (plot_mode = ''none'').\n');
%         end
%     catch ME
%         warning('reach_acc_TNODEV: plotting failed (%s)', ME.message);
%     end
% 
%     %% 6. Persist results to disk
%     try
%         save_filename = fullfile(out_dir, 'results.mat');
%         save(save_filename, ...
%              'total_time', 'final_result', 'iter', ...
%              'heuristic_choice', 'system_choice', 'sys_name', ...
%              'N', 'MAX_ITER', 'batch_size', 'plot_mode', 'fig_path');
%         fprintf('Results saved to: %s\n', save_filename);
%     catch ME
%         warning('reach_acc_TNODEV: failed to save results (%s)', ME.message);
%     end
% 
%     %% 7. Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%     fprintf('=================================================\n');
%     fprintf('System:          %s (choice=%d)\n', sys_name, system_choice);
%     fprintf('Heuristic Used:  %s\n', heuristic_choice);
%     fprintf('Plot Mode:       %s\n', plot_mode);
%     fprintf('Final Result:    %s\n', final_result);
%     fprintf('Total Batches:   %d\n', iter - 1);
%     fprintf('Full Time:       %.4f seconds\n', total_time);
%     fprintf('Output Folder:   %s\n', out_dir);
%     fprintf('=================================================\n\n');
% 
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Option A — Single-pass plot from the original initial set
% %  Runs one non-refined NNCS simulation from lb_init..ub_init and plots
% %  the rel-dist vs safe-dist envelope as discrete per-step boxes.
% % ======================================================================
% function fig_path = local_plot_single_pass(out_dir, lb_init, ub_init, N, states, ...
%     ode_params, map_mat, U_fix, net, C, sys_name, final_result)
% 
%     fprintf('\n[plot] Running single-pass visualization trajectory...\n');
%     X0 = Star(lb_init, ub_init);
%     trajR = X0;
% 
%     local_ode = ode_params;
% 
%     for k = 1:N
%         lb_all = zeros(states, length(X0));
%         ub_all = zeros(states, length(X0));
%         for s_idx = 1:length(X0)
%             B = X0(s_idx).getBox();
%             lb_all(:, s_idx) = B.lb;
%             ub_all(:, s_idx) = B.ub;
%         end
%         lb_hull = min(lb_all, [], 2);
%         ub_hull = max(ub_all, [], 2);
% 
%         local_ode.x0 = (lb_hull + ub_hull) / 2;
%         local_ode.Initial_radius = (ub_hull - lb_hull) / 2;
%         tnode_res = NODE_RA_New(local_ode);
% 
%         X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
%         trajR = [trajR X0];
% 
%         ppp = X0.affineMap(map_mat, []);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
% 
%         x08 = X0.affineMap([0 0 0 0 0 0 0 1], []);
%         X0 = X0.affineMap(C(1:6, :), []);
%         X0 = X0.concatenate(Rc);
%         X0 = X0.concatenate(x08);
%     end
% 
%     fig_path = render_distance_figure(out_dir, trajR, N, sys_name, final_result, 'single_pass');
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Option B — Plot union of refined verified trajectories
% %  For each time-step k, takes the envelope (min lb, max ub) across all
% %  verified cells' per-step stars. This is a sound over-approximation of
% %  the reachable tube that was actually certified by refinement.
% % ======================================================================
% function fig_path = local_plot_refined_union(out_dir, verified_trajectories, N, sys_name, final_result)
% 
%     fprintf('\n[plot] Aggregating %d verified-cell trajectories...\n', length(verified_trajectories));
% 
%     % Each cell may have up to N+1 entries (initial + one per step).
%     % We take the union box per time index across all cells.
%     t_gap = 1.4; D_default = 10; alp = 1;
% 
%     % First, determine the maximum trajectory length (should be N+1 for safe cells).
%     max_len = 0;
%     for c = 1:length(verified_trajectories)
%         max_len = max(max_len, length(verified_trajectories{c}));
%     end
% 
%     rel_dist_min  =  inf(1, max_len);
%     rel_dist_max  = -inf(1, max_len);
%     safe_dist_min =  inf(1, max_len);
%     safe_dist_max = -inf(1, max_len);
% 
%     for c = 1:length(verified_trajectories)
%         traj = verified_trajectories{c};
%         for i = 1:length(traj)
%             rel_star  = traj(i).affineMap([1 0 0 -1 0 0 0 0], []);
%             safe_star = traj(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%             B_rel  = rel_star.getBox();
%             B_safe = safe_star.getBox();
% 
%             rel_dist_min(i)  = min(rel_dist_min(i),  B_rel.lb);
%             rel_dist_max(i)  = max(rel_dist_max(i),  B_rel.ub);
%             safe_dist_min(i) = min(safe_dist_min(i), B_safe.lb);
%             safe_dist_max(i) = max(safe_dist_max(i), B_safe.ub);
%         end
%     end
% 
%     fig_path = render_distance_figure_from_bounds(out_dir, ...
%         rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%         N, sys_name, final_result, 'refined_union');
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: given a single Star-array trajectory, render the figure
% % ======================================================================
% function fig_path = render_distance_figure(out_dir, trajR, N, sys_name, final_result, tag)
%     t_gap = 1.4; D_default = 10; alp = 1;
% 
%     M = length(trajR);
%     rel_dist_min  = zeros(1, M);
%     rel_dist_max  = zeros(1, M);
%     safe_dist_min = zeros(1, M);
%     safe_dist_max = zeros(1, M);
% 
%     for i = 1:M
%         rel_star  = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
%         safe_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%         B_rel  = rel_star.getBox();
%         B_safe = safe_star.getBox();
%         rel_dist_min(i)  = B_rel.lb;
%         rel_dist_max(i)  = B_rel.ub;
%         safe_dist_min(i) = B_safe.lb;
%         safe_dist_max(i) = B_safe.ub;
%     end
% 
%     fig_path = render_distance_figure_from_bounds(out_dir, ...
%         rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%         N, sys_name, final_result, tag);
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: render the per-step boxed figure from 4 bound vectors
% % ======================================================================
% function fig_path = render_distance_figure_from_bounds(out_dir, ...
%     rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%     N, sys_name, final_result, tag)
% 
%     times = 0:0.1:0.1*N;
%     L = min(length(times)-1, length(rel_dist_min));   % guard against short trajectories
% 
%     f = figure('Color', 'w', 'Visible', 'off'); hold on;
%     for i = 1:L
%         t_start = times(i);
%         t_width = times(i+1) - times(i);
% 
%         y_safe_min = safe_dist_min(i);
%         y_safe_max = safe_dist_max(i);
%         if isfinite(y_safe_min) && isfinite(y_safe_max) && y_safe_max > y_safe_min
%             rectangle('Position', [t_start, y_safe_min, t_width, y_safe_max - y_safe_min], ...
%                       'EdgeColor', 'm', 'LineWidth', 0.5);
%         end
% 
%         y_rel_min = rel_dist_min(i);
%         y_rel_max = rel_dist_max(i);
%         if isfinite(y_rel_min) && isfinite(y_rel_max) && y_rel_max > y_rel_min
%             rectangle('Position', [t_start, y_rel_min, t_width, y_rel_max - y_rel_min], ...
%                       'EdgeColor', 'k', 'LineWidth', 0.5);
%         end
%     end
% 
%     pr = plot(NaN, NaN, 'k', 'LineWidth', 1.5);
%     pb = plot(NaN, NaN, 'm', 'LineWidth', 1.5);
% 
%     ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
%     xlabel('Time (s)',     'FontSize', 16, 'FontWeight', 'bold');
%     ylabel('Distance (m)', 'FontSize', 16, 'FontWeight', 'bold');
%     ylim([40 110]);
% 
%     title(sprintf('%s — verdict: %s', sys_name, final_result), ...
%           'Interpreter', 'none', 'FontSize', 14);
%     legend([pr, pb], {'rel dist', 'safe dist'}, 'Location', 'best', 'FontSize', 14);
% 
%     fig_stem = sprintf('distance_%s', tag);
%     pdf_path = fullfile(out_dir, [fig_stem '.pdf']);
%     png_path = fullfile(out_dir, [fig_stem '.png']);
%     exportgraphics(f, pdf_path, 'ContentType', 'vector');
%     saveas(f, png_path);
%     close(f);
% 
%     fig_path = pdf_path;
%     fprintf('[plot] Saved: %s\n', pdf_path);
% end

%% Aded CORA explosion  try/check

% function reach_acc_TNODEV(heuristic_choice, system_choice, plot_mode)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %           + Robust CORA/TIRA explosion handling
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %   plot_mode        : 'single_pass' | 'refined_union' | 'none'
%     %                      (defaults to 'single_pass')
%     %
%     % Outputs are saved under:
%     %   ./results/acc_sys<ID>_<heuristic>_<timestamp>/
%     %     - results.mat      (verdict, timings, config)
%     %     - distance.pdf     (vector figure)
%     %     - distance.png     (raster figure)
% 
%     if nargin < 2 || isempty(system_choice)
%         system_choice = 17;   % default: nonlinear NODE
%     end
%     if nargin < 3 || isempty(plot_mode)
%         plot_mode = 'single_pass';
%     end
%     if ~ismember(system_choice, [17, 24])
%         error('reach_acc_TNODEV: system_choice must be 17 (nonlinear) or 24 (linear), got %d.', system_choice);
%     end
%     if ~ismember(plot_mode, {'single_pass', 'refined_union', 'none'})
%         error('reach_acc_TNODEV: plot_mode must be ''single_pass'', ''refined_union'', or ''none''.');
%     end
% 
%     switch system_choice
%         case 17, sys_name = 'Nonlinear NODE';
%         case 24, sys_name = 'Linear NODE';
%     end
% 
%     % Note: system_choice is a broadcast variable inside the parfor loop;
%     % every worker sees the same scalar value assigned at function entry.
% 
%     %% 1. Configuration
% 
%     N = 50;         % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations
% 
%     % Safety spec (used by both refinement check and plotting)
%     SPEC_T_GAP     = 1.4;
%     SPEC_D_DEFAULT = 10;
%     SPEC_ALPHA     = 1;
% 
%     %% 2. Load objects
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = system_choice;
%     ode_params.t_final = 0.1;
%     ode_params.timeStep = 10;
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]);
% 
%     %% 3. Initial Setup
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11];
%     v_ego = [30 30.2];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2];
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2];
% 
%     % Queue stores cells of {lb, ub}
%     queue = {};
%     queue{end+1} = {lb_init, ub_init};
% 
%     final_result = 'SAFE';
% 
%     %% 3b. Prepare output folder (unique per run via timestamp)
%     timestamp = datestr(now, 'yyyymmdd_HHMMSS');
%     run_name = sprintf('acc_sys%d_%s_%s', system_choice, heuristic_choice, timestamp);
%     out_dir = fullfile(pwd, 'results', run_name);
%     if ~exist(out_dir, 'dir')
%         mkdir(out_dir);
%     end
%     fprintf('Output folder: %s\n', out_dir);
% 
%     %% --- OPTIMIZATION START ---
%     fprintf('\nInitializing Parallel Pool...\n');
%     p = gcp('nocreate');
%     if isempty(p)
%         p = parpool;
%     end
% 
%     worker_load_factor = 5;
%     batch_size = p.NumWorkers * worker_load_factor;
% 
%     fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%             p.NumWorkers, batch_size, worker_load_factor);
%     % --- OPTIMIZATION END ---
% 
%     t_full = tic;
%     iter = 1;
% 
%     %% 4. Main Parallel Refinement Loop (Batch Processing)
% 
%     % For plot_mode == 'refined_union', we need to collect the per-cell
%     % trajectories. For other modes we skip this to avoid memory overhead.
%     collect_trajectories = strcmp(plot_mode, 'refined_union');
%     verified_trajectories = {};   % will hold cell arrays of Star objects
% 
%     % Track CORA/TIRA explosion count for diagnostics
%     total_explosions = 0;
% 
%     while ~isempty(queue)
% 
%         if iter > MAX_ITER
%             fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%             final_result = 'UNKNOWN (Wrapping effect dominated)';
%             break;
%         end
% 
%         num_tasks = min(length(queue), batch_size);
%         current_batch = queue(1:num_tasks);
%         queue(1:num_tasks) = [];
% 
%         fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                 iter, num_tasks, length(queue));
% 
%         results = cell(1, num_tasks);
% 
%         % =========================================================
%         % PARALLEL COMPUTATION BLOCK
%         % =========================================================
%         parfor idx = 1:num_tasks
%             task_data = current_batch{idx};
%             lb_curr = task_data{1};
%             ub_curr = task_data{2};
% 
%             x_i = (lb_curr + ub_curr) / 2;
%             r_i = (ub_curr - lb_curr) / 2;
% 
%             X0 = Star(lb_curr, ub_curr);
%             is_subregion_safe = true;
%             had_explosion = false;   % track if this cell triggered a CORA/TIRA failure
% 
%             local_ode_params = ode_params;
%             trajR_local = X0;
% 
%             % --- N-Step Reachability Loop ---
%             for k = 1:N
%                 % 1. Extract Interval Hull
%                 lb_all = zeros(states, length(X0));
%                 ub_all = zeros(states, length(X0));
%                 for s_idx = 1:length(X0)
%                     B = X0(s_idx).getBox();
%                     lb_all(:, s_idx) = B.lb;
%                     ub_all(:, s_idx) = B.ub;
%                 end
%                 lb_hull = min(lb_all, [], 2);
%                 ub_hull = max(ub_all, [], 2);
% 
%                 local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                 local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
% 
%                 % ==========================================================
%                 %  TRY-CATCH BLOCK to handle CORA reachable-set explosion
%                 %  or TIRA failure on cells that are too large.
%                 %  When either occurs, mark the cell as unsafe and break so
%                 %  the heuristic split logic will bisect it on the next batch.
%                 % ==========================================================
%                 try
%                     % 2. TNODEV Step
%                     tnode_res = NODE_RA_New(local_ode_params);
% 
%                     % 3. Check Safety
%                     X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                     rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                     safe_dist_star = X0_new.affineMap( ...
%                         [0 0 0 0 SPEC_ALPHA*SPEC_T_GAP 0 0 0], SPEC_ALPHA*SPEC_D_DEFAULT);
% 
%                     B_rel = rel_dist_star.getBox();
%                     min_rel_dist = B_rel.lb;
% 
%                     B_safe = safe_dist_star.getBox();
%                     max_safe_dist = B_safe.ub;
% 
%                     if min_rel_dist <= max_safe_dist
%                         is_subregion_safe = false;
%                         break; % Stop simulating, region is unsafe
%                     end
% 
%                 catch ME %#ok<NASGU>
%                     % CORA reachable-set explosion or TIRA failure.
%                     % Cannot prove safety for this cell at its current size.
%                     % Mark as unsafe so refinement splits it next iteration.
%                     is_subregion_safe = false;
%                     had_explosion = true;
%                     break;
%                 end
%                 % ==========================================================
% 
%                 % Add to trajectory
%                 trajR_local = [trajR_local X0_new];
% 
%                 % 4. Controller Step & State Update (if safe)
%                 X0 = X0_new;
%                 ppp = X0.affineMap(map_mat,[]);
%                 Uin = U_fix.concatenate(ppp);
%                 Rc = net.reach(Uin);
% 
%                 x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                 X0 = X0.affineMap(C(1:6,:),[]);
%                 X0 = X0.concatenate(Rc);
%                 X0 = X0.concatenate(x08);
%             end
% 
%             % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
%             split_dim = 1;
%             if ~is_subregion_safe
%                 switch heuristic_choice
%                     case 'naive'
%                         [~, split_dim] = max(r_i);
% 
%                     case 'most-sensitive-input-radius'
%                         % MSIR itself could fail on exploded cells
%                         % (Jacobian bounding calls CORA internally for case 17).
%                         % Fall back to naive on any failure.
%                         try
%                             [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, system_choice);
%                             absJ = max(abs(J_lb), abs(J_ub));
%                             sens = max(absJ, [], 1);
%                             sens = max(sens, 1e-10);
%                             scores = r_i(:)' .* sens;
%                             [~, split_dim] = max(scores);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
% 
%                     case 'interval-norm-gradient'
%                         % ING makes n extra NODE_RA_New calls per decision.
%                         % Any one can explode. Fall back to naive on failure.
%                         try
%                             base_p = local_ode_params;
%                             base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                             base_res = NODE_RA_New(base_p);
%                             base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                             grad = zeros(states, 1);
%                             delta = 1e-6;
%                             for j = 1:states
%                                 x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                 p_pert = base_p; p_pert.x0 = x_pert;
%                                 p_res = NODE_RA_New(p_pert);
%                                 pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                 grad(j) = (pert_norm - base_norm) / delta;
%                             end
%                             heuristic = abs(grad);
%                             [~, split_dim] = max(heuristic);
%                         catch
%                             [~, split_dim] = max(r_i);
%                         end
%                 end
%             end
% 
%             % Save worker results (conditionally include trajectory)
%             if collect_trajectories && is_subregion_safe
%                 results{idx} = struct('is_safe', is_subregion_safe, ...
%                                       'split_dim', split_dim, ...
%                                       'lb', lb_curr, 'ub', ub_curr, ...
%                                       'had_explosion', had_explosion, ...
%                                       'traj', {trajR_local});
%             else
%                 results{idx} = struct('is_safe', is_subregion_safe, ...
%                                       'split_dim', split_dim, ...
%                                       'lb', lb_curr, 'ub', ub_curr, ...
%                                       'had_explosion', had_explosion, ...
%                                       'traj', {{}});
%             end
%         end
%         % =========================================================
% 
%         % --- Process Parallel Results & Rebuild Queue ---
%         unsafe_count = 0;
%         explosion_count = 0;
%         for idx = 1:num_tasks
%             res = results{idx};
%             if res.had_explosion
%                 explosion_count = explosion_count + 1;
%             end
%             if ~res.is_safe
%                 unsafe_count = unsafe_count + 1;
%                 split_dim = res.split_dim;
%                 lb_c = res.lb;
%                 ub_c = res.ub;
% 
%                 mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                 lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                 lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                 queue{end+1} = {lb_1, ub_1};
%                 queue{end+1} = {lb_2, ub_2};
%             else
%                 % Collect verified-cell trajectory if requested
%                 if collect_trajectories && ~isempty(res.traj)
%                     verified_trajectories{end+1} = res.traj;
%                 end
%             end
%         end
%         total_explosions = total_explosions + explosion_count;
% 
%         fprintf('  -> Batch %d complete: %d safe, %d unsafe (%d CORA explosions, spawned %d new sets).\n', ...
%                 iter, num_tasks - unsafe_count, unsafe_count, explosion_count, unsafe_count * 2);
% 
%         % Early exit: if every single cell exploded AND the queue is not
%         % shrinking, we're in a degenerate regime where sub-cells won't
%         % get small enough to help. Warn but keep going up to MAX_ITER.
%         if explosion_count == num_tasks && num_tasks >= batch_size
%             fprintf('  [!] All %d cells in this batch exploded. Refinement may not converge.\n', num_tasks);
%         end
% 
%         iter = iter + 1;
%     end
% 
%     total_time = toc(t_full);
% 
%     if total_explosions > 0
%         fprintf('\n[diagnostic] Total CORA/TIRA explosions across run: %d\n', total_explosions);
%     end
% 
%     %% 5. Plotting pass (user-selectable mode)
%     fig_path = '';
%     try
%         switch plot_mode
%             case 'single_pass'
%                 fig_path = local_plot_single_pass(out_dir, ...
%                     lb_init, ub_init, N, states, ode_params, ...
%                     map_mat, U_fix, net, C, sys_name, final_result, ...
%                     SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
% 
%             case 'refined_union'
%                 if isempty(verified_trajectories)
%                     fprintf('[plot] No verified trajectories to plot (verdict = %s). ', final_result);
%                     fprintf('Falling back to single_pass.\n');
%                     fig_path = local_plot_single_pass(out_dir, ...
%                         lb_init, ub_init, N, states, ode_params, ...
%                         map_mat, U_fix, net, C, sys_name, final_result, ...
%                         SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
%                 else
%                     fig_path = local_plot_refined_union(out_dir, ...
%                         verified_trajectories, N, sys_name, final_result, ...
%                         SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
%                 end
% 
%             case 'none'
%                 fprintf('[plot] Skipped (plot_mode = ''none'').\n');
%         end
%     catch ME
%         warning('reach_acc_TNODEV: plotting failed (%s)', ME.message);
%     end
% 
%     %% 6. Persist results to disk
%     try
%         save_filename = fullfile(out_dir, 'results.mat');
%         save(save_filename, ...
%              'total_time', 'final_result', 'iter', ...
%              'heuristic_choice', 'system_choice', 'sys_name', ...
%              'N', 'MAX_ITER', 'batch_size', 'plot_mode', 'fig_path', ...
%              'total_explosions', ...
%              'SPEC_T_GAP', 'SPEC_D_DEFAULT', 'SPEC_ALPHA');
%         fprintf('Results saved to: %s\n', save_filename);
%     catch ME
%         warning('reach_acc_TNODEV: failed to save results (%s)', ME.message);
%     end
% 
%     %% 7. Print Output Summaries
%     fprintf('\n=================================================\n');
%     fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%     fprintf('=================================================\n');
%     fprintf('System:          %s (choice=%d)\n', sys_name, system_choice);
%     fprintf('Heuristic Used:  %s\n', heuristic_choice);
%     fprintf('Plot Mode:       %s\n', plot_mode);
%     fprintf('Final Result:    %s\n', final_result);
%     fprintf('Total Batches:   %d\n', iter - 1);
%     fprintf('Explosions:      %d\n', total_explosions);
%     fprintf('Full Time:       %.4f seconds\n', total_time);
%     fprintf('Output Folder:   %s\n', out_dir);
%     fprintf('=================================================\n\n');
% 
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Option A — Single-pass plot from the original initial set
% % ======================================================================
% function fig_path = local_plot_single_pass(out_dir, lb_init, ub_init, N, states, ...
%     ode_params, map_mat, U_fix, net, C, sys_name, final_result, ...
%     t_gap, D_default, alp)
% 
%     fprintf('\n[plot] Running single-pass visualization trajectory...\n');
%     X0 = Star(lb_init, ub_init);
%     trajR = X0;
% 
%     local_ode = ode_params;
% 
%     for k = 1:N
%         lb_all = zeros(states, length(X0));
%         ub_all = zeros(states, length(X0));
%         for s_idx = 1:length(X0)
%             B = X0(s_idx).getBox();
%             lb_all(:, s_idx) = B.lb;
%             ub_all(:, s_idx) = B.ub;
%         end
%         lb_hull = min(lb_all, [], 2);
%         ub_hull = max(ub_all, [], 2);
% 
%         local_ode.x0 = (lb_hull + ub_hull) / 2;
%         local_ode.Initial_radius = (ub_hull - lb_hull) / 2;
% 
%         % Single-pass plot is best-effort. If CORA explodes at the top
%         % level, we print a warning and abort the plot gracefully.
%         try
%             tnode_res = NODE_RA_New(local_ode);
%         catch ME
%             warning(['local_plot_single_pass: NODE_RA_New failed at step %d (%s). ' ...
%                      'Single-pass plot will be truncated.'], k, ME.message);
%             break;
%         end
% 
%         X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
%         trajR = [trajR X0];
% 
%         ppp = X0.affineMap(map_mat, []);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
% 
%         x08 = X0.affineMap([0 0 0 0 0 0 0 1], []);
%         X0 = X0.affineMap(C(1:6, :), []);
%         X0 = X0.concatenate(Rc);
%         X0 = X0.concatenate(x08);
%     end
% 
%     fig_path = render_distance_figure(out_dir, trajR, N, sys_name, final_result, ...
%         'single_pass', t_gap, D_default, alp);
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Option B — Plot union of refined verified trajectories
% % ======================================================================
% function fig_path = local_plot_refined_union(out_dir, verified_trajectories, N, ...
%     sys_name, final_result, t_gap, D_default, alp)
% 
%     fprintf('\n[plot] Aggregating %d verified-cell trajectories...\n', length(verified_trajectories));
% 
%     max_len = 0;
%     for c = 1:length(verified_trajectories)
%         max_len = max(max_len, length(verified_trajectories{c}));
%     end
% 
%     rel_dist_min  =  inf(1, max_len);
%     rel_dist_max  = -inf(1, max_len);
%     safe_dist_min =  inf(1, max_len);
%     safe_dist_max = -inf(1, max_len);
% 
%     for c = 1:length(verified_trajectories)
%         traj = verified_trajectories{c};
%         for i = 1:length(traj)
%             rel_star  = traj(i).affineMap([1 0 0 -1 0 0 0 0], []);
%             safe_star = traj(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%             B_rel  = rel_star.getBox();
%             B_safe = safe_star.getBox();
% 
%             rel_dist_min(i)  = min(rel_dist_min(i),  B_rel.lb);
%             rel_dist_max(i)  = max(rel_dist_max(i),  B_rel.ub);
%             safe_dist_min(i) = min(safe_dist_min(i), B_safe.lb);
%             safe_dist_max(i) = max(safe_dist_max(i), B_safe.ub);
%         end
%     end
% 
%     fig_path = render_distance_figure_from_bounds(out_dir, ...
%         rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%         N, sys_name, final_result, 'refined_union');
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: given a single Star-array trajectory, render the figure
% % ======================================================================
% function fig_path = render_distance_figure(out_dir, trajR, N, sys_name, final_result, ...
%     tag, t_gap, D_default, alp)
% 
%     M = length(trajR);
%     rel_dist_min  = zeros(1, M);
%     rel_dist_max  = zeros(1, M);
%     safe_dist_min = zeros(1, M);
%     safe_dist_max = zeros(1, M);
% 
%     for i = 1:M
%         rel_star  = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
%         safe_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
%         B_rel  = rel_star.getBox();
%         B_safe = safe_star.getBox();
%         rel_dist_min(i)  = B_rel.lb;
%         rel_dist_max(i)  = B_rel.ub;
%         safe_dist_min(i) = B_safe.lb;
%         safe_dist_max(i) = B_safe.ub;
%     end
% 
%     fig_path = render_distance_figure_from_bounds(out_dir, ...
%         rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%         N, sys_name, final_result, tag);
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: render the per-step boxed figure from 4 bound vectors
% % ======================================================================
% function fig_path = render_distance_figure_from_bounds(out_dir, ...
%     rel_dist_min, rel_dist_max, safe_dist_min, safe_dist_max, ...
%     N, sys_name, final_result, tag)
% 
%     times = 0:0.1:0.1*N;
%     L = min(length(times)-1, length(rel_dist_min));
% 
%     f = figure('Color', 'w', 'Visible', 'off'); hold on;
%     for i = 1:L
%         t_start = times(i);
%         t_width = times(i+1) - times(i);
% 
%         y_safe_min = safe_dist_min(i);
%         y_safe_max = safe_dist_max(i);
%         if isfinite(y_safe_min) && isfinite(y_safe_max) && y_safe_max > y_safe_min
%             rectangle('Position', [t_start, y_safe_min, t_width, y_safe_max - y_safe_min], ...
%                       'EdgeColor', 'm', 'LineWidth', 0.5);
%         end
% 
%         y_rel_min = rel_dist_min(i);
%         y_rel_max = rel_dist_max(i);
%         if isfinite(y_rel_min) && isfinite(y_rel_max) && y_rel_max > y_rel_min
%             rectangle('Position', [t_start, y_rel_min, t_width, y_rel_max - y_rel_min], ...
%                       'EdgeColor', 'k', 'LineWidth', 0.5);
%         end
%     end
% 
%     pr = plot(NaN, NaN, 'k', 'LineWidth', 1.5);
%     pb = plot(NaN, NaN, 'm', 'LineWidth', 1.5);
% 
%     ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
%     xlabel('Time (s)',     'FontSize', 16, 'FontWeight', 'bold');
%     ylabel('Distance (m)', 'FontSize', 16, 'FontWeight', 'bold');
%     ylim([40 110]);
% 
%     title(sprintf('%s — verdict: %s', sys_name, final_result), ...
%           'Interpreter', 'none', 'FontSize', 14);
%     legend([pr, pb], {'rel dist', 'safe dist'}, 'Location', 'best', 'FontSize', 14);
% 
%     fig_stem = sprintf('distance_%s', tag);
%     pdf_path = fullfile(out_dir, [fig_stem '.pdf']);
%     png_path = fullfile(out_dir, [fig_stem '.png']);
%     exportgraphics(f, pdf_path, 'ContentType', 'vector');
%     saveas(f, png_path);
%     close(f);
% 
%     fig_path = pdf_path;
%     fprintf('[plot] Saved: %s\n', pdf_path);
% end


%% [17-03-2026] A version which allows the user to choose in `Verifier_FINAL.m` or `Verifier_MAIN_CALL.m` between Parallel Iterative Refinement and Single-Pass Verification Similar to NNV2.0 |--> Not working do it as an optimization later on 

% %------------- BEGIN OF CODE --------------
% function reach_acc_TNODEV(do_refinement, heuristic_choice)
%     % Reachability analysis of ACC using TNODEV
%     % Handles both Single-Pass (Distance Plotting) and Parallel Refinement
% 
%     %% 1. Configuration & Common Setup
%     N = 70; % 50 steps total (5 seconds)
%     MAX_ITER = 100; % Maximum number of batch iterations (for refinement)
% 
%     % Load controller
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     ode_params = struct();
%     ode_params.system_choice = 17; 
%     ode_params.t_final = 0.1;      
%     ode_params.timeStep = 10;      
% 
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
% 
%     map_mat = [0 0 0 0 1 0 0 0;
%                1 0 0 -1 0 0 0 0;
%                0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); 
% 
%     % Initial Setup (NNV Benchmark)
%     xlead = [90 110];
%     v_lead = [32 32.2];
%     x_ego = [10 11]; 
%     v_ego = [30 30.2];
% 
%     lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2]; 
%     ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2]; 
% 
%     % =====================================================================
%     % BRANCH A: SINGLE-PASS VERIFICATION (NO REFINEMENT)
%     % =====================================================================
%     if ~do_refinement
%         fprintf('Starting Single-Pass Verification...\n');
%         X0 = Star(lb_init, ub_init); 
% 
%         trajR = X0;
%         trajU = [];
% 
%         hull_max_widths = zeros(1, N);
%         hull_volumes = zeros(1, N);
% 
%         t_full = tic;
%         t_reach = tic;
% 
%         for k=1:N
%             fprintf('Computing step %d / %d...\n', k, N);
% 
%             % 1. Extract Interval Hull
%             lb_all = zeros(states, length(X0));
%             ub_all = zeros(states, length(X0));
%             for s_idx = 1:length(X0)
%                 B = X0(s_idx).getBox();
%                 lb_all(:, s_idx) = B.lb;
%                 ub_all(:, s_idx) = B.ub;
%             end
%             lb_current = min(lb_all, [], 2);
%             ub_current = max(ub_all, [], 2);
% 
%             % 2. Setup TNODEV parameters
%             ode_params.x0 = (lb_current + ub_current) / 2;
%             ode_params.Initial_radius = (ub_current - lb_current) / 2;
% 
%             % 3. Run Interval Reachability
%             tnode_res = NODE_RA_New(ode_params);
% 
%             % Metrics
%             current_widths = tnode_res.intersect_up - tnode_res.intersect_low;
%             hull_max_widths(k) = max(current_widths);
%             hull_volumes(k) = prod(current_widths); 
% 
%             % 4. State Update (TNODEV -> NNV Controller)
%             X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
%             trajR = [trajR X0]; 
% 
%             ppp = X0.affineMap(map_mat,[]);
%             Uin = U_fix.concatenate(ppp);
%             Rc = net.reach(Uin);
%             trajU = [trajU Rc];
% 
%             x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%             X0 = X0.affineMap(C(1:6,:),[]); 
%             X0 = X0.concatenate(Rc); 
%             X0 = X0.concatenate(x08);
%         end
%         rT = toc(t_reach); 
% 
%         %% Safety Check and Plotting
%         t_gap = 1.4; D_default = 10; alp = 1;
%         is_safe = true; 
% 
%         % Pre-allocate arrays to hold the bounds for plotting
%         rel_dist_min = zeros(1, length(trajR));
%         rel_dist_max = zeros(1, length(trajR));
%         safe_dist_min = zeros(1, length(trajR));
%         safe_dist_max = zeros(1, length(trajR));
% 
%         for i=1:length(trajR)
%             rel_dist_star = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
%             safe_dist_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%             % Extract and store Relative Distance bounds
%             B_rel = rel_dist_star.getBox();
%             rel_dist_min(i) = B_rel.lb;
%             rel_dist_max(i) = B_rel.ub;
% 
%             % Extract and store Safe Distance bounds
%             B_safe = safe_dist_star.getBox();
%             safe_dist_min(i) = B_safe.lb;
%             safe_dist_max(i) = B_safe.ub;
% 
%             % Safety Check
%             if B_rel.lb <= B_safe.ub
%                 is_safe = false;
%             end
%         end
%         total_time = toc(t_full);
% 
%         % Summaries and Saves
%         save('results_acc_single_pass.mat','rT',"trajR","trajU", "hull_max_widths", "hull_volumes");
% 
%         fprintf('\n=================================================\n');
%         fprintf('VERIFICATION SUMMARY (SINGLE-PASS)\n');
%         fprintf('=================================================\n');
%         if is_safe
%             fprintf('Result: SAFE (Relative distance is always > Safe distance)\n');
%         else
%             fprintf('Result: UNSAFE (Safety violation detected. Try enabling Refinement!)\n');
%         end
%         fprintf('Reachability Time: %.4f seconds\n', rT);
%         fprintf('Total Time: %.4f seconds\n', total_time);
%         fprintf('Final Hull Max Width: %.4f\n', hull_max_widths(end));
%         fprintf('=================================================\n\n');
% 
%         % =========================================================
%         % Visualization (Manual Fill to Bypass NNV Plotting Bugs)          |---> Continuous Figure 
%         % =========================================================
%         % times = 0:0.1:0.1*N;
%         % f = figure('Color', 'w'); hold on; grid on;
%         % 
%         % % Create the X polygon array (forward then backward)
%         % X_fill = [times, fliplr(times)];
%         % 
%         % % 1. Plot Safe Distance Tube (Magenta, translucent)
%         % Y_safe_fill = [safe_dist_min, fliplr(safe_dist_max)];
%         % pb = fill(X_fill, Y_safe_fill, 'm', 'EdgeColor', 'm', 'FaceAlpha', 0.3);
%         % 
%         % % 2. Plot Relative Distance Tube (Grey/Black, translucent)
%         % Y_rel_fill = [rel_dist_min, fliplr(rel_dist_max)];
%         % pr = fill(X_fill, Y_rel_fill, [0.6 0.6 0.6], 'EdgeColor', 'k', 'FaceAlpha', 0.6);
%         % 
%         % % Formatting
%         % ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
%         % xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
%         % ylabel('Distance (m)', 'FontSize', 16, 'FontWeight', 'bold'); 
%         % ylim([40 110]);
%         % 
%         % % Because pb and pr are native MATLAB fill handles, this legend will NEVER crash!
%         % legend([pr, pb], {'Relative Distance', 'Safe Distance'}, "Location", "best", 'FontSize', 14);
%         % 
%         % exportgraphics(f, 'acc_single_distance.pdf', 'ContentType', 'vector');
%         % saveas(f, 'acc_single_distance.png');
%         % fprintf('Saved distance plots to current directory.\n');
% 
% 
%         % =========================================================
%         % Visualization (Manual Boxes to Bypass NNV Plotting Bugs)         |---> Similar discrete boxes Figure to the one in NNV2.0
%         % =========================================================
%         times = 0:0.1:0.1*N;
%         f = figure('Color', 'w'); hold on;
% 
%         % Loop through each time step and draw the boxes manually
%         for i = 1:(length(times)-1)
%             t_start = times(i);
%             t_width = times(i+1) - times(i);
% 
%             % 1. Plot Safe Distance Box (Magenta edges, no fill)
%             y_safe_min = safe_dist_min(i);
%             y_safe_max = safe_dist_max(i);
%             rectangle('Position', [t_start, y_safe_min, t_width, y_safe_max - y_safe_min], ...
%                       'EdgeColor', 'm', 'LineWidth', 0.5);
% 
%             % 2. Plot Relative Distance Box (Black edges, no fill)
%             y_rel_min = rel_dist_min(i);
%             y_rel_max = rel_dist_max(i);
%             rectangle('Position', [t_start, y_rel_min, t_width, y_rel_max - y_rel_min], ...
%                       'EdgeColor', 'k', 'LineWidth', 0.5);
%         end
% 
%         % Create dummy handles for the legend
%         pr = plot(NaN, NaN, 'k', 'LineWidth', 1.5);
%         pb = plot(NaN, NaN, 'm', 'LineWidth', 1.5);
% 
%         % Formatting
%         ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
%         xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
%         ylabel('Distance (m)', 'FontSize', 16, 'FontWeight', 'bold'); 
%         ylim([40 110]);
% 
%         % Because pb and pr are native MATLAB plot handles, this legend will NEVER crash!
%         legend([pr, pb], {'Relative Distance', 'Safe Distance'}, "Location", "best", 'FontSize', 14);
% 
%         exportgraphics(f, 'acc_single_distance.pdf', 'ContentType', 'vector');
%         saveas(f, 'acc_single_distance.png');
%         fprintf('Saved distance plots to current directory.\n');
% 
%     % =====================================================================
%     % BRANCH B: PARALLEL REFINEMENT
%     % =====================================================================
%     else
%         queue = {};
%         queue{end+1} = {lb_init, ub_init};
%         final_result = 'SAFE'; 
% 
%         fprintf('\nInitializing Parallel Pool...\n');
%         p = gcp('nocreate'); 
%         if isempty(p), p = parpool; end 
%         worker_load_factor = 5;   
%         batch_size = p.NumWorkers * worker_load_factor;
% 
%         fprintf('  [Parallel Config] Workers: %d | Batch Size: %d\n', p.NumWorkers, batch_size);
% 
%         t_full = tic;
%         iter = 1;
% 
%         while ~isempty(queue)
%             if iter > MAX_ITER
%                 fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
%                 final_result = 'UNKNOWN (Wrapping effect dominated)';
%                 break;
%             end
% 
%             num_tasks = min(length(queue), batch_size);
%             current_batch = queue(1:num_tasks);
%             queue(1:num_tasks) = []; 
% 
%             fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
%                     iter, num_tasks, length(queue));
% 
%             results = cell(1, num_tasks);
% 
%             parfor idx = 1:num_tasks
%                 task_data = current_batch{idx};
%                 lb_curr = task_data{1};
%                 ub_curr = task_data{2};
% 
%                 x_i = (lb_curr + ub_curr) / 2;
%                 r_i = (ub_curr - lb_curr) / 2;
% 
%                 X0 = Star(lb_curr, ub_curr);
%                 is_subregion_safe = true;
%                 local_ode_params = ode_params;
% 
%                 % --- N-Step Reachability Loop ---
%                 for k = 1:N
%                     lb_all = zeros(states, length(X0));
%                     ub_all = zeros(states, length(X0));
%                     for s_idx = 1:length(X0)
%                         B = X0(s_idx).getBox();
%                         lb_all(:, s_idx) = B.lb;
%                         ub_all(:, s_idx) = B.ub;
%                     end
%                     lb_hull = min(lb_all, [], 2);
%                     ub_hull = max(ub_all, [], 2);
% 
%                     local_ode_params.x0 = (lb_hull + ub_hull) / 2;
%                     local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
% 
%                     % ==========================================================
%                     % NEW: TRY-CATCH BLOCK TO HANDLE CORA/TIRA EXPLOSIONS
%                     % ==========================================================
%                     try
%                         % 2. TNODEV Step
%                         tnode_res = NODE_RA_New(local_ode_params);
% 
%                         % 3. Check Safety
%                         X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);
% 
%                         t_gap = 1.4; D_default = 10; alp = 1;
%                         rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
%                         safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
% 
%                         B_rel = rel_dist_star.getBox(); min_rel_dist = B_rel.lb;
%                         B_safe = safe_dist_star.getBox(); max_safe_dist = B_safe.ub;
% 
%                         if min_rel_dist <= max_safe_dist
%                             is_subregion_safe = false;
%                             break; % Stop simulating, region is unsafe
%                         end
% 
%                     catch ME
%                         % If CORA explodes or TIRA fails, we cannot prove safety.
%                         % Mark it as unsafe so the heuristic splits it and tries again!
%                         is_subregion_safe = false;
%                         break; 
%                     end
%                     % ==========================================================
% 
%                     X0 = X0_new;
%                     ppp = X0.affineMap(map_mat,[]);
%                     Uin = U_fix.concatenate(ppp);
%                     Rc = net.reach(Uin);
% 
%                     x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%                     X0 = X0.affineMap(C(1:6,:),[]); 
%                     X0 = X0.concatenate(Rc); 
%                     X0 = X0.concatenate(x08);
%                 end
% 
%                 split_dim = 1; 
%                 if ~is_subregion_safe
%                     switch heuristic_choice
%                         case 'naive'
%                             [~, split_dim] = max(r_i);
%                         case 'most-sensitive-input-radius'
%                             try
%                                 [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, 17);
%                                 absJ = max(abs(J_lb), abs(J_ub));
%                                 sens = max(max(absJ, [], 1), 1e-10); 
%                                 [~, split_dim] = max(r_i(:)' .* sens);
%                             catch
%                                 [~, split_dim] = max(r_i);
%                             end
%                         case 'interval-norm-gradient'
%                             try
%                                 base_p = local_ode_params;
%                                 base_p.x0 = x_i; base_p.Initial_radius = r_i;
%                                 base_res = NODE_RA_New(base_p);
%                                 base_norm = sum(base_res.intersect_up - base_res.intersect_low);
% 
%                                 grad = zeros(states, 1);
%                                 delta = 1e-6;
%                                 for j = 1:states
%                                     x_pert = x_i; x_pert(j) = x_pert(j) + delta;
%                                     p_pert = base_p; p_pert.x0 = x_pert;
%                                     p_res = NODE_RA_New(p_pert);
%                                     pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
%                                     grad(j) = (pert_norm - base_norm) / delta;
%                                 end
%                                 [~, split_dim] = max(abs(grad));
%                             catch
%                                 [~, split_dim] = max(r_i);
%                             end
%                     end
%                 end
%                 results{idx} = struct('is_safe', is_subregion_safe, 'split_dim', split_dim, 'lb', lb_curr, 'ub', ub_curr);
%             end
% 
%             unsafe_count = 0;
%             for idx = 1:num_tasks
%                 res = results{idx};
%                 if ~res.is_safe
%                     unsafe_count = unsafe_count + 1;
%                     split_dim = res.split_dim;
%                     lb_c = res.lb; ub_c = res.ub;
%                     mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;
% 
%                     lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
%                     lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;
% 
%                     queue{end+1} = {lb_1, ub_1};
%                     queue{end+1} = {lb_2, ub_2};
%                 end
%             end
%             fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
%                     iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);
%             iter = iter + 1;
%         end
%         total_time = toc(t_full);
% 
%         fprintf('\n=================================================\n');
%         fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
%         fprintf('=================================================\n');
%         fprintf('Heuristic Used: %s\n', heuristic_choice);
%         fprintf('Final Result: %s\n', final_result);
%         fprintf('Total Batches Processed: %d\n', iter - 1);
%         fprintf('Full Verification Time: %.4f seconds\n', total_time);
%         fprintf('=================================================\n\n');
%     end
% end
% %------------- END OF CODE --------------


