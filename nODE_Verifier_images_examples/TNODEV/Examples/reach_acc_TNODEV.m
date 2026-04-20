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

function reach_acc_TNODEV(heuristic_choice, system_choice)

    % Reachability analysis of ACC using TNODEV with Iterative Refinement
    % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
    %
    % Inputs:
    %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
    %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
    %                      24 = linear   3rd-order NODE (no activation)
    %                      (defaults to 17 for backward compatibility)

    if nargin < 2 || isempty(system_choice)
        system_choice = 17;   % default: nonlinear NODE
    end
    if ~ismember(system_choice, [17, 24])
        error('reach_acc_TNODEV: system_choice must be 17 (nonlinear) or 24 (linear), got %d.', system_choice);
    end

    switch system_choice
        case 17, sys_name = 'Nonlinear NODE';
        case 24, sys_name = 'Linear NODE';
    end

    % Note: system_choice is a broadcast variable inside the parfor loop;
    % every worker sees the same scalar value assigned at function entry.

    %% 1. Configuration

    N = 50;        % 50 steps total (5 seconds)
    MAX_ITER = 100; % Maximum number of batch iterations

    %% 2. Load objects
    net = load_NN_from_mat('controller_5_20.mat');

    ode_params = struct();
    ode_params.system_choice = system_choice;   % was hardcoded to 17
    ode_params.t_final = 0.1;
    ode_params.timeStep = 10;

    states = 8;
    C = eye(states); C(7,7) = 0; C(end) = 0;

    map_mat = [0 0 0 0 1 0 0 0;
               1 0 0 -1 0 0 0 0;
               0 1 0 0 -1 0 0 0];
    U_fix = Star([30;1.4],[30;1.4]);

    %% 3. Initial Setup
    xlead = [90 110];
    v_lead = [32 32.2];
    x_ego = [10 11];
    v_ego = [30 30.2];

    lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2];
    ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2];

    % Queue stores cells of {lb, ub}
    queue = {};
    queue{end+1} = {lb_init, ub_init};

    final_result = 'SAFE';

    %% --- OPTIMIZATION START ---
    fprintf('\nInitializing Parallel Pool...\n');
    p = gcp('nocreate');
    if isempty(p)
        % Start pool utilizing physical cores
        p = parpool;
    end

    % Define Load Factor (The Tweak)
    % A factor of 3 to 5 is usually the "sweet spot" for uneven tasks.
    worker_load_factor = 5;   % Max 8 based on the processor cores on Dell OptiPlex 7071 tower

    % Set Batch Size
    batch_size = p.NumWorkers * worker_load_factor;

    fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
            p.NumWorkers, batch_size, worker_load_factor);
    % --- OPTIMIZATION END ---

    t_full = tic;
    iter = 1;

    %% 4. Main Parallel Refinement Loop (Batch Processing)
    while ~isempty(queue)

        if iter > MAX_ITER
            fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
            final_result = 'UNKNOWN (Wrapping effect dominated)';
            break;
        end

        % --- Extract strictly up to 'batch_size' tasks from the queue ---
        num_tasks = min(length(queue), batch_size);
        current_batch = queue(1:num_tasks);
        queue(1:num_tasks) = []; % Remove extracted elements from queue

        fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
                iter, num_tasks, length(queue));

        % Preallocate results array for parfor
        results = cell(1, num_tasks);

        % =========================================================
        % PARALLEL COMPUTATION BLOCK
        % =========================================================
        parfor idx = 1:num_tasks
            task_data = current_batch{idx};
            lb_curr = task_data{1};
            ub_curr = task_data{2};

            x_i = (lb_curr + ub_curr) / 2;
            r_i = (ub_curr - lb_curr) / 2;

            X0 = Star(lb_curr, ub_curr);
            is_subregion_safe = true;

            % Local copy of ode_params for the worker
            local_ode_params = ode_params;

            % Track trajectory locally
            trajR_local = X0;

            % --- N-Step Reachability Loop ---
            for k = 1:N
                % 1. Extract Interval Hull
                lb_all = zeros(states, length(X0));
                ub_all = zeros(states, length(X0));
                for s_idx = 1:length(X0)
                    B = X0(s_idx).getBox();
                    lb_all(:, s_idx) = B.lb;
                    ub_all(:, s_idx) = B.ub;
                end
                lb_hull = min(lb_all, [], 2);
                ub_hull = max(ub_all, [], 2);

                % 2. TNODEV Step
                local_ode_params.x0 = (lb_hull + ub_hull) / 2;
                local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;
                tnode_res = NODE_RA_New(local_ode_params);

                % 3. Check Safety
                X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);

                t_gap = 1.4; D_default = 10; alp = 1;
                rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
                safe_dist_star = X0_new.affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);

                % --- PROPER getBox() SYNTAX FOR SAFETY CHECK ---
                B_rel = rel_dist_star.getBox();
                min_rel_dist = B_rel.lb;

                B_safe = safe_dist_star.getBox();
                max_safe_dist = B_safe.ub;

                if min_rel_dist <= max_safe_dist
                    is_subregion_safe = false;
                    break; % Stop simulating, region is unsafe
                end

                % Add to trajectory
                trajR_local = [trajR_local X0_new];

                % 4. Controller Step & State Update (if safe)
                X0 = X0_new;
                ppp = X0.affineMap(map_mat,[]);
                Uin = U_fix.concatenate(ppp);
                Rc = net.reach(Uin);

                x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
                X0 = X0.affineMap(C(1:6,:),[]);
                X0 = X0.concatenate(Rc);
                X0 = X0.concatenate(x08);
            end

            % Map trajectory to Distance stars ONLY if it survived all N steps
            outAll_local = [];
            safe_dis_local = [];
            if is_subregion_safe
                for i=1:length(trajR_local)
                    rel_dist_star = trajR_local(i).affineMap([1 0 0 -1 0 0 0 0], []);
                    safe_dist_star = trajR_local(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
                    outAll_local = [outAll_local rel_dist_star];
                    safe_dis_local = [safe_dis_local safe_dist_star];
                end
            end

            % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
            split_dim = 1;
            if ~is_subregion_safe
                switch heuristic_choice
                    case 'naive'
                        [~, split_dim] = max(r_i);

                    case 'most-sensitive-input-radius'
                        try
                            [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, system_choice);
                            absJ = max(abs(J_lb), abs(J_ub));
                            sens = max(absJ, [], 1);
                            sens = max(sens, 1e-10);
                            scores = r_i(:)' .* sens;
                            [~, split_dim] = max(scores);
                        catch
                            [~, split_dim] = max(r_i);
                        end

                    case 'interval-norm-gradient'
                        try
                            % Get base norm for the initial box (1 step)
                            base_p = local_ode_params;
                            base_p.x0 = x_i; base_p.Initial_radius = r_i;
                            base_res = NODE_RA_New(base_p);
                            base_norm = sum(base_res.intersect_up - base_res.intersect_low);

                            grad = zeros(states, 1);
                            delta = 1e-6;
                            for j = 1:states
                                x_pert = x_i; x_pert(j) = x_pert(j) + delta;
                                p_pert = base_p; p_pert.x0 = x_pert;
                                p_res = NODE_RA_New(p_pert);
                                pert_norm = sum(p_res.intersect_up - p_res.intersect_low);
                                grad(j) = (pert_norm - base_norm) / delta;
                            end
                            heuristic = abs(grad);
                            [~, split_dim] = max(heuristic);
                        catch
                            [~, split_dim] = max(r_i);
                        end
                end
            end

            % Save worker results
            results{idx} = struct('is_safe', is_subregion_safe, 'split_dim', split_dim, 'lb', lb_curr, 'ub', ub_curr);
        end
        % =========================================================

        % --- Process Parallel Results & Rebuild Queue ---
        unsafe_count = 0;
        for idx = 1:num_tasks
            res = results{idx};
            if ~res.is_safe
                unsafe_count = unsafe_count + 1;
                split_dim = res.split_dim;
                lb_c = res.lb;
                ub_c = res.ub;

                mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;

                lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
                lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;

                % Add new splits to the queue
                queue{end+1} = {lb_1, ub_1};
                queue{end+1} = {lb_2, ub_2};
            end
        end

        fprintf('  -> Batch %d complete: %d safe, %d unsafe (spawned %d new sets).\n', ...
                iter, num_tasks - unsafe_count, unsafe_count, unsafe_count * 2);

        iter = iter + 1;
    end

    total_time = toc(t_full);

    %% Persist results to disk (before printing, so a print error can't lose them)
    try
        save_filename = sprintf('results_acc_sys%d_%s.mat', system_choice, heuristic_choice,datestr(now, 'yyyymmdd_HHMMSS'));
        save(save_filename, ...
             'total_time', 'final_result', 'iter', ...
             'heuristic_choice', 'system_choice', 'sys_name', ...
             'N', 'MAX_ITER', 'batch_size');
        fprintf('Results saved to: %s\n', save_filename);
    catch ME
        warning('reach_acc_TNODEV: failed to save results (%s)', ME.message);
    end

    %% Print Output Summaries
    fprintf('\n=================================================\n');
    fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
    fprintf('=================================================\n');
    fprintf('System:          %s (choice=%d)\n', sys_name, system_choice);
    fprintf('Heuristic Used:  %s\n', heuristic_choice);
    fprintf('Final Result:    %s\n', final_result);
    fprintf('Total Batches:   %d\n', iter - 1);
    fprintf('Full Time:       %.4f seconds\n', total_time);
    fprintf('=================================================\n\n');

end

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


