% % =========================================================================
% % File: reach_acc_TNODEV.m
% %
% % Purpose:
% %   End-to-end reachability analysis of the Adaptive Cruise Control (ACC)
% %   hybrid NNCS benchmark using the TNODEV pipeline, with parallel
% %   iterative refinement and three selectable splitting heuristics.
% %
% % Description:
% %   Runs an N-step NNCS reachability loop over the initial input set,
% %   driving the Neural ODE plant through NODE_RA_New (TIRA-based) and
% %   the neural-network controller through NNV's Star/reach API.
% %   Unsafe cells are bisected according to the chosen heuristic and
% %   re-scheduled for parallel re-verification.
% %
% %   Workers run under parfor with a configurable load factor; CORA/TIRA
% %   failures (reachable-set explosion on cells that are too large) are
% %   caught per-cell, marked unsafe, and let the refinement split them
% %   on the next batch.
% %
% %   Safety spec (rel_dist >= t_gap * v_ego + D_default):
% %     SPEC_T_GAP     = 1.4
% %     SPEC_D_DEFAULT = 10
% %     SPEC_ALPHA     = 1
% %
% % Inputs:
% %   heuristic_choice : 'naive'
% %                      'most-sensitive-input-radius'
% %                      'interval-norm-gradient'
% %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
% %                      24 = linear   3rd-order NODE (no activation)
% %                      (defaults to 17)
% %
% % Outputs (written to disk under ./results/acc_sys<ID>_<heuristic>_<timestamp>/):
% %   - results.mat    (verdict, timings, config)
% %   - distance.pdf   (vector figure, rel-dist vs safe-dist envelope)
% %   - distance.png   (raster figure)
% %
% % Prerequisites:
% %   - CORA          (reachable-tube bounds)
% %   - NNV 2.0       (Star set, controller reach)
% %   - controller_5_20.mat on the MATLAB path
% %
% % Part of the TNODEV Toolbox
% % Tool Paper: "Add Title here"
% %
% % Authors:
% %   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
% %   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
% %   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
% %   COSYS-ESTAS, Univ Gustave Eiffel
% %
% % License: CC BY-NC-SA 4.0
% % =========================================================================
% 
% %------------- BEGIN CODE --------------
% function reach_acc_TNODEV(heuristic_choice, system_choice)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %           + Robust CORA/TIRA explosion handling
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %                      (defaults to 17)
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
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, ...
%                                   'split_dim', split_dim, ...
%                                   'lb', lb_curr, 'ub', ub_curr, ...
%                                   'had_explosion', had_explosion);
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
%     %% 5. Plotting pass (single-pass from the original initial set)
%     fig_path = '';
%     try
%         fig_path = local_plot_single_pass(out_dir, ...
%             lb_init, ub_init, N, states, ode_params, ...
%             map_mat, U_fix, net, C, sys_name, final_result, ...
%             SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
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
%              'N', 'MAX_ITER', 'batch_size', 'fig_path', ...
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
% %  LOCAL HELPER: Single-pass plot from the original initial set
% %  Runs one non-refined NNCS simulation from lb_init..ub_init and plots
% %  the rel-dist vs safe-dist envelope as discrete per-step boxes.
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
%     % Extract distance bounds from the trajectory and render the figure
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
%     pdf_path = fullfile(out_dir, 'distance.pdf');
%     png_path = fullfile(out_dir, 'distance.png');
%     exportgraphics(f, pdf_path, 'ContentType', 'vector');
%     saveas(f, png_path);
%     close(f);
% 
%     fig_path = pdf_path;
%     fprintf('[plot] Saved: %s\n', pdf_path);
% end
% 
% %------------- END OF CODE --------------


%% Fixed the saving pattern and now saving in the Current_dir similar to the other examples


% % =========================================================================
% % File: reach_acc_TNODEV.m
% %
% % Purpose:
% %   End-to-end reachability analysis of the Adaptive Cruise Control (ACC)
% %   hybrid NNCS benchmark using the TNODEV pipeline, with parallel
% %   iterative refinement and three selectable splitting heuristics.
% %
% % Description:
% %   Runs an N-step NNCS reachability loop over the initial input set,
% %   driving the Neural ODE plant through NODE_RA_New (TIRA-based) and
% %   the neural-network controller through NNV's Star/reach API.
% %   Unsafe cells are bisected according to the chosen heuristic and
% %   re-scheduled for parallel re-verification.
% %
% %   Workers run under parfor with a configurable load factor; CORA/TIRA
% %   failures (reachable-set explosion on cells that are too large) are
% %   caught per-cell, marked unsafe, and let the refinement split them
% %   on the next batch.
% %
% %   Safety spec (rel_dist >= t_gap * v_ego + D_default):
% %     SPEC_T_GAP     = 1.4
% %     SPEC_D_DEFAULT = 10
% %     SPEC_ALPHA     = 1
% %
% % Inputs:
% %   heuristic_choice : 'naive'
% %                      'most-sensitive-input-radius'
% %                      'interval-norm-gradient'
% %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
% %                      24 = linear   3rd-order NODE (no activation)
% %                      (defaults to 17)
% %
% % Outputs (written to disk under ./results/acc_sys<ID>_<heuristic>_<timestamp>/):
% %   - results.mat    (verdict, timings, config)
% %   - distance.pdf   (vector figure, rel-dist vs safe-dist envelope)
% %   - distance.png   (raster figure)
% %
% % Prerequisites:
% %   - CORA          (reachable-tube bounds)
% %   - NNV 2.0       (Star set, controller reach)
% %   - controller_5_20.mat on the MATLAB path
% %
% % Part of the TNODEV Toolbox
% % Tool Paper: "Add Title here"
% %
% % Authors:
% %   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
% %   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
% %   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
% %   COSYS-ESTAS, Univ Gustave Eiffel
% %
% % License: CC BY-NC-SA 4.0
% % =========================================================================
% 
% %------------- BEGIN CODE --------------
% function reach_acc_TNODEV(heuristic_choice, system_choice)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %           + Robust CORA/TIRA explosion handling
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %                      (defaults to 17)
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
%     % Match the naming scheme used by the pure-NODE pipeline in
%     % Verifier_MAIN_CALL.m / Verifier_FINAL.m (FINAL_RESULTS_<sys>_<date>)
%     % so every example lands next to each other in the working directory.
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     run_name  = sprintf('FINAL_RESULTS_acc_sys%d_%s_%s', ...
%                         system_choice, heuristic_choice, timestamp);
%     out_dir   = fullfile(pwd, run_name);
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
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, ...
%                                   'split_dim', split_dim, ...
%                                   'lb', lb_curr, 'ub', ub_curr, ...
%                                   'had_explosion', had_explosion);
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
%     % Stop the verification timer here -- plotting and saving happen below
%     % and are NOT included in the verification time reported in the summary.
%     verification_time = toc(t_full);
% 
%     if total_explosions > 0
%         fprintf('\n[diagnostic] Total CORA/TIRA explosions across run: %d\n', total_explosions);
%     end
% 
%     %% 5. Plotting pass (single-pass from the original initial set)
%     fig_path = '';
%     try
%         fig_path = local_plot_single_pass(out_dir, ...
%             lb_init, ub_init, N, states, ode_params, ...
%             map_mat, U_fix, net, C, sys_name, final_result, ...
%             SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
%     catch ME
%         warning('reach_acc_TNODEV: plotting failed (%s)', ME.message);
%     end
% 
%     %% 6. Persist results to disk
%     try
%         save_filename = fullfile(out_dir, 'results.mat');
%         save(save_filename, ...
%              'verification_time', 'final_result', 'iter', ...
%              'heuristic_choice', 'system_choice', 'sys_name', ...
%              'N', 'MAX_ITER', 'batch_size', 'fig_path', ...
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
%     fprintf('Final Result:    %s\n', final_result);
%     fprintf('Total Batches:   %d\n', iter - 1);
%     fprintf('Explosions:      %d\n', total_explosions);
%     fprintf('Verification Time: %.4f seconds\n', verification_time);
%     fprintf('Output Folder:   %s\n', out_dir);
%     fprintf('=================================================\n\n');
% 
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Single-pass plot from the original initial set
% %  Runs one non-refined NNCS simulation from lb_init..ub_init and plots
% %  the rel-dist vs safe-dist envelope as discrete per-step boxes.
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
%     % Extract distance bounds from the trajectory and render the figure
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
%     pdf_path = fullfile(out_dir, 'distance.pdf');
%     png_path = fullfile(out_dir, 'distance.png');
%     exportgraphics(f, pdf_path, 'ContentType', 'vector');
%     saveas(f, png_path);
%     close(f);
% 
%     fig_path = pdf_path;
%     fprintf('[plot] Saved: %s\n', pdf_path);
% end
% 
% %------------- END OF CODE --------------

%% Added correct ING refinement heuristic

% =========================================================================
% File: reach_acc_TNODEV.m
%
% Purpose:
%   End-to-end reachability analysis of the Adaptive Cruise Control (ACC)
%   hybrid NNCS benchmark using the TNODEV pipeline, with parallel
%   iterative refinement and three selectable splitting heuristics.
%
% Description:
%   Runs an N-step NNCS reachability loop over the initial input set,
%   driving the Neural ODE plant through NODE_RA_New (TIRA-based) and
%   the neural-network controller through NNV's Star/reach API.
%   Unsafe cells are bisected according to the chosen heuristic and
%   re-scheduled for parallel re-verification.
%
%   Workers run under parfor with a configurable load factor; CORA/TIRA
%   failures (reachable-set explosion on cells that are too large) are
%   caught per-cell, marked unsafe, and let the refinement split them
%   on the next batch.
%
%   Safety spec (rel_dist >= t_gap * v_ego + D_default):
%     SPEC_T_GAP     = 1.4
%     SPEC_D_DEFAULT = 10
%     SPEC_ALPHA     = 1
%
% Inputs:
%   heuristic_choice : 'naive'
%                      'most-sensitive-input-radius'
%                      'interval-norm-gradient'
%   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%                      24 = linear   3rd-order NODE (no activation)
%                      (defaults to 17)
%
% Outputs (written to disk under ./results/acc_sys<ID>_<heuristic>_<timestamp>/):
%   - results.mat    (verdict, timings, config)
%   - distance.pdf   (vector figure, rel-dist vs safe-dist envelope)
%   - distance.png   (raster figure)
%
% Prerequisites:
%   - CORA          (reachable-tube bounds)
%   - NNV 2.0       (Star set, controller reach)
%   - controller_5_20.mat on the MATLAB path
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

% %------------- BEGIN CODE --------------
% function reach_acc_TNODEV(heuristic_choice, system_choice)
% 
%     % Reachability analysis of ACC using TNODEV with Iterative Refinement
%     % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
%     %           + Robust CORA/TIRA explosion handling
%     %
%     % Inputs:
%     %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
%     %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%     %                      24 = linear   3rd-order NODE (no activation)
%     %                      (defaults to 17)
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
%     N = 50;         % 50 steps total (5 seconds)
%     MAX_ITER = 5000; % Maximum number of batch iterations
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
%     % Match the naming scheme used by the pure-NODE pipeline in
%     % Verifier_MAIN_CALL.m / Verifier_FINAL.m (FINAL_RESULTS_<sys>_<date>)
%     % so every example lands next to each other in the working directory.
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     run_name  = sprintf('FINAL_RESULTS_acc_sys%d_%s_%s', ...
%                         system_choice, heuristic_choice, timestamp);
%     out_dir   = fullfile(pwd, run_name);
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
%                             % Element-wise score per input dimension i:
%                             %   s(i) = r(i) * max_k max(|J_lb(k,i)|, |J_ub(k,i)|)
%                             % matching the MSIR formula in the paper. Using
%                             % column-form on both sides avoids the outer-
%                             % product trap of `r_i .* sens` (column .* row).
%                             scores = r_i(:) .* sens(:);
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
%             % Save worker results
%             results{idx} = struct('is_safe', is_subregion_safe, ...
%                                   'split_dim', split_dim, ...
%                                   'lb', lb_curr, 'ub', ub_curr, ...
%                                   'had_explosion', had_explosion);
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
%     % Stop the verification timer here -- plotting and saving happen below
%     % and are NOT included in the verification time reported in the summary.
%     verification_time = toc(t_full);
% 
%     if total_explosions > 0
%         fprintf('\n[diagnostic] Total CORA/TIRA explosions across run: %d\n', total_explosions);
%     end
% 
%     %% 5. Plotting pass (single-pass from the original initial set)
%     fig_path = '';
%     try
%         fig_path = local_plot_single_pass(out_dir, ...
%             lb_init, ub_init, N, states, ode_params, ...
%             map_mat, U_fix, net, C, sys_name, final_result, ...
%             SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
%     catch ME
%         warning('reach_acc_TNODEV: plotting failed (%s)', ME.message);
%     end
% 
%     %% 6. Persist results to disk
%     try
%         save_filename = fullfile(out_dir, 'results.mat');
%         save(save_filename, ...
%              'verification_time', 'final_result', 'iter', ...
%              'heuristic_choice', 'system_choice', 'sys_name', ...
%              'N', 'MAX_ITER', 'batch_size', 'fig_path', ...
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
%     fprintf('Final Result:    %s\n', final_result);
%     fprintf('Total Batches:   %d\n', iter - 1);
%     fprintf('Explosions:      %d\n', total_explosions);
%     fprintf('Verification Time: %.4f seconds\n', verification_time);
%     fprintf('Output Folder:   %s\n', out_dir);
%     fprintf('=================================================\n\n');
% 
% end
% 
% 
% %% =====================================================================
% %  LOCAL HELPER: Single-pass plot from the original initial set
% %  Runs one non-refined NNCS simulation from lb_init..ub_init and plots
% %  the rel-dist vs safe-dist envelope as discrete per-step boxes.
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
%     % Extract distance bounds from the trajectory and render the figure
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
%     pdf_path = fullfile(out_dir, 'distance.pdf');
%     png_path = fullfile(out_dir, 'distance.png');
%     exportgraphics(f, pdf_path, 'ContentType', 'vector');
%     saveas(f, png_path);
%     close(f);
% 
%     fig_path = pdf_path;
%     fprintf('[plot] Saved: %s\n', pdf_path);
% end
% 
% %------------- END OF CODE --------------

%% Added correct ING refinement heuristic with correct prints that differentiates between no. of batches and cells

% =========================================================================
% File: reach_acc_TNODEV.m
%
% Purpose:
%   End-to-end reachability analysis of the Adaptive Cruise Control (ACC)
%   hybrid NNCS benchmark using the TNODEV pipeline, with parallel
%   iterative refinement and three selectable splitting heuristics.
%
% Description:
%   Runs an N-step NNCS reachability loop over the initial input set,
%   driving the Neural ODE plant through NODE_RA_New (TIRA-based) and
%   the neural-network controller through NNV's Star/reach API.
%   Unsafe cells are bisected according to the chosen heuristic and
%   re-scheduled for parallel re-verification.
%
%   Workers run under parfor with a configurable load factor; CORA/TIRA
%   failures (reachable-set explosion on cells that are too large) are
%   caught per-cell, marked unsafe, and let the refinement split them
%   on the next batch.
%
%   Safety spec (rel_dist >= t_gap * v_ego + D_default):
%     SPEC_T_GAP     = 1.4
%     SPEC_D_DEFAULT = 10
%     SPEC_ALPHA     = 1
%
% Inputs:
%   heuristic_choice : 'naive'
%                      'most-sensitive-input-radius'
%                      'interval-norm-gradient'
%   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
%                      24 = linear   3rd-order NODE (no activation)
%                      (defaults to 17)
%
% Outputs (written to disk under ./results/acc_sys<ID>_<heuristic>_<timestamp>/):
%   - results.mat    (verdict, timings, config)
%   - distance.pdf   (vector figure, rel-dist vs safe-dist envelope)
%   - distance.png   (raster figure)
%
% Prerequisites:
%   - CORA          (reachable-tube bounds)
%   - NNV 2.0       (Star set, controller reach)
%   - controller_5_20.mat on the MATLAB path
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
function reach_acc_TNODEV(heuristic_choice, system_choice)

    % Reachability analysis of ACC using TNODEV with Iterative Refinement
    % Features: Parfor Parallelization (Optimized) + 3 Splitting Heuristics
    %           + Robust CORA/TIRA explosion handling
    %
    % Inputs:
    %   heuristic_choice : 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'
    %   system_choice    : 17 = nonlinear 3rd-order NODE (tanh activation)
    %                      24 = linear   3rd-order NODE (no activation)
    %                      (defaults to 17)
    %
    % Outputs are saved under:
    %   ./results/acc_sys<ID>_<heuristic>_<timestamp>/
    %     - results.mat      (verdict, timings, config)
    %     - distance.pdf     (vector figure)
    %     - distance.png     (raster figure)

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

    N = 50;         % 50 steps total (5 seconds)
    MAX_ITER = 5000; % Maximum number of batch iterations
    TIMEOUT_SECONDS = 120 * 60; % Wall-clock timeout T_max for the verification & refinement loop ONLY (does not include the CORA / Jacobian / falsification preamble). The loop stops once EITHER MAX_ITER or TIMEOUT_SECONDS is reached, whichever comes first; a non-empty queue at that point yields an UNKNOWN verdict. Set to `Inf` to disable the timeout and recover budget-only behavior.
    
    % --- ADD THESE THREE COUNTERS HERE ---
    total_refinements = 0;     % total number of cells that got split
    verified_subsets = 0;      % total number of cells that ended SAFE
    max_queue_size = 1;        % deepest the queue ever got
    % -------------------------------------

    % Safety spec (used by both refinement check and plotting)
    SPEC_T_GAP     = 1.4;   % Original was 1.4
    SPEC_D_DEFAULT = 10;    % Original was 10 | 20 "works" | 
    SPEC_ALPHA     = 1;

    %% 2. Load objects
    net = load_NN_from_mat('controller_5_20.mat');

    ode_params = struct();
    ode_params.system_choice = system_choice;
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
    % v_lead = [32 32.2]; % original
    v_lead = [31 33];
    x_ego = [10 11];
    % v_ego = [30 30.2]; % original
    v_ego = [29 31];

    lb_init = [xlead(1); v_lead(1); 0; x_ego(1); v_ego(1); 0; 0; -2];
    ub_init = [xlead(2); v_lead(2); 0; x_ego(2); v_ego(2); 0; 0; -2];

    % Queue stores cells of {lb, ub}
    queue = {};
    queue{end+1} = {lb_init, ub_init};

    final_result = 'SAFE';

    %% 3b. Prepare output folder (unique per run via timestamp)
    % Match the naming scheme used by the pure-NODE pipeline in
    % Verifier_MAIN_CALL.m / Verifier_FINAL.m (FINAL_RESULTS_<sys>_<date>)
    % so every example lands next to each other in the working directory.
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    run_name  = sprintf('FINAL_RESULTS_acc_sys%d_%s_%s', ...
                        system_choice, heuristic_choice, timestamp);
    out_dir   = fullfile(pwd, run_name);
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    fprintf('Output folder: %s\n', out_dir);

    %% --- OPTIMIZATION START ---
    fprintf('\nInitializing Parallel Pool...\n');
    p = gcp('nocreate');
    if isempty(p)
        p = parpool;
    end

    worker_load_factor = 5;
    batch_size = p.NumWorkers * worker_load_factor;

    fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
            p.NumWorkers, batch_size, worker_load_factor);
    % --- OPTIMIZATION END ---

    t_full = tic;
    iter = 1;

    %% 4. Main Parallel Refinement Loop (Batch Processing)

    % Track CORA/TIRA explosion count for diagnostics
    total_explosions = 0;

    while ~isempty(queue)

        if iter > MAX_ITER
            fprintf('\n[!] MAX_ITER (%d) reached. Stopping refinement.\n', MAX_ITER);
            final_result = 'UNKNOWN (Wrapping effect dominated)';
            break;
        end

        if toc(t_full) >= TIMEOUT_SECONDS
            fprintf('\n[!] TIMEOUT (T_max = %.0f min) reached after %d batches. Stopping refinement.\n', ...
                    TIMEOUT_SECONDS/60, iter - 1);
            final_result = sprintf('UNKNOWN (Timeout: T_max = %.0f min, %d sub-regions remain in queue).', ...
                                   TIMEOUT_SECONDS/60, length(queue));
            break;
        end

        num_tasks = min(length(queue), batch_size);
        current_batch = queue(1:num_tasks);
        queue(1:num_tasks) = [];

        fprintf('\n--- Batch Iteration %d: Testing %d parallel sub-regions (Queue remaining: %d) ---\n', ...
                iter, num_tasks, length(queue));

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
            had_explosion = false;   % track if this cell triggered a CORA/TIRA failure

            local_ode_params = ode_params;

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

                local_ode_params.x0 = (lb_hull + ub_hull) / 2;
                local_ode_params.Initial_radius = (ub_hull - lb_hull) / 2;

                % ==========================================================
                %  TRY-CATCH BLOCK to handle CORA reachable-set explosion
                %  or TIRA failure on cells that are too large.
                %  When either occurs, mark the cell as unsafe and break so
                %  the heuristic split logic will bisect it on the next batch.
                % ==========================================================
                try
                    % 2. TNODEV Step
                    tnode_res = NODE_RA_New(local_ode_params);

                    % 3. Check Safety
                    X0_new = Star(tnode_res.intersect_low, tnode_res.intersect_up);

                    rel_dist_star = X0_new.affineMap([1 0 0 -1 0 0 0 0], []);
                    safe_dist_star = X0_new.affineMap( ...
                        [0 0 0 0 SPEC_ALPHA*SPEC_T_GAP 0 0 0], SPEC_ALPHA*SPEC_D_DEFAULT);

                    B_rel = rel_dist_star.getBox();
                    min_rel_dist = B_rel.lb;

                    B_safe = safe_dist_star.getBox();
                    max_safe_dist = B_safe.ub;

                    if min_rel_dist <= max_safe_dist
                        is_subregion_safe = false;
                        break; % Stop simulating, region is unsafe
                    end

                catch ME %#ok<NASGU>
                    % CORA reachable-set explosion or TIRA failure.
                    % Cannot prove safety for this cell at its current size.
                    % Mark as unsafe so refinement splits it next iteration.
                    is_subregion_safe = false;
                    had_explosion = true;
                    break;
                end
                % ==========================================================

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

            % --- HEURISTIC SPLIT LOGIC (Runs only if unsafe) ---
            split_dim = 1;
            if ~is_subregion_safe
                switch heuristic_choice
                    case 'naive'
                        [~, split_dim] = max(r_i);

                    case 'most-sensitive-input-radius'
                        % MSIR itself could fail on exploded cells
                        % (Jacobian bounding calls CORA internally for case 17).
                        % Fall back to naive on any failure.
                        try
                            [J_lb, J_ub] = Jacob_bounds_element_wise(lb_curr, ub_curr, system_choice);
                            absJ = max(abs(J_lb), abs(J_ub));
                            sens = max(absJ, [], 1);
                            sens = max(sens, 1e-10);
                            % Element-wise score per input dimension i:
                            %   s(i) = r(i) * max_k max(|J_lb(k,i)|, |J_ub(k,i)|)
                            % matching the MSIR formula in the paper. Using
                            % column-form on both sides avoids the outer-
                            % product trap of `r_i .* sens` (column .* row).
                            scores = r_i(:) .* sens(:);
                            [~, split_dim] = max(scores);
                        catch
                            [~, split_dim] = max(r_i);
                        end

                    case 'interval-norm-gradient'
                        % ING makes n extra NODE_RA_New calls per decision.
                        % Any one can explode. Fall back to naive on failure.
                        try
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
            results{idx} = struct('is_safe', is_subregion_safe, ...
                                  'split_dim', split_dim, ...
                                  'lb', lb_curr, 'ub', ub_curr, ...
                                  'had_explosion', had_explosion);
        end
        % =========================================================

        % --- Process Parallel Results & Rebuild Queue ---
        unsafe_count = 0;
        explosion_count = 0;
        for idx = 1:num_tasks
            res = results{idx};
            if res.had_explosion
                explosion_count = explosion_count + 1;
            end
            if ~res.is_safe
                unsafe_count = unsafe_count + 1;
                split_dim = res.split_dim;
                lb_c = res.lb;
                ub_c = res.ub;

                mid_point = (lb_c(split_dim) + ub_c(split_dim)) / 2;

                lb_1 = lb_c; ub_1 = ub_c; ub_1(split_dim) = mid_point;
                lb_2 = lb_c; ub_2 = ub_c; lb_2(split_dim) = mid_point;

                queue{end+1} = {lb_1, ub_1};
                queue{end+1} = {lb_2, ub_2};

            else
                verified_subsets = verified_subsets + 1;   % <-- ADDED 12/05
            end
        end
        total_refinements = total_refinements + unsafe_count;   % <-- ADDED 12/05
        max_queue_size = max(max_queue_size, length(queue));     % <-- ADDED 12/05
        total_explosions = total_explosions + explosion_count;

        fprintf('  -> Batch %d complete: %d safe, %d unsafe (%d CORA explosions, spawned %d new sets).\n', ...
                iter, num_tasks - unsafe_count, unsafe_count, explosion_count, unsafe_count * 2);

        % Early exit: if every single cell exploded AND the queue is not
        % shrinking, we're in a degenerate regime where sub-cells won't
        % get small enough to help. Warn but keep going up to MAX_ITER.
        if explosion_count == num_tasks && num_tasks >= batch_size
            fprintf('  [!] All %d cells in this batch exploded. Refinement may not converge.\n', num_tasks);
        end

        iter = iter + 1;
    end

    % Stop the verification timer here -- plotting and saving happen below
    % and are NOT included in the verification time reported in the summary.
    verification_time = toc(t_full);

    % --- ADD THIS LINE to match the table and get the number of total cells processed ---
    total_cells = total_refinements + verified_subsets;
    % ---------------------

    if total_explosions > 0
        fprintf('\n[diagnostic] Total CORA/TIRA explosions across run: %d\n', total_explosions);
    end

    %% 5. Plotting pass (single-pass from the original initial set)
    fig_path = '';
    try
        fig_path = local_plot_single_pass(out_dir, ...
            lb_init, ub_init, N, states, ode_params, ...
            map_mat, U_fix, net, C, sys_name, final_result, ...
            SPEC_T_GAP, SPEC_D_DEFAULT, SPEC_ALPHA);
    catch ME
        warning('reach_acc_TNODEV: plotting failed (%s)', ME.message);
    end

    %% 6. Persist results to disk
    try
        save_filename = fullfile(out_dir, 'results.mat');
        save(save_filename, ...
             'verification_time', 'final_result', 'iter', ...
             'total_cells', ...                                    % <-- ADDED on 12/05
             'total_refinements', 'verified_subsets', ...           % <-- ADDED on 12/05
             'max_queue_size', ...                                  % <-- ADDED on 12/05
             'heuristic_choice', 'system_choice', 'sys_name', ...
             'N', 'MAX_ITER', 'TIMEOUT_SECONDS', 'batch_size', 'fig_path', ...
             'total_explosions', ...
             'SPEC_T_GAP', 'SPEC_D_DEFAULT', 'SPEC_ALPHA');
        fprintf('Results saved to: %s\n', save_filename);
    catch ME
        warning('reach_acc_TNODEV: failed to save results (%s)', ME.message);
    end

    %% 7. Print Output Summaries
    fprintf('\n=================================================\n');
    fprintf('VERIFICATION SUMMARY (PARALLEL REFINEMENT)\n');
    fprintf('=================================================\n');
    fprintf('System:          %s (choice=%d)\n', sys_name, system_choice);
    fprintf('Heuristic Used:  %s\n', heuristic_choice);
    fprintf('Final Result:    %s\n', final_result);
    fprintf('Total cells processed:   %d\n', total_cells);        % <-- ADDED on 12/05
    fprintf('Total Batches:   %d\n', iter - 1);                 % keep for diagnostic
    fprintf('Refinement splits:   %d\n', total_refinements);    % <-- ADDED 12/05
    fprintf('Verified subsets:    %d\n', verified_subsets);     % <-- ADDED 12/05
    fprintf('Max queue size:      %d\n', max_queue_size);       % <-- ADDED 12/05
    fprintf('Explosions:      %d\n', total_explosions);
    fprintf('Verification Time: %.4f seconds\n', verification_time);
    fprintf('Output Folder:   %s\n', out_dir);
    fprintf('=================================================\n\n');

end


%% =====================================================================
%  LOCAL HELPER: Single-pass plot from the original initial set
%  Runs one non-refined NNCS simulation from lb_init..ub_init and plots
%  the rel-dist vs safe-dist envelope as discrete per-step boxes.
% ======================================================================
function fig_path = local_plot_single_pass(out_dir, lb_init, ub_init, N, states, ...
    ode_params, map_mat, U_fix, net, C, sys_name, final_result, ...
    t_gap, D_default, alp)

    fprintf('\n[plot] Running single-pass visualization trajectory...\n');
    X0 = Star(lb_init, ub_init);
    trajR = X0;

    local_ode = ode_params;

    for k = 1:N
        lb_all = zeros(states, length(X0));
        ub_all = zeros(states, length(X0));
        for s_idx = 1:length(X0)
            B = X0(s_idx).getBox();
            lb_all(:, s_idx) = B.lb;
            ub_all(:, s_idx) = B.ub;
        end
        lb_hull = min(lb_all, [], 2);
        ub_hull = max(ub_all, [], 2);

        local_ode.x0 = (lb_hull + ub_hull) / 2;
        local_ode.Initial_radius = (ub_hull - lb_hull) / 2;

        % Single-pass plot is best-effort. If CORA explodes at the top
        % level, we print a warning and abort the plot gracefully.
        try
            tnode_res = NODE_RA_New(local_ode);
        catch ME
            warning(['local_plot_single_pass: NODE_RA_New failed at step %d (%s). ' ...
                     'Single-pass plot will be truncated.'], k, ME.message);
            break;
        end

        X0 = Star(tnode_res.intersect_low, tnode_res.intersect_up);
        trajR = [trajR X0];

        ppp = X0.affineMap(map_mat, []);
        Uin = U_fix.concatenate(ppp);
        Rc = net.reach(Uin);

        x08 = X0.affineMap([0 0 0 0 0 0 0 1], []);
        X0 = X0.affineMap(C(1:6, :), []);
        X0 = X0.concatenate(Rc);
        X0 = X0.concatenate(x08);
    end

    % Extract distance bounds from the trajectory and render the figure
    M = length(trajR);
    rel_dist_min  = zeros(1, M);
    rel_dist_max  = zeros(1, M);
    safe_dist_min = zeros(1, M);
    safe_dist_max = zeros(1, M);

    for i = 1:M
        rel_star  = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
        safe_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
        B_rel  = rel_star.getBox();
        B_safe = safe_star.getBox();
        rel_dist_min(i)  = B_rel.lb;
        rel_dist_max(i)  = B_rel.ub;
        safe_dist_min(i) = B_safe.lb;
        safe_dist_max(i) = B_safe.ub;
    end

    times = 0:0.1:0.1*N;
    L = min(length(times)-1, length(rel_dist_min));

    f = figure('Color', 'w', 'Visible', 'off'); hold on;
    for i = 1:L
        t_start = times(i);
        t_width = times(i+1) - times(i);

        y_safe_min = safe_dist_min(i);
        y_safe_max = safe_dist_max(i);
        if isfinite(y_safe_min) && isfinite(y_safe_max) && y_safe_max > y_safe_min
            rectangle('Position', [t_start, y_safe_min, t_width, y_safe_max - y_safe_min], ...
                      'EdgeColor', 'm', 'LineWidth', 0.5);
        end

        y_rel_min = rel_dist_min(i);
        y_rel_max = rel_dist_max(i);
        if isfinite(y_rel_min) && isfinite(y_rel_max) && y_rel_max > y_rel_min
            rectangle('Position', [t_start, y_rel_min, t_width, y_rel_max - y_rel_min], ...
                      'EdgeColor', 'k', 'LineWidth', 0.5);
        end
    end

    pr = plot(NaN, NaN, 'k', 'LineWidth', 1.5);
    pb = plot(NaN, NaN, 'm', 'LineWidth', 1.5);

    ax = gca; ax.XAxis.FontSize = 15; ax.YAxis.FontSize = 15;
    xlabel('Time (s)',     'FontSize', 16, 'FontWeight', 'bold');
    ylabel('Distance (m)', 'FontSize', 16, 'FontWeight', 'bold');
    ylim([40 110]);

    title(sprintf('%s — verdict: %s', sys_name, final_result), ...
          'Interpreter', 'none', 'FontSize', 14);
    legend([pr, pb], {'rel dist', 'safe dist'}, 'Location', 'best', 'FontSize', 14);

    pdf_path = fullfile(out_dir, 'distance.pdf');
    png_path = fullfile(out_dir, 'distance.png');
    exportgraphics(f, pdf_path, 'ContentType', 'vector');
    saveas(f, png_path);
    close(f);

    fig_path = pdf_path;
    fprintf('[plot] Saved: %s\n', pdf_path);
end

%------------- END OF CODE --------------
