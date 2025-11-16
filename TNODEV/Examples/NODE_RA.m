% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 22 October 2025
% Last update: 14 November 2025
% Last revision: October 2025

% %------------- BEGIN CODE --------------
% 
% 
% function reachability_results = NODE_RA(params)
% % NODE_RA - Reachability analysis for neural ODEs using TIRA
% % Input:
% %   params - struct with fields:
% %       .system_choice      : 14 or 15
% %       .x0                 : initial center
% %       .Initial_radius     : radius of initial set
% %       .t_final            : final time
% %       .timeStep           : discretization step
% %       .plot_last_only     : true/false
% %       .sample_succ_number : number of random samples
% % Output:
% %   reachability_results - struct containing all reachable sets, plots, etc.
% 
% %% Initialization
% close all
% bool_discrete_time = 0;
% addpath('../OA_methods')
% addpath('../Utilities')    
% 
% %% User-defined parameters (override defaults with input)
% plot_last_only = true;
% if isfield(params, 'plot_last_only'), plot_last_only = params.plot_last_only; end
% 
% sample_succ_number = 1000;
% if isfield(params, 'sample_succ_number'), sample_succ_number = params.sample_succ_number; end
% 
% %% Choice of the example system
% global system_choice
% system_choice = params.system_choice;
% 
% % Map system choice to system name
% system_names = containers.Map({14, 15}, {'fpa', 'spiral_non'});
% if ~isKey(system_names, system_choice)
%     error('Unknown system_choice %d.', system_choice);
% end
% system_name = system_names(system_choice);
% 
% %% Optional submethod choice
% global g_sensitivity_bounds_method
% g_sensitivity_bounds_method = nan;
% 
% %% Definition of the reachability problem
% global u
% switch system_choice
%     case 14
%         Initial_radius = 0.01;
%         x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
%         t_final = 2;
%         steps = 0.05;
%     case 15
%         Initial_radius = params.Initial_radius;
%         x0 = params.x0;
%         t_final = params.t_final;
%         steps = params.timeStep;
% end
% 
% x_low = x0 - Initial_radius;
% x_up  = x0 + Initial_radius;
% t_init = 0;
% t_points = t_init:steps:t_final;
% 
% p_low = zeros(length(x_low),1); 
% p_up  = zeros(length(x_low),1);
% 
% % Dimensions
% n_x = length(x_low);
% n_p = length(p_low);
% n_t = length(t_points);
% 
% % Verify dimensions
% if system_choice == 14 && (n_x ~= 5 || n_p ~= 5)
%     error('FPA expects 5D state/parameter.');
% elseif system_choice == 15 && (n_x ~= 2 || n_p ~= 2)
%     error('Spiral_non expects 2D state/parameter.');
% end
% 
% % Define methods
% methods = [3, 4]; 
% n_methods = length(methods);
% succ_low_all_methods = NaN(n_x, n_t, n_methods);
% succ_up_all_methods  = NaN(n_x, n_t, n_methods);
% 
% %% Boundary splits
% num_splits = 2*n_x;
% Initial_set_boundaries = cell(num_splits, 1);
% succ_low_boundary = cell(num_splits, 1);
% succ_up_boundary  = cell(num_splits, 1);
% boundary_times = zeros(num_splits, n_methods);
% 
% for i = 1:n_x
%     temp_low = x_low; temp_up = x_up; temp_low(i) = x_up(i);
%     Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
%     temp_low = x_low; temp_up = x_up; temp_up(i) = x_low(i);
%     Initial_set_boundaries{2*i} = interval(temp_low(:), temp_up(:));
% end
% 
% %% Main reachability computation
% try
%     time_initial = zeros(1, n_methods);
%     time_boundary = zeros(num_splits, n_methods);
% 
%     % Single-step (initial approach)
%     for m = 1:n_methods
%         tStart = tic;
%         [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
%         time_initial(m) = toc(tStart);
%         succ_low_all_methods(:, :, m) = repmat(succ_low, 1, n_t);
%         succ_up_all_methods(:, :, m)  = repmat(succ_up,  1, n_t);
%     end
% 
%     % Boundary approach
%     for i = 1:num_splits
%         succ_low_boundary{i} = NaN(n_x, n_t, n_methods);
%         succ_up_boundary{i}  = NaN(n_x, n_t, n_methods);
%         for m = 1:n_methods
%             if plot_last_only
%                 tStart = tic;
%                 [succ_low, succ_up] = TIRA([t_init, t_final], ...
%                     Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
%                 time_boundary(i, m) = toc(tStart);
%                 succ_low_boundary{i}(:, end, m) = succ_low;
%                 succ_up_boundary{i}(:, end, m)  = succ_up;
%             else
%                 for k = 1:n_t
%                     t_final_k = t_points(k);
%                     tStart = tic;
%                     [succ_low, succ_up] = TIRA([t_init, t_final_k], ...
%                         Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
%                     time_boundary(i, m) = time_boundary(i, m) + toc(tStart);
%                     succ_low_boundary{i}(:, k, m) = succ_low;
%                     succ_up_boundary{i}(:, k, m)  = succ_up;
%                 end
%             end
%         end
%     end
% catch e
%     rethrow(e);
% end
% 
% %% Random successors
% try
%     run('Solver_parameters.m');
%     ode_solver = parameters.ode_solver;
%     ode_options = parameters.ode_options;
% 
%     rand_succ = NaN(n_x, sample_succ_number);
%     for i = 1:sample_succ_number
%         x0_rand = x_low + rand(n_x,1).*(x_up - x_low);
%         p_rand  = p_low + rand(n_p,1).*(p_up - p_low);
%         [~, x_traj] = ode_solver(@(t,x) System_description(t,x,p_rand), [t_init t_final], x0_rand, ode_options);
%         rand_succ(:,i) = x_traj(end,:)';
%     end
% catch e
%     rethrow(e);
% end
% 
% 
% %% Plot the reachable set and over-approximations
% try
%     % Create timestamped directory for all outputs
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     save_dir = fullfile(pwd, sprintf('results_%s_%s', system_name, timestamp));
%     if ~exist(save_dir, 'dir')
%         [status, msg] = mkdir(save_dir);
%         if status ~= 1
%             warning('Failed to create directory %s: %s', save_dir, msg);
%         else
%             fprintf('Created directory: %s\n', save_dir);
%         end
%     end
% 
%     % Define plot pairs based on system choice
%     plot_pairs = [];
%     if system_choice == 14
%         plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
%     elseif system_choice == 15
%         plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
%     end
%     colors = [0 0 1; 1 0 0]; % Dark Blue, Red
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     for p = 1:size(plot_pairs, 1)
%         figure('Name', sprintf('Reachability Plot - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
%         hold on;
%         grid on;
%         idx1 = plot_pairs(p, 1);
%         idx2 = plot_pairs(p, 2);
% 
%         % Plot the successors from random initial states
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Calculate minimum bounding box for black dots
%         x_min_actual = min(rand_succ(idx1, :));
%         x_max_actual = max(rand_succ(idx1, :));
%         y_min_actual = min(rand_succ(idx2, :));
%         y_max_actual = max(rand_succ(idx2, :));
%         area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));
% 
%         % Plot over-approximation rectangles
%         handles = [];
%         proxy_handles = [];
%         tightness_metrics = zeros(1, 4); % For 4 approaches (Single-step and Boundary only)
% 
%         if plot_last_only
%             k = size(succ_low_all_methods, 2); % Last time step
% 
%             % 1: Mixed-Monotonicity (Boundary) - Dashed Magenta
%             all_x_min_bound = [];
%             all_x_max_bound = [];
%             all_y_min_bound = [];
%             all_y_max_bound = [];
%             for i = 1:num_splits
%                 % fprintf('Boundary data m=1, split %d: low=[%f %f], up=[%f %f]\n', i, ...
%                 %         succ_low_boundary{i}(idx1, k, 1), succ_low_boundary{i}(idx2, k, 1), ...
%                 %         succ_up_boundary{i}(idx1, k, 1), succ_up_boundary{i}(idx2, k, 1));
%                 all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 1)];
%                 all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 1)];
%                 all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 1)];
%                 all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 1)];
%             end
%             x_min_bound = min(all_x_min_bound);
%             x_max_bound = max(all_x_max_bound);
%             y_min_bound = min(all_y_min_bound);
%             y_max_bound = max(all_y_max_bound);
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Mixed-Monotonicity (Boundary) computed bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 h_bound1 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
%                                     'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
%                 h_proxy_bound1 = plot(NaN, NaN, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_bound1];
%                 proxy_handles = [proxy_handles, h_proxy_bound1];
%                 tightness_metrics(1) = (width_bound * height_bound) / area_actual;
%             else
%                 warning('Mixed-Monotonicity (Boundary) has invalid bounds. Using actual range.');
%                 h_bound1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
%                                    'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1, 'LineStyle', ':');
%                 h_proxy_bound1 = plot(NaN, NaN, 'm:', 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_bound1];
%                 proxy_handles = [proxy_handles, h_proxy_bound1];
%                 tightness_metrics(1) = 1.0;
%             end
% 
%             % 2: Sampled-Data Mixed-Monotonicity (Boundary) - Dashed Green
%             all_x_min_bound = [];
%             all_x_max_bound = [];
%             all_y_min_bound = [];
%             all_y_max_bound = [];
%             for i = 1:num_splits
%                 % fprintf('Boundary data m=2, split %d: low=[%f %f], up=[%f %f]\n', i, ...
%                 %         succ_low_boundary{i}(idx1, k, 2), succ_low_boundary{i}(idx2, k, 2), ...
%                 %         succ_up_boundary{i}(idx1, k, 2), succ_up_boundary{i}(idx2, k, 2));
%                 all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 2)];
%                 all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 2)];
%                 all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 2)];
%                 all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 2)];
%             end
%             x_min_bound = min(all_x_min_bound);
%             x_max_bound = max(all_x_max_bound);
%             y_min_bound = min(all_y_min_bound);
%             y_max_bound = max(all_y_max_bound);
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Sampled-Data Mixed-Monotonicity (Boundary) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 h_bound2 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
%                                     'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
%                 h_proxy_bound2 = plot(NaN, NaN, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_bound2];
%                 proxy_handles = [proxy_handles, h_proxy_bound2];
%                 tightness_metrics(2) = (width_bound * height_bound) / area_actual;
%             else
%                 warning('Skipping Sampled-Data Mixed-Monotonicity (Boundary) due to invalid parameters');
%             end
% 
%             % 3: Mixed-Monotonicity (Single-step) - Dark Blue Solid
%             x_min_single = succ_low_all_methods(idx1, k, 1); % Method 1
%             y_min_single = succ_low_all_methods(idx2, k, 1);
%             width_single = succ_up_all_methods(idx1, k, 1) - x_min_single;
%             height_single = succ_up_all_methods(idx2, k, 1) - y_min_single;
%             fprintf('Mixed-Monotonicity (Single-step) computed bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     x_min_single, y_min_single, width_single, height_single);
%             if all(isfinite([x_min_single, y_min_single, width_single, height_single])) && width_single >= 0 && height_single >= 0
%                 h_single1 = rectangle('Position', [x_min_single, y_min_single, max(width_single, 0.001), max(height_single, 0.001)], ...
%                                     'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
%                 h_proxy_single1 = plot(NaN, NaN, '-', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_single1];
%                 proxy_handles = [proxy_handles, h_proxy_single1];
%                 tightness_metrics(3) = (width_single * height_single) / area_actual;
%             else
%                 warning('Mixed-Monotonicity (Single-step) has invalid bounds. Using actual range.');
%                 h_single1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
%                                    'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1, 'LineStyle', ':');
%                 h_proxy_single1 = plot(NaN, NaN, ':', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_single1];
%                 proxy_handles = [proxy_handles, h_proxy_single1];
%                 tightness_metrics(3) = 1.0;
%             end
% 
%             % 4: Sampled-Data Mixed-Monotonicity (Single-step) - Red Solid
%             x_min_single = succ_low_all_methods(idx1, k, 2); % Method 2
%             y_min_single = succ_low_all_methods(idx2, k, 2);
%             width_single = succ_up_all_methods(idx1, k, 2) - x_min_single;
%             height_single = succ_up_all_methods(idx2, k, 2) - y_min_single;
%             fprintf('Sampled-Data Mixed-Monotonicity (Single-step) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     x_min_single, y_min_single, width_single, height_single);
%             if all(isfinite([x_min_single, y_min_single, width_single, height_single])) && width_single >= 0 && height_single >= 0
%                 h_single2 = rectangle('Position', [x_min_single, y_min_single, max(width_single, 0.001), max(height_single, 0.001)], ...
%                                     'EdgeColor', colors(2,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
%                 h_proxy_single2 = plot(NaN, NaN, '-', 'Color', colors(2,:), 'LineWidth', 1.5, 'Visible', 'off');
%                 handles = [handles, h_single2];
%                 proxy_handles = [proxy_handles, h_proxy_single2];
%                 tightness_metrics(4) = (width_single * height_single) / area_actual;
%             else
%                 warning('Skipping Sampled-Data Mixed-Monotonicity (Single-step) due to invalid parameters');
%             end
%         end
% 
%         % Adjust axis to include all bounds
%         x_min_all = min([x_min_actual, x_min_bound, x_min_single]);
%         x_max_all = max([x_max_actual, x_max_bound + width_bound, x_min_single + width_single]);
%         y_min_all = min([y_min_actual, y_min_bound, y_min_single]);
%         y_max_all = max([y_max_actual, y_max_bound + height_bound, y_min_single + height_single]);
%         x_range = x_max_all - x_min_all;
%         y_range = y_max_all - y_min_all;
%         buffer = 0.3; % 10% padding
%         axis([x_min_all - buffer*x_range, x_max_all + buffer*x_range, y_min_all - buffer*y_range, y_max_all + buffer*y_range]);
% 
%         % Add labels
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
% 
% 
%         % <<< SAFE-SET: Green Thick Rectangle (drawn with plot to support legend) >>>
%         if isfield(params,'safe_inf') && isfield(params,'safe_sup')
%             safe_inf = params.safe_inf;
%             safe_sup = params.safe_sup;
% 
%             % === Only draw if valid 2D bounds ===
%             if numel(safe_inf) == 2 && numel(safe_sup) == 2 && ...
%                all(isfinite(safe_inf)) && all(isfinite(safe_sup)) && ...
%                safe_sup(1) > safe_inf(1) && safe_sup(2) > safe_inf(2)
% 
%                 x1 = safe_inf(1); y1 = safe_inf(2);
%                 x2 = safe_sup(1); y2 = safe_sup(2);
% 
%                 % Draw using plot (legend-safe)
%                 h_safe = plot([x1 x2 x2 x1 x1], [y1 y1 y2 y2 y1], ...
%                               'Color', [0 0.8 0], 'LineWidth', 2.5, 'DisplayName', 'Safe Set');
% 
%                 % Add to legend
%                 proxy_handles = [proxy_handles, h_safe];
%             else
%                 warning('Safe set invalid or not 2D — not plotted, but still returned for verification.');
%             end
%         end
% 
% 
%         % Build legend: sampled points + all proxy handles in correct order
%         legend_entries = [plot(NaN, NaN, 'k.', 'MarkerSize', 5)];
%         legend_labels  = {'Sampled points'};
% 
%         % Add in the EXACT order they were pushed to proxy_handles
%         label_counter = 1;
% 
%         % 1. Mixed-Monotonicity (Boundary) - Magenta dashed
%         if exist('h_proxy_bound1', 'var')
%             legend_entries(end+1) = h_proxy_bound1;
%             legend_labels{end+1}  = 'Mixed-Monotonicity (Boundary)';
%         end
% 
%         % 2. Sampled-Data MM (Boundary) - Green dashed
%         if exist('h_proxy_bound2', 'var')
%             legend_entries(end+1) = h_proxy_bound2;
%             legend_labels{end+1}  = 'Sampled-Data Mixed-Monotonicity (Boundary)';
%         end
% 
%         % 3. Mixed-Monotonicity (Single-step) - Blue solid
%         if exist('h_proxy_single1', 'var')
%             legend_entries(end+1) = h_proxy_single1;
%             legend_labels{end+1}  = 'Mixed-Monotonicity (Single-step)';
%         end
% 
%         % 4. Sampled-Data MM (Single-step) - Red solid
%         if exist('h_proxy_single2', 'var')
%             legend_entries(end+1) = h_proxy_single2;
%             legend_labels{end+1}  = 'Sampled-Data Mixed-Monotonicity (Single-step)';
%         end
% 
%         % 5. Safe Set - Thick Green
%         if exist('h_safe', 'var')
%             legend_entries(end+1) = h_safe;
%             legend_labels{end+1}  = 'Safe Set';
%         end
% 
%         % Create legend
%         lgd = legend(legend_entries, legend_labels, 'Location', 'southeast');
%         set(lgd, 'FontSize', 5, 'FontWeight', 'bold');
% 
%         % Save the reachability plot
%         if exist(save_dir, 'dir')
%             try
%                 filename_base = sprintf('reachability_plot_%d_%d_%s', idx1, idx2, timestamp);
%                 saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
%                 fprintf('Reachability plot saved as PNG: %s\n', fullfile(save_dir, [filename_base '.png']));
%                 saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
%                 fprintf('Reachability plot saved as EPS: %s\n', fullfile(save_dir, [filename_base '.eps']));
%             catch e
%                 warning('Failed to save reachability plot: %s', e.message);
%             end
%         else
%             warning('Directory %s does not exist. Skipping reachability plot save.', save_dir);
%         end
% 
%         %% Tightness
%         fprintf('Tightness Metrics (Area Ratio) for x_%d vs x_%d plane:\n', idx1, idx2);
%         fprintf('(Single-step) CT Mixed-Monotonicity: %.2f\n', tightness_metrics(3));
%         fprintf('(Boundary) CT Mixed-Monotonicity: %.2f\n', tightness_metrics(1));
%         fprintf('(Single-step) CT Sampled-Data Mixed-Monotonicity: %.2f\n', tightness_metrics(4));
%         fprintf('(Boundary) CT Sampled-Data Mixed-Monotonicity: %.2f\n', tightness_metrics(2));
%     end
% catch e
%     fprintf('Error in plotting: %s\n', e.message);
%     rethrow(e);
% end
% 
% %% Plot the Boundary reachability splits (10~5D or 4~2D)
% try
%     % Define plot pairs based on system choice
%     plot_pairs = [];
%     if system_choice == 14
%         plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
%     elseif system_choice == 15
%         plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
%     end
%     colors = jet(num_splits); % Generate a range of colors for boundary splits
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     for p = 1:size(plot_pairs, 1)
%         % Plot boundary points and union interval hull
%         figure('Name', sprintf('Boundary Points - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
%         hold on;
%         grid on;
%         idx1 = plot_pairs(p, 1);
%         idx2 = plot_pairs(p, 2);
% 
%         % Plot the successors from random initial states
%         h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Calculate initial axis limits with padding
%         x_min = min(rand_succ(idx1, :));
%         x_max = max(rand_succ(idx1, :));
%         y_min = min(rand_succ(idx2, :));
%         y_max = max(rand_succ(idx2, :));
% 
%         if plot_last_only
%             k = size(succ_low_boundary{1}, 2); % Use actual size of second dimension
%         else
%             k = n_t; % Use all time steps
%         end
% 
%         % Debug: Check the size of the boundary arrays
%         fprintf('Size of succ_low_boundary{1}: %s\n', mat2str(size(succ_low_boundary{1})));
% 
%         % Plot boundary points (vertices of boundary splits) with unified colors
%         handles = [];
%         for i = 1:num_splits
%             for m = 1:n_methods
%                 x_min_bound = succ_low_boundary{i}(idx1, k, m);
%                 x_max_bound = succ_up_boundary{i}(idx1, k, m);
%                 y_min_bound = succ_low_boundary{i}(idx2, k, m);
%                 y_max_bound = succ_up_boundary{i}(idx2, k, m);
%                 % Plot vertices of the boundary rectangle
%                 vertices_x = [x_min_bound, x_max_bound, x_max_bound, x_min_bound];
%                 vertices_y = [y_min_bound, y_min_bound, y_max_bound, y_max_bound];
%                 if all(isfinite([vertices_x, vertices_y]))
%                     if m == 1
%                         plot_color = 'b';
%                         method_label = 'CT Mixed-Monotonicity';
%                     else
%                         plot_color = 'r';
%                         method_label = 'CT Sampled-Data Mixed-Monotonicity';
%                     end
%                     h = plot(vertices_x, vertices_y, 'o', 'Color', plot_color, 'MarkerSize', 5, ...
%                              'MarkerFaceColor', plot_color, 'DisplayName', method_label);
%                     rectangle('Position', [x_min_bound, y_min_bound, x_max_bound - x_min_bound, y_max_bound - y_min_bound]);
%                     if i == 1
%                         handles = [handles, h(1)];
%                     end
%                 else
%                     fprintf('Warning: Invalid bounds for split %d, method %d: skipping points\n', i, m);
%                 end
%                 x_min = min(x_min, x_min_bound);
%                 x_max = max(x_max, x_max_bound);
%                 y_min = min(y_min, y_min_bound);
%                 y_max = max(y_max, y_max_bound);
%             end
%         end
% 
%         % Adjust axis with larger padding
%         x_range = x_max - x_min;
%         y_range = y_max - y_min;
%         buffer = 0.4; % Increased to 50% padding for better zoom out
%         axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);
% 
%         % Add labels
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['Boundary Points - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
% 
%         % Add legend
%         if ~isempty(handles)
%             legend([h_random, handles], {'Random Successors', 'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'}, 'Location', 'southeast');
%         else
%             fprintf('Warning: No valid handles for legend, skipping legend.\n');
%         end
% 
%         % Save the boundary points plot
%         if exist(save_dir, 'dir')
%             try
%                 filename_base = sprintf('boundary_points_%d_%d_%s', idx1, idx2, timestamp);
%                 saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
%                 fprintf('Boundary points plot saved as PNG: %s\n', fullfile(save_dir, [filename_base '.png']));
%                 saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
%                 fprintf('Boundary points plot saved as EPS: %s\n', fullfile(save_dir, [filename_base '.eps']));
%             catch e
%                 warning('Failed to save boundary points plot: %s', e.message);
%             end
%         else
%             warning('Directory %s does not exist. Skipping boundary points plot save.', save_dir);
%         end
% 
%         % Plot Interval Hulls for each OA method
%         figure('Name', sprintf('Interval Hull - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
%         hold on;
%         grid on;
% 
%         % Plot random successors again
%         h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Plot per-method interval hulls
%         x_min = min(rand_succ(idx1, :));
%         x_max = max(rand_succ(idx1, :));
%         y_min = min(rand_succ(idx2, :));
%         y_max = max(rand_succ(idx2, :));
%         handles = [];
%         for m = 1:n_methods
%             x_min_hull_m = Inf;
%             x_max_hull_m = -Inf;
%             y_min_hull_m = Inf;
%             y_max_hull_m = -Inf;
%             for i = 1:num_splits
%                 x_min_hull_m = min(x_min_hull_m, succ_low_boundary{i}(idx1, k, m));
%                 x_max_hull_m = max(x_max_hull_m, succ_up_boundary{i}(idx1, k, m));
%                 y_min_hull_m = min(y_min_hull_m, succ_low_boundary{i}(idx2, k, m));
%                 y_max_hull_m = max(y_max_hull_m, succ_up_boundary{i}(idx2, k, m));
%             end
%             width_hull_m = x_max_hull_m - x_min_hull_m;
%             height_hull_m = y_max_hull_m - y_min_hull_m;
%             if all(isfinite([x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m])) && width_hull_m >= 0 && height_hull_m >= 0
%                 if m == 1
%                     plot_color = 'b';
%                 else
%                     plot_color = 'r';
%                 end
%                 h_method = rectangle('Position', [x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m], ...
%                                     'EdgeColor', plot_color, 'LineWidth', 2, 'LineStyle', '-', 'FaceAlpha', 0.1);
%                 % Create proxy for legend
%                 h_method_proxy = plot(NaN, NaN, plot_color, 'LineWidth', 2);
%                 handles = [handles, h_method_proxy];
%             else
%                 fprintf('Warning: Invalid bounds for interval hull, method %d: skipping rectangle\n', m);
%             end
%             x_min = min(x_min, x_min_hull_m);
%             x_max = max(x_max, x_max_hull_m);
%             y_min = min(y_min, y_min_hull_m);
%             y_max = max(y_max, y_max_hull_m);
%         end
% 
%         % Adjust axis
%         x_range = x_max - x_min;
%         y_range = y_max - y_min;
%         axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);
% 
%         % Add labels
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['Interval Hull - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
% 
%         % Add legend
%         if ~isempty(handles)
%             legend([h_random, handles], {'Random Successors', method_names{:}}, 'Location', 'southeast');
%         else
%             fprintf('Warning: No valid handles for legend, skipping legend.\n');
%         end
% 
%         % Save the figure
%         saveas(gcf, fullfile(save_dir, sprintf('interval_hull_%d_%d_%s.png', idx1, idx2, timestamp)), 'png');
%     end
% catch e
%     fprintf('Error in plotting: %s\n', e.message);
%     rethrow(e);
% end
% 
% % Display timing results
% fprintf('Computation Times (seconds) for all plots:\n');
% for m = 1:n_methods
%     fprintf('(Single-step) %s: %.6f\n', method_names{m}, time_initial(m));
%     fprintf('(Boundary) %s: %.6f\n', method_names{m}, sum(time_boundary(:, m)));
% end
% 
% %% Save the reachable sets and random successors
% try
%     timestamp = datestr(now, 'yyyy-mm-dd');
%     filename_base = sprintf('reachability_data_sys_TIRA%d_all_%s', system_choice, timestamp);
%     save(fullfile(save_dir, [filename_base '.mat']), 'succ_low_all_methods', 'succ_up_all_methods', ...
%          'succ_low_boundary', 'succ_up_boundary', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', ...
%          'time_initial', 'time_boundary');
%     disp(['Data saved to ' fullfile(save_dir, [filename_base '.mat'])]);
% catch e
%     fprintf('Error in saving data: %s\n', e.message);
%     rethrow(e);
% end
% 
% 
% %% === RETURN ALL reachability_results ===
% reachability_results = struct();
% reachability_results.succ_low_all_methods = succ_low_all_methods;
% reachability_results.succ_up_all_methods  = succ_up_all_methods;
% reachability_results.succ_low_boundary    = succ_low_boundary;
% reachability_results.succ_up_boundary     = succ_up_boundary;
% reachability_results.rand_succ            = rand_succ;
% reachability_results.t_points             = t_points;
% reachability_results.time_initial         = time_initial;
% reachability_results.time_boundary        = time_boundary;
% reachability_results.save_dir             = save_dir;
% reachability_results.plot_pairs           = [1 2];  % Spiral: only x1 vs x2
% if system_choice == 14
%     reachability_results.plot_pairs = [1 2; 3 4; 4 5];
% end
% reachability_results.plot_last_only       = plot_last_only;
% reachability_results.n_x                  = n_x;
% reachability_results.n_methods            = n_methods;
% reachability_results.methods              = methods;
% reachability_results.system_choice        = system_choice;
% reachability_results.num_splits           = num_splits;
% reachability_results.plot_pairs = plot_pairs;  % Already set correctly
% 
% 
% % =========================================================================
% % === ENSURE safe_inf and safe_sup are ALWAYS returned to Verifier.m ===
% % =========================================================================
% if isfield(params, 'safe_inf') && isfield(params, 'safe_sup')
%     reachability_results.safe_inf = params.safe_inf;
%     reachability_results.safe_sup = params.safe_sup;
% else
%     reachability_results.safe_inf = [];
%     reachability_results.safe_sup = [];
% end
% 
% end
% 
% 
% %------------- END OF CODE --------------








%------------- BEGIN CODE with Correct Fix to verifiy the 4 OAs--------------


function reachability_results = NODE_RA(params)
% NODE_RA - Reachability analysis for neural ODEs using TIRA
% Input:
%   params - struct with fields:
%       .system_choice      : 14 or 15
%       .x0                 : initial center
%       .Initial_radius     : radius of initial set
%       .t_final            : final time
%       .timeStep           : discretization step
%       .plot_last_only     : true/false
%       .sample_succ_number : number of random samples
% Output:
%   reachability_results - struct containing all reachable sets, plots, etc.

%% Initialization
close all
bool_discrete_time = 0;
addpath('../OA_methods')
addpath('../Utilities')    

%% === PLOTTING CONTROL DEFAULTS (NEW) ===
default_plot_control.show_figures               = true;
default_plot_control.save_each_iteration        = false;
default_plot_control.save_final_only_if_verified = true;
default_plot_control.save_reachability          = true;
default_plot_control.save_boundary              = true;
default_plot_control.save_interval_hull         = true;

if isfield(params, 'plot_control')
    pc = params.plot_control;
    
    % ---- NEW SAFE ACCESS -------------------------------------------------
    plot_control.show_figures               = getOrDefault(pc,'show_figures', default_plot_control.show_figures);
    plot_control.save_each_iteration        = getOrDefault(pc,'save_each_iteration', default_plot_control.save_each_iteration);
    plot_control.save_final_only_if_verified = getOrDefault(pc,'save_final_only_if_verified', default_plot_control.save_final_only_if_verified);
    plot_control.save_reachability          = getOrDefault(pc,'save_reachability', default_plot_control.save_reachability);
    plot_control.save_boundary              = getOrDefault(pc,'save_boundary', default_plot_control.save_boundary);
    plot_control.save_interval_hull         = getOrDefault(pc,'save_interval_hull', default_plot_control.save_interval_hull);
    % --------------------------------------------------------------------
else
    plot_control = default_plot_control;
end

% -----------------------------------------------------------------------
% Helper that works even if the field does not exist
function val = getOrDefault(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
% -----------------------------------------------------------------------

%% User-defined parameters (override defaults with input)
plot_last_only = true;      % Default
if isfield(params, 'plot_last_only')
    plot_last_only = params.plot_last_only;
end

sample_succ_number = 1000;  % Default
if isfield(params, 'sample_succ_number')
    sample_succ_number = params.sample_succ_number;
end

%% Choice of the example system
global system_choice
system_choice = params.system_choice;

% Map system choice to system name
system_names = containers.Map({14, 15}, {'fpa', 'spiral_non'});
if ~isKey(system_names, system_choice)
    error('Unknown system_choice %d.', system_choice);
end
system_name = system_names(system_choice);

%% Optional submethod choice
global g_sensitivity_bounds_method
g_sensitivity_bounds_method = nan;

%% Definition of the reachability problem

global u system_choice
system_choice = get_from_params(params, 'system_choice');

% === Unified Parameter Loading(both systems) ===

Initial_radius = get_from_params(params, 'Initial_radius');
x0             = get_from_params(params, 'x0');
t_final        = get_from_params(params, 't_final');
steps          = get_from_params(params, 'timeStep');

x_low = x0 - Initial_radius;
x_up  = x0 + Initial_radius;
t_init = 0;

p_low = zeros(length(x_low),1); 
p_up  = zeros(length(x_low),1);

% Dimensions
n_x = length(x_low);
n_p = length(p_low);

% Verify dimensions
if system_choice == 14 && (n_x ~= 5 || n_p ~= 5)
    error('FPA expects 5D state/parameter.');
elseif system_choice == 15 && (n_x ~= 2 || n_p ~= 2)
    error('Spiral_non expects 2D state/parameter.');
end


% === DEFINE 4 METHODS (Single-step + Boundary) ===
methods = [3, 4];  % TIRA codes: 3 = CT MMM, 4 = SD MM
n_methods_single_step = length(methods);  % 2
n_methods_total = 4;  % 2 Single-step OAs + 2 Boundary OAs

% Preallocate for all 4 methods
succ_low_all_methods = NaN(n_x, 1, n_methods_total); % 1: RA @ Final time
succ_up_all_methods = NaN(n_x, 1, n_methods_total);

%% Boundary splits
num_splits = 2*n_x;
Initial_set_boundaries = cell(num_splits, 1);
succ_low_boundary = cell(num_splits, 1);
succ_up_boundary  = cell(num_splits, 1);
boundary_times = zeros(num_splits, n_methods_single_step);

for i = 1:n_x
    temp_low = x_low; temp_up = x_up; temp_low(i) = x_up(i);
    Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
    temp_low = x_low; temp_up = x_up; temp_up(i) = x_low(i);
    Initial_set_boundaries{2*i} = interval(temp_low(:), temp_up(:));
end

%% Main reachability computation
try
    time_initial = zeros(1, n_methods_single_step);
    time_boundary = zeros(num_splits, n_methods_single_step);

    % Single-step (initial approach)
    for m = 1:n_methods_single_step
        tStart = tic;
        [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
        time_initial(m) = toc(tStart);
        succ_low_all_methods(:, 1, m) = succ_low;
        succ_up_all_methods(:, 1, m)  = succ_up;
    end

    % Boundary approach
    for i = 1:num_splits
        succ_low_boundary{i} = NaN(n_x, 1, n_methods_single_step);  
        succ_up_boundary{i}  = NaN(n_x, 1, n_methods_single_step);
        for m = 1:n_methods_single_step
            tStart = tic;
            [succ_low, succ_up] = TIRA([t_init, t_final], ...
                Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
            time_boundary(i, m) = toc(tStart);
            succ_low_boundary{i}(:, 1, m) = succ_low;  % Only final time
            succ_up_boundary{i}(:, 1, m)  = succ_up;
         end
     end

catch e
    rethrow(e);
end

%% Random successors
try
    % === CALL CONFIG FUNCTION ===
    global g_sensitivity_bounds_method
    parameters = get_solver_parameters(n_x, n_p, g_sensitivity_bounds_method);

    % Now use it
    ode_solver = parameters.ode_solver;
    ode_options = parameters.ode_options;

    rand_succ = NaN(n_x, sample_succ_number);
    for i = 1:sample_succ_number
        x0_rand = x_low + rand(n_x,1).*(x_up - x_low);
        p_rand  = p_low + rand(n_p,1).*(p_up - p_low);
        [~, x_traj] = ode_solver(@(t,x) System_description(t,x,p_rand), [t_init t_final], x0_rand, ode_options);
        rand_succ(:,i) = x_traj(end,:)';
    end
catch e
    rethrow(e);
end


%% === CREATE SAVE DIRECTORY ONLY IF SAVING ===
save_anything = plot_control.save_each_iteration || ...
    (isfield(params, 'is_final_call') && params.is_final_call && ...
     (~plot_control.save_final_only_if_verified || (isfield(params, 'fully_verified') && params.fully_verified)));

if save_anything
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    save_dir = fullfile(pwd, sprintf('results_%s_%s', system_name, timestamp));
    if ~exist(save_dir, 'dir')
        [status, msg] = mkdir(save_dir);
        if status ~= 1
            warning('Failed to create directory %s: %s', save_dir, msg);
            save_dir = '';
        else
            fprintf('Created directory: %s\n', save_dir);
        end
    end
else
    save_dir = '';
end


%% Plot the reachable set and over-approximations
try
    % Define plot pairs based on system choice
    plot_pairs = [];
    if system_choice == 14
        plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
    elseif system_choice == 15
        plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
    end
    colors = [0 0 1; 1 0 0]; % Dark Blue, Red
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    for p = 1:size(plot_pairs, 1)
        figure('Name', sprintf('Reachability Plot - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
        hold on;
        grid on;
        idx1 = plot_pairs(p, 1);
        idx2 = plot_pairs(p, 2);

        % Plot the successors from random initial states
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Calculate minimum bounding box for black dots
        x_min_actual = min(rand_succ(idx1, :));
        x_max_actual = max(rand_succ(idx1, :));
        y_min_actual = min(rand_succ(idx2, :));
        y_max_actual = max(rand_succ(idx2, :));
        area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));

        % Plot over-approximation rectangles
        handles = [];
        proxy_handles = [];
        tightness_metrics = zeros(1, 4); % For 4 approaches (Single-step and Boundary only)

        
        % === FINAL TIME INDEX: ALWAYS 1 ===
        k = 1;

        % 1: Mixed-Monotonicity (Boundary) - Dashed Magenta
        all_x_min_bound = [];
        all_x_max_bound = [];
        all_y_min_bound = [];
        all_y_max_bound = [];
        for i = 1:num_splits
            % fprintf('Boundary data m=1, split %d: low=[%f %f], up=[%f %f]\n', i, ...
            %         succ_low_boundary{i}(idx1, k, 1), succ_low_boundary{i}(idx2, k, 1), ...
            %         succ_up_boundary{i}(idx1, k, 1), succ_up_boundary{i}(idx2, k, 1));
            all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 1)];
            all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 1)];
            all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 1)];
            all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 1)];
        end
        x_min_bound = min(all_x_min_bound);
        x_max_bound = max(all_x_max_bound);
        y_min_bound = min(all_y_min_bound);
        y_max_bound = max(all_y_max_bound);
        width_bound = x_max_bound - x_min_bound;
        height_bound = y_max_bound - y_min_bound;
        fprintf('Mixed-Monotonicity (Boundary) computed bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                x_min_bound, y_min_bound, width_bound, height_bound);
        if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
            h_bound1 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
                                'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
            h_proxy_bound1 = plot(NaN, NaN, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_bound1];
            proxy_handles = [proxy_handles, h_proxy_bound1];
            tightness_metrics(1) = (width_bound * height_bound) / area_actual;
        else
            warning('Mixed-Monotonicity (Boundary) has invalid bounds. Using actual range.');
            h_bound1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
                               'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1, 'LineStyle', ':');
            h_proxy_bound1 = plot(NaN, NaN, 'm:', 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_bound1];
            proxy_handles = [proxy_handles, h_proxy_bound1];
            tightness_metrics(1) = 1.0;
        end

        % 2: Sampled-Data Mixed-Monotonicity (Boundary) - Dashed Green
        all_x_min_bound = [];
        all_x_max_bound = [];
        all_y_min_bound = [];
        all_y_max_bound = [];
        for i = 1:num_splits
            % fprintf('Boundary data m=2, split %d: low=[%f %f], up=[%f %f]\n', i, ...
            %         succ_low_boundary{i}(idx1, k, 2), succ_low_boundary{i}(idx2, k, 2), ...
            %         succ_up_boundary{i}(idx1, k, 2), succ_up_boundary{i}(idx2, k, 2));
            all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 2)];
            all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 2)];
            all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 2)];
            all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 2)];
        end
        x_min_bound = min(all_x_min_bound);
        x_max_bound = max(all_x_max_bound);
        y_min_bound = min(all_y_min_bound);
        y_max_bound = max(all_y_max_bound);
        width_bound = x_max_bound - x_min_bound;
        height_bound = y_max_bound - y_min_bound;
        fprintf('Sampled-Data Mixed-Monotonicity (Boundary) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                x_min_bound, y_min_bound, width_bound, height_bound);
        if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
            h_bound2 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
                                'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
            h_proxy_bound2 = plot(NaN, NaN, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_bound2];
            proxy_handles = [proxy_handles, h_proxy_bound2];
            tightness_metrics(2) = (width_bound * height_bound) / area_actual;
        else
            warning('Skipping Sampled-Data Mixed-Monotonicity (Boundary) due to invalid parameters');
        end

        % 3: Mixed-Monotonicity (Single-step) - Dark Blue Solid
        x_min_single = succ_low_all_methods(idx1, k, 1); % Method 1
        y_min_single = succ_low_all_methods(idx2, k, 1);
        width_single = succ_up_all_methods(idx1, k, 1) - x_min_single;
        height_single = succ_up_all_methods(idx2, k, 1) - y_min_single;
        fprintf('Mixed-Monotonicity (Single-step) computed bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                x_min_single, y_min_single, width_single, height_single);
        if all(isfinite([x_min_single, y_min_single, width_single, height_single])) && width_single >= 0 && height_single >= 0
            h_single1 = rectangle('Position', [x_min_single, y_min_single, max(width_single, 0.001), max(height_single, 0.001)], ...
                                'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
            h_proxy_single1 = plot(NaN, NaN, '-', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_single1];
            proxy_handles = [proxy_handles, h_proxy_single1];
            tightness_metrics(3) = (width_single * height_single) / area_actual;
        else
            warning('Mixed-Monotonicity (Single-step) has invalid bounds. Using actual range.');
            h_single1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
                               'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1, 'LineStyle', ':');
            h_proxy_single1 = plot(NaN, NaN, ':', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_single1];
            proxy_handles = [proxy_handles, h_proxy_single1];
            tightness_metrics(3) = 1.0;
        end

        % 4: Sampled-Data Mixed-Monotonicity (Single-step) - Red Solid
        x_min_single = succ_low_all_methods(idx1, k, 2); % Method 2
        y_min_single = succ_low_all_methods(idx2, k, 2);
        width_single = succ_up_all_methods(idx1, k, 2) - x_min_single;
        height_single = succ_up_all_methods(idx2, k, 2) - y_min_single;
        fprintf('Sampled-Data Mixed-Monotonicity (Single-step) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                x_min_single, y_min_single, width_single, height_single);
        if all(isfinite([x_min_single, y_min_single, width_single, height_single])) && width_single >= 0 && height_single >= 0
            h_single2 = rectangle('Position', [x_min_single, y_min_single, max(width_single, 0.001), max(height_single, 0.001)], ...
                                'EdgeColor', colors(2,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
            h_proxy_single2 = plot(NaN, NaN, '-', 'Color', colors(2,:), 'LineWidth', 1.5, 'Visible', 'off');
            handles = [handles, h_single2];
            proxy_handles = [proxy_handles, h_proxy_single2];
            tightness_metrics(4) = (width_single * height_single) / area_actual;
        else
            warning('Skipping Sampled-Data Mixed-Monotonicity (Single-step) due to invalid parameters');
        end
    end

        % Adjust axis to include all bounds
        x_min_all = min([x_min_actual, x_min_bound, x_min_single]);
        x_max_all = max([x_max_actual, x_max_bound + width_bound, x_min_single + width_single]);
        y_min_all = min([y_min_actual, y_min_bound, y_min_single]);
        y_max_all = max([y_max_actual, y_max_bound + height_bound, y_min_single + height_single]);
        x_range = x_max_all - x_min_all;
        y_range = y_max_all - y_min_all;
        buffer = 0.3; % 10% padding
        axis([x_min_all - buffer*x_range, x_max_all + buffer*x_range, y_min_all - buffer*y_range, y_max_all + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);


        % <<< SAFE-SET: Green Thick Rectangle (drawn with plot to support legend) >>>
        if isfield(params,'safe_inf') && isfield(params,'safe_sup')
            safe_inf = params.safe_inf;
            safe_sup = params.safe_sup;

            % === Only draw if valid 2D bounds ===
            if numel(safe_inf) == 2 && numel(safe_sup) == 2 && ...
               all(isfinite(safe_inf)) && all(isfinite(safe_sup)) && ...
               safe_sup(1) > safe_inf(1) && safe_sup(2) > safe_inf(2)

                x1 = safe_inf(1); y1 = safe_inf(2);
                x2 = safe_sup(1); y2 = safe_sup(2);

                % Draw using plot (legend-safe)
                h_safe = plot([x1 x2 x2 x1 x1], [y1 y1 y2 y2 y1], ...
                              'Color', [0 0.8 0], 'LineWidth', 2.5, 'DisplayName', 'Safe Set');

                % Add to legend
                proxy_handles = [proxy_handles, h_safe];
            else
                warning('Safe set invalid or not 2D — not plotted, but still returned for verification.');
            end
        end


        % Build legend: sampled points + all proxy handles in correct order
        legend_entries = [plot(NaN, NaN, 'k.', 'MarkerSize', 5)];
        legend_labels  = {'Sampled points'};

        % Add in the EXACT order they were pushed to proxy_handles
        label_counter = 1;

        % 1. Mixed-Monotonicity (Boundary) - Magenta dashed
        if exist('h_proxy_bound1', 'var')
            legend_entries(end+1) = h_proxy_bound1;
            legend_labels{end+1}  = 'Mixed-Monotonicity (Boundary)';
        end

        % 2. Sampled-Data MM (Boundary) - Green dashed
        if exist('h_proxy_bound2', 'var')
            legend_entries(end+1) = h_proxy_bound2;
            legend_labels{end+1}  = 'Sampled-Data Mixed-Monotonicity (Boundary)';
        end

        % 3. Mixed-Monotonicity (Single-step) - Blue solid
        if exist('h_proxy_single1', 'var')
            legend_entries(end+1) = h_proxy_single1;
            legend_labels{end+1}  = 'Mixed-Monotonicity (Single-step)';
        end

        % 4. Sampled-Data MM (Single-step) - Red solid
        if exist('h_proxy_single2', 'var')
            legend_entries(end+1) = h_proxy_single2;
            legend_labels{end+1}  = 'Sampled-Data Mixed-Monotonicity (Single-step)';
        end

        % 5. Safe Set - Thick Green
        if exist('h_safe', 'var')
            legend_entries(end+1) = h_safe;
            legend_labels{end+1}  = 'Safe Set';
        end

        % Create legend
        lgd = legend(legend_entries, legend_labels, 'Location', 'southeast');
        set(lgd, 'FontSize', 5, 'FontWeight', 'bold');
        
        
        % Save the reachability plot
        if plot_control.save_reachability && ~isempty(save_dir) && ...
           (plot_control.save_each_iteration || (isfield(params, 'is_final_call') && params.is_final_call))
            try
                filename_base = sprintf('reachability_plot_%d_%d_%s', idx1, idx2, timestamp);
                saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
                fprintf('Reachability plot saved as PNG: %s\n', fullfile(save_dir, [filename_base '.png']));
                saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
                fprintf('Reachability plot saved as EPS: %s\n', fullfile(save_dir, [filename_base '.eps']));
            catch e
                warning('Failed to save reachability plot: %s', e.message);
            end
        end

        if ~plot_control.show_figures
            close(gcf);
        end

catch e
    fprintf('Error in plotting: %s\n', e.message);
    rethrow(e);
end
        

        % Save the reachability plot
        if exist(save_dir, 'dir')
            try
                filename_base = sprintf('reachability_plot_%d_%d_%s', idx1, idx2, timestamp);
                saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
                fprintf('Reachability plot saved as PNG: %s\n', fullfile(save_dir, [filename_base '.png']));
                saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
                fprintf('Reachability plot saved as EPS: %s\n', fullfile(save_dir, [filename_base '.eps']));
            catch e
                warning('Failed to save reachability plot: %s', e.message);
            end
        else
            warning('Directory %s does not exist. Skipping reachability plot save.', save_dir);
        end

        %% Tightness
        fprintf('Tightness Metrics (Area Ratio) for x_%d vs x_%d plane:\n', idx1, idx2);
        fprintf('(Single-step) CT Mixed-Monotonicity: %.2f\n', tightness_metrics(3));
        fprintf('(Boundary) CT Mixed-Monotonicity: %.2f\n', tightness_metrics(1));
        fprintf('(Single-step) CT Sampled-Data Mixed-Monotonicity: %.2f\n', tightness_metrics(4));
        fprintf('(Boundary) CT Sampled-Data Mixed-Monotonicity: %.2f\n', tightness_metrics(2));

%% Plot the Boundary reachability splits (10~5D or 4~2D)
try
    % Define plot pairs based on system choice
    plot_pairs = [];
    if system_choice == 14
        plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
    elseif system_choice == 15
        plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
    end
    colors = jet(num_splits); % Generate a range of colors for boundary splits
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    for p = 1:size(plot_pairs, 1)
        % Plot boundary points and union interval hull
        figure('Name', sprintf('Boundary Points - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
        hold on;
        grid on;
        idx1 = plot_pairs(p, 1);
        idx2 = plot_pairs(p, 2);

        % Plot the successors from random initial states
        h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Calculate initial axis limits with padding
        x_min = min(rand_succ(idx1, :));
        x_max = max(rand_succ(idx1, :));
        y_min = min(rand_succ(idx2, :));
        y_max = max(rand_succ(idx2, :));
        
        % Method 3: Mixed-Monotonicity (Boundary) — union of all boundary splits
        all_low_m3 = arrayfun(@(i) succ_low_boundary{i}(:, 1, 1), 1:num_splits, 'UniformOutput', false);
        all_up_m3  = arrayfun(@(i) succ_up_boundary{i}(:, 1, 1), 1:num_splits, 'UniformOutput', false);
        succ_low_all_methods(:, 1, 3) = min(cat(2, all_low_m3{:}), [], 2);
        succ_up_all_methods(:, 1, 3)  = max(cat(2, all_up_m3{:}), [], 2);
        
        % Method 4: Sampled-Data Mixed-Monotonicity (Boundary)
        all_low_m4 = arrayfun(@(i) succ_low_boundary{i}(:, 1, 2), 1:num_splits, 'UniformOutput', false);
        all_up_m4  = arrayfun(@(i) succ_up_boundary{i}(:, 1, 2), 1:num_splits, 'UniformOutput', false);
        succ_low_all_methods(:, 1, 4) = min(cat(2, all_low_m4{:}), [], 2);
        succ_up_all_methods(:, 1, 4)  = max(cat(2, all_up_m4{:}), [], 2);
        
        % Fill earlier time steps with final bounds (for consistency)
        for m = 3:4
            succ_low_all_methods(:, 1:k-1, m) = repmat(succ_low_all_methods(:, k, m), 1, k-1);
            succ_up_all_methods(:, 1:k-1, m)  = repmat(succ_up_all_methods(:, k, m), 1, k-1);
        end



        % Debug: Check the size of the boundary arrays
        fprintf('Size of succ_low_boundary{1}: %s\n', mat2str(size(succ_low_boundary{1})));

        % Plot boundary points (vertices of boundary splits) with unified colors
        handles = [];
        for i = 1:num_splits
            for m = 1:n_methods_single_step
                x_min_bound = succ_low_boundary{i}(idx1, 1, m);
                x_max_bound = succ_up_boundary{i}(idx1, 1, m);
                y_min_bound = succ_low_boundary{i}(idx2, 1, m);
                y_max_bound = succ_up_boundary{i}(idx2, 1, m);
                % Plot vertices of the boundary rectangle
                vertices_x = [x_min_bound, x_max_bound, x_max_bound, x_min_bound];
                vertices_y = [y_min_bound, y_min_bound, y_max_bound, y_max_bound];
                if all(isfinite([vertices_x, vertices_y]))
                    if m == 1
                        plot_color = 'b';
                        method_label = 'CT Mixed-Monotonicity';
                    else
                        plot_color = 'r';
                        method_label = 'CT Sampled-Data Mixed-Monotonicity';
                    end
                    h = plot(vertices_x, vertices_y, 'o', 'Color', plot_color, 'MarkerSize', 5, ...
                             'MarkerFaceColor', plot_color, 'DisplayName', method_label);
                    rectangle('Position', [x_min_bound, y_min_bound, x_max_bound - x_min_bound, y_max_bound - y_min_bound]);
                    if i == 1
                        handles = [handles, h(1)];
                    end
                else
                    fprintf('Warning: Invalid bounds for split %d, method %d: skipping points\n', i, m);
                end
                x_min = min(x_min, x_min_bound);
                x_max = max(x_max, x_max_bound);
                y_min = min(y_min, y_min_bound);
                y_max = max(y_max, y_max_bound);
            end
        end

        % Adjust axis with larger padding
        x_range = x_max - x_min;
        y_range = y_max - y_min;
        buffer = 0.4; % Increased to 50% padding for better zoom out
        axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['Boundary Points - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add legend
        if ~isempty(handles)
            legend([h_random, handles], {'Random Successors', 'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'}, 'Location', 'southeast');
        else
            fprintf('Warning: No valid handles for legend, skipping legend.\n');
        end
        
        % Save the boundary points plot
        if plot_control.save_boundary && ~isempty(save_dir) && ...
       (plot_control.save_each_iteration || (isfield(params, 'is_final_call') && params.is_final_call))
        filename_base = sprintf('boundary_points_%d_%d_%s', idx1, idx2, timestamp);
        saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
        saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
    end

    if ~plot_control.show_figures
        close(gcf);
    end
end

        % Plot Interval Hulls for each OA method
        figure('Name', sprintf('Interval Hull - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
        hold on;
        grid on;

        % Plot random successors again
        h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Plot per-method interval hulls
        x_min = min(rand_succ(idx1, :));
        x_max = max(rand_succ(idx1, :));
        y_min = min(rand_succ(idx2, :));
        y_max = max(rand_succ(idx2, :));
        handles = [];
        for m = 1:n_methods_single_step
            x_min_hull_m = Inf;
            x_max_hull_m = -Inf;
            y_min_hull_m = Inf;
            y_max_hull_m = -Inf;
            for i = 1:num_splits
                x_min_hull_m = min(x_min_hull_m, succ_low_boundary{i}(idx1, k, m));
                x_max_hull_m = max(x_max_hull_m, succ_up_boundary{i}(idx1, k, m));
                y_min_hull_m = min(y_min_hull_m, succ_low_boundary{i}(idx2, k, m));
                y_max_hull_m = max(y_max_hull_m, succ_up_boundary{i}(idx2, k, m));
            end
            width_hull_m = x_max_hull_m - x_min_hull_m;
            height_hull_m = y_max_hull_m - y_min_hull_m;
            if all(isfinite([x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m])) && width_hull_m >= 0 && height_hull_m >= 0
                if m == 1
                    plot_color = 'b';
                else
                    plot_color = 'r';
                end
                h_method = rectangle('Position', [x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m], ...
                                    'EdgeColor', plot_color, 'LineWidth', 2, 'LineStyle', '-', 'FaceAlpha', 0.1);
                % Create proxy for legend
                h_method_proxy = plot(NaN, NaN, plot_color, 'LineWidth', 2);
                handles = [handles, h_method_proxy];
            else
                fprintf('Warning: Invalid bounds for interval hull, method %d: skipping rectangle\n', m);
            end
            x_min = min(x_min, x_min_hull_m);
            x_max = max(x_max, x_max_hull_m);
            y_min = min(y_min, y_min_hull_m);
            y_max = max(y_max, y_max_hull_m);
        end

        % Adjust axis
        x_range = x_max - x_min;
        y_range = y_max - y_min;
        axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['Interval Hull - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add legend
        if ~isempty(handles)
            legend([h_random, handles], {'Random Successors', method_names{:}}, 'Location', 'southeast');
        else
            fprintf('Warning: No valid handles for legend, skipping legend.\n');
        end
        

        % save Interval Hull
        if plot_control.save_interval_hull && ~isempty(save_dir) && ...
       (plot_control.save_each_iteration || (isfield(params, 'is_final_call') && params.is_final_call))
        filename_base = sprintf('interval_hull_%d_%d_%s', idx1, idx2, timestamp);
        saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
         end

    if ~plot_control.show_figures
        close(gcf);
    end
end
        
% Display timing results
fprintf('Computation Times (seconds) for all plots:\n');
for m = 1:n_methods_single_step
    fprintf('(Single-step) %s: %.6f\n', method_names{m}, time_initial(m));
    fprintf('(Boundary) %s: %.6f\n', method_names{m}, sum(time_boundary(:, m)));
end

%% Save the reachable sets and random successors
try
    timestamp = datestr(now, 'yyyy-mm-dd');
    filename_base = sprintf('reachability_data_sys_TIRA%d_all_%s', system_choice, timestamp);
    save(fullfile(save_dir, [filename_base '.mat']), 'succ_low_all_methods', 'succ_up_all_methods', ...
         'succ_low_boundary', 'succ_up_boundary', 'rand_succ', 'methods', 'n_x', 'n_methods_total', ...
         'time_initial', 'time_boundary');
    disp(['Data saved to ' fullfile(save_dir, [filename_base '.mat'])]);
catch e
    fprintf('Error in saving data: %s\n', e.message);
    rethrow(e);
end


%% === RETURN ALL reachability_results ===
reachability_results = struct();
reachability_results.succ_low_all_methods = succ_low_all_methods;
reachability_results.succ_up_all_methods  = succ_up_all_methods;
reachability_results.succ_low_boundary    = succ_low_boundary;
reachability_results.succ_up_boundary     = succ_up_boundary;
reachability_results.rand_succ            = rand_succ;
reachability_results.time_initial         = time_initial;
reachability_results.time_boundary        = time_boundary;
reachability_results.save_dir             = save_dir;
reachability_results.plot_pairs           = [1 2];  % Spiral: only x1 vs x2
if system_choice == 14
    reachability_results.plot_pairs = [1 2; 3 4; 4 5];
end
reachability_results.n_x                  = n_x;
reachability_results.n_methods            = n_methods_total; % 4 OAs
reachability_results.methods              = methods;
reachability_results.system_choice        = system_choice;
reachability_results.num_splits           = num_splits;
reachability_results.plot_pairs = plot_pairs;  % Already set correctly


% =========================================================================
% === ENSURE safe_inf and safe_sup are ALWAYS returned to Verifier.m ===
% =========================================================================
if isfield(params, 'safe_inf') && isfield(params, 'safe_sup')
    reachability_results.safe_inf = params.safe_inf;
    reachability_results.safe_sup = params.safe_sup;
else
    reachability_results.safe_inf = [];
    reachability_results.safe_sup = [];
end

end

% =========================================================================
% === Helper Functions==
% =========================================================================
function val = get_from_params(params, field)
    % Safely extract field from params struct
    % Throws clear error if missing
    if isfield(params, field) && ~isempty(params.(field))
        val = params.(field);
    else
        error('NODE_RA: Missing required parameter "%s". Must be set in Verifier_FINAL.m', field);
    end
end


%------------- END OF CODE with Correct Fix to verifiy the 4 OAs --------------