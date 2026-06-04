% Paper:
%   Mixed Monotonicity Reachability Analysis of Neural ODE: A Trade-Off Between Tightness and Efficiency
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 17th of July 2025
% Last update: 30th of May 2026
% Last revision: 30th of May 2026


%------------- BEGIN CODE --------------

%% Initialization
close all
clear
bool_discrete_time = 0;
% Folder containing the various over-approximation methods
addpath('../OA_methods')
% Folder containing useful tools and functions
addpath('../Utilities')

%% User-defined approach and plotting selection
% Set plot_last_only to true to plot only the last reachable set, false for full reachable sets
plot_last_only = true; % Options: true or false

%% Choice of the example system
% To find a description of each of these systems (and references),
% look at their corresponding definitions in System_description.m.

global system_choice
system_choice = 14;      % FPA (continuous-time)
% system_choice = 15;     % Spiral_non (continuous-time)

% Map system choice to system name for directory naming
system_names = containers.Map({14, 15}, {'fpa', 'spiral_non'});
if ~isKey(system_names, system_choice)
    error('Unknown system_choice %d. Please define system name in system_names map.', system_choice);
end
system_name = system_names(system_choice);

%% Optional submethod choice for the sampled-data mixed-monotonicity method
% Kept as a TIRA-interface global (set to nan); not used by CT mixed-monotonicity.
global g_sensitivity_bounds_method
g_sensitivity_bounds_method = nan;

%% Definition of the reachability problem (for each example system)
global u        % Some systems need an external control input
switch system_choice
    case 14
        %% FPA System (continuous-time)
        % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
        Initial_radius = 0.01; % Uncertainty in dynamics
        x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
        x_low = x0 - Initial_radius;
        x_up = x0 + Initial_radius;

        % Define time and input bounds
        t_init = 0;     % Initial time
        t_final = 2;    % Final time
        steps = 0.05;    % Discretization steps / multiple subintervals
        t_points = t_init:steps:t_final;    % Time points: 0, 0.1, 0.2, ..., 2.0
        p_low = zeros(5,1);
        p_up = zeros(5,1);

    case 15
        %% Spiral_non System (continuous-time)
        % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
        Initial_radius = 0.2; % Uncertainty in dynamics
        x0 = [2.0; 0.0];
        x_low = x0 - Initial_radius;
        x_up = x0 + Initial_radius;

        % Define time and input bounds
        t_init = 0;     % Initial time
        t_final = 1;    % Final time
        steps = 0.01;    % Discretization steps 0.01
        t_points = t_init:steps:t_final;    % Time points
        p_low = zeros(2,1);
        p_up = zeros(2,1);
end

% State and input dimensions
n_x = length(x_low);
n_p = length(p_low);
n_t = length(t_points);

% Verify dimensions dynamically based on system
if system_choice == 14 && (n_x ~= 5 || n_p ~= 5)
    error('Expected n_x=5 and n_p=5 for FPA, got n_x=%d, n_p=%d', n_x, n_p);
elseif system_choice == 15 && (n_x ~= 2 || n_p ~= 2)
    error('Expected n_x=2 and n_p=2 for Spiral_non, got n_x=%d, n_p=%d', n_x, n_p);
end
if ~isvector(x_low) || length(x_low) ~= n_x || ~isvector(x_up) || length(x_up) ~= n_x
    error('Initial set bounds must be %dx1, got x_low=%s, x_up=%s', n_x, mat2str(size(x_low)), mat2str(size(x_up)));
end
if ~isvector(p_low) || length(p_low) ~= n_p || ~isvector(p_up) || length(p_up) ~= n_p
    error('Input bounds must be %dx1, got p_low=%s, p_up=%s', n_p, mat2str(size(p_low)), mat2str(size(p_up)));
end

% Define over-approximation methods
methods = 3; % 3: Continuous-time mixed-monotonicity
n_methods = length(methods);
succ_low_all_methods = NaN(n_x, n_t, n_methods); % For initial approach
succ_up_all_methods = NaN(n_x, n_t, n_methods);
succ_low_inc_methods = NaN(n_x, n_t, n_methods); % For incremental approach
succ_up_inc_methods = NaN(n_x, n_t, n_methods);

%% Call of the main over-approximation function
try
    if bool_discrete_time
        [succ_low, succ_up] = TIRA(t_init, x_low, x_up, p_low, p_up);
    else
        time_initial = zeros(1, n_methods);
        time_incremental = zeros(1, n_methods);

        % Initial approach: Compute reachable set from t_init to t_final in one step
        for m = 1:n_methods
            fprintf('Computing initial approach, method %d, time %.2f to %.2f, x_low=%s, x_up=%s\n', ...
                    methods(m), t_init, t_final, mat2str(x_low), mat2str(x_up));
            tStart = tic;
            [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
            time_initial(m) = toc(tStart); % Time only the single call
            if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                error('TIRA returned invalid dimensions for initial approach, method %d, time %.2f: succ_low=%s, succ_up=%s', ...
                      methods(m), t_final, mat2str(size(succ_low)), mat2str(size(succ_up)));
            end
            % Replicate the result across all time points
            succ_low_all_methods(:, :, m) = repmat(succ_low, 1, n_t);
            succ_up_all_methods(:, :, m) = repmat(succ_up, 1, n_t);
        end

        % Incremental approach: Compute reachable sets step-by-step
        for m = 1:n_methods
            succ_low_inc_methods(:,1,m) = x_low;
            succ_up_inc_methods(:,1,m) = x_up;
            for k = 1:n_t-1
                t_start = t_points(k);
                t_end = t_points(k+1);
                current_low = succ_low_inc_methods(:,k,m);
                current_up = succ_up_inc_methods(:,k,m);
                fprintf('Computing incremental approach, method %d, time %.2f to %.2f, x_low=%s, x_up=%s\n', ...
                        methods(m), t_start, t_end, mat2str(current_low), mat2str(current_up));
                tStart = tic;
                [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
                time_incremental(m) = time_incremental(m) + toc(tStart); % Accumulate time for each step
                if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                    error('TIRA returned invalid dimensions for incremental approach, method %d, time %.2f to %.2f: succ_low=%s, succ_up=%s', ...
                          methods(m), t_start, t_end, mat2str(size(succ_low)), mat2str(size(succ_up)));
                end
                succ_low_inc_methods(:,k+1,m) = succ_low;
                succ_up_inc_methods(:,k+1,m) = succ_up;
            end
        end
    end

    % Display timing results
    fprintf('\n=== Reachability Computation Times at t = %.2f (seconds) ===\n', t_final);
    for m = 1:n_methods
        fprintf('Initial Method %d (CT Mixed-Monotonicity): %.6f\n', methods(m), time_initial(m));
        fprintf('Incremental Method %d (CT Mixed-Monotonicity): %.6f\n', methods(m), time_incremental(m));
    end
catch e
    fprintf('Error in reachability computation: %s\n', e.message);
    rethrow(e);
end

%% Volume-based metric for the n-dimensional reachable-set OAs
% Following PJ: compute volume of the OA, then take the n-th root so the
% number is dimensionally comparable across benchmarks of different
% dimension (per-dim volume = volume^(1/n), or equivalently the equivalent
% box side length).
fprintf('\n=== Volume Metric for Final Reachable-Set OAs at t = %.2f ===\n', t_final);
fprintf('  Dimension n = %d\n', n_x);

% Container for the two approaches x one method we will report:
%   row 1: Single-step    (method 1 = CT-MM)
%   row 2: Incremental    (method 1 = CT-MM)
volume_per_dim_table = nan(2, n_methods);
volume_table         = nan(2, n_methods);

k = size(succ_low_all_methods, 2);   % last time step

for m = 1:n_methods
    % ---- Single-step OA (method m) ----
    widths_single = succ_up_all_methods(:, k, m) - succ_low_all_methods(:, k, m);
    if all(isfinite(widths_single)) && all(widths_single >= 0)
        vol_single = prod(widths_single);
        volume_table(1, m)         = vol_single;
        volume_per_dim_table(1, m) = vol_single^(1/n_x);
    end

    % ---- Incremental OA (method m) ----
    widths_inc = succ_up_inc_methods(:, k, m) - succ_low_inc_methods(:, k, m);
    if all(isfinite(widths_inc)) && all(widths_inc >= 0)
        vol_inc = prod(widths_inc);
        volume_table(2, m)         = vol_inc;
        volume_per_dim_table(2, m) = vol_inc^(1/n_x);
    end
end

approach_names = {'Single-step', 'Incremental'};
method_names   = {'CT Mixed-Monotonicity'};
for m = 1:n_methods
    fprintf('\n[%s]\n', method_names{m});
    for a = 1:2
        fprintf('  %-15s  Volume = %.6e   Volume^(1/n) = %.6f\n', ...
                approach_names{a}, volume_table(a, m), volume_per_dim_table(a, m));
    end
end

%% Compute successors from random initial states and disturbances
try
    sample_succ_number = 1000;
    log = logger.get_logger();

    % Extract ODE solver to use
    run('Solver_parameters.m');
    ode_solver = parameters.ode_solver;
    ode_options = parameters.ode_options;

    log.info('Compute successors from %d random initial states and parameters ...\n', sample_succ_number)
    t_start = tic;
    rand_succ = NaN(n_x,sample_succ_number);
    for i = 1:sample_succ_number
            x0 = x_low + rand(n_x,1).*(x_up-x_low);
            p = p_low + rand(n_p,1).*(p_up-p_low);
            if bool_discrete_time
                rand_succ(:,i) = System_description(t_init, x0, p);
            else
                [~, x_traj] = ode_solver(@(t, x) System_description(t, x, p), [t_init t_final], x0, ode_options);
                rand_succ(:, i) = x_traj(end, :)';
            end
    end
    t_rand_succs = toc(t_start);
    log.runtime('Time to generate random successors: %f seconds\n', t_rand_succs);
catch e
    fprintf('Error in random successor computation: %s\n', e.message);
    rethrow(e);
end

%% Plot the reachable set and over-approximations
try
    % Create timestamped directory for all outputs
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    save_dir = fullfile(pwd, sprintf('results_%s_%s', system_name, timestamp));
    if ~exist(save_dir, 'dir')
        [status, msg] = mkdir(save_dir);
        if status ~= 1
            warning('Failed to create directory %s: %s', save_dir, msg);
        else
            fprintf('Created directory: %s\n', save_dir);
        end
    end

    % Define plot pairs based on system choice
    plot_pairs = [];
    if system_choice == 14
        plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
    elseif system_choice == 15
        plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
    end
    colors = [0 0 1; 0 1 1]; % Dark Blue (Single-step), Light Blue/Cyan (Incremental)
    method_names = {'CT Mixed-Monotonicity'};

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
        tightness_metrics = zeros(1, 2); % [Single-step, Incremental]

        if plot_last_only
            k = size(succ_low_all_methods, 2); % Last time step

            % Mixed-Monotonicity (Single-step) - Dark Blue Solid
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
                tightness_metrics(1) = (width_single * height_single) / area_actual;
            else
                warning('Mixed-Monotonicity (Single-step) has invalid bounds. Using actual range.');
                h_single1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
                                   'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'FaceAlpha', 0.1, 'LineStyle', ':');
                h_proxy_single1 = plot(NaN, NaN, ':', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
                tightness_metrics(1) = 1.0;
            end

            % Mixed-Monotonicity (Incremental) - Light Blue Solid
            x_min_inc = succ_low_inc_methods(idx1, k, 1); % Method 1
            y_min_inc = succ_low_inc_methods(idx2, k, 1);
            width_inc = succ_up_inc_methods(idx1, k, 1) - x_min_inc;
            height_inc = succ_up_inc_methods(idx2, k, 1) - y_min_inc;
            fprintf('Mixed-Monotonicity (Incremental) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    x_min_inc, y_min_inc, width_inc, height_inc);
            if all(isfinite([x_min_inc, y_min_inc, width_inc, height_inc])) && width_inc >= 0 && height_inc >= 0
                h_inc1 = rectangle('Position', [x_min_inc, y_min_inc, max(width_inc, 0.001), max(height_inc, 0.001)], ...
                                 'EdgeColor', colors(2,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
                h_proxy_inc1 = plot(NaN, NaN, '-', 'Color', colors(2,:), 'LineWidth', 1.5, 'Visible', 'off');
                tightness_metrics(2) = (width_inc * height_inc) / area_actual;
            else
                warning('Skipping Mixed-Monotonicity (Incremental) due to invalid parameters');
            end
        end

        % Adjust axis to include all bounds
        x_min_all = min([x_min_actual, x_min_single, x_min_inc]);
        x_max_all = max([x_max_actual, x_min_single + width_single, x_min_inc + width_inc]);
        y_min_all = min([y_min_actual, y_min_single, y_min_inc]);
        y_max_all = max([y_max_actual, y_min_single + height_single, y_min_inc + height_inc]);
        x_range = x_max_all - x_min_all;
        y_range = y_max_all - y_min_all;
        buffer = 0.2; % padding
        axis([x_min_all - buffer*x_range, x_max_all + buffer*x_range, y_min_all - buffer*y_range, y_max_all + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add single legend with correct method assignments
        all_proxy_handles = [h_proxy_single1, h_proxy_inc1];
        all_legend_labels = {'Mixed-Monotonicity (Single-step)', 'Mixed-Monotonicity (Incremental)'};
        lgd = legend([plot(NaN, NaN, 'k.', 'MarkerSize', 5), all_proxy_handles], ...
                     ['Sampled points', all_legend_labels], 'Location', 'southeast');
        set(lgd, 'FontSize', 5, 'FontWeight', 'bold');

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

        % fprintf('Per-projection tightness (legacy diagnostic) for x_%d vs x_%d:\n', idx1, idx2);
        % fprintf('Mixed-Monotonicity (Single-step): %.2f\n', tightness_metrics(1));
        % fprintf('Mixed-Monotonicity (Incremental): %.2f\n', tightness_metrics(2));
    end
catch e
    fprintf('Error in plotting: %s\n', e.message);
    rethrow(e);
end

% Display timing results
fprintf('Computation Times (seconds) for all plots:\n');
for m = 1:n_methods
    fprintf('Initial Method %s: %.6f\n', method_names{m}, time_initial(m));
    fprintf('Incremental Method %s: %.6f\n', method_names{m}, time_incremental(m));
end

%% Save the reachable sets and random successors
try
    timestamp = datestr(now, 'yyyy-mm-dd');
    filename_base = sprintf('reachability_data_sys_TIRA%d_all_%s', system_choice, timestamp);
    save(fullfile(save_dir, [filename_base '.mat']), 'succ_low_all_methods', 'succ_up_all_methods', 'succ_low_inc_methods', 'succ_up_inc_methods', ...
         'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', ...
         'time_initial', 'time_incremental', ...
         'volume_table', 'volume_per_dim_table');
    disp(['Data saved to ' fullfile(save_dir, [filename_base '.mat'])]);
catch e
    fprintf('Error in saving data: %s\n', e.message);
    rethrow(e);
end

%------------- END OF CODE --------------