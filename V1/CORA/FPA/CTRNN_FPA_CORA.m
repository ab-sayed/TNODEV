% Paper:
%   "Add the paper name here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 1st of August 2025
% Last update: 10th of August 2025
% Last revision: 

%------------- BEGIN CODE --------------

%% 5D volume

% % % Built in exact-volume method
% 
% % User switch for plotting mode: 'full' for all reachable sets, 'final' for last set only
% plot_mode = 'final'; % Options: 'full' or 'final'
% 
% dim = 5;
% dim1 = 1;
% dim2 = 2;
% 
% % Change default options
% options.timeStep = 0.01;
% options.taylorTerms = 3;
% options.zonotopeOrder = 10;
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% % Initial conditions
% x0 = [0, -0.58587, 0.8, 0.52323, 0.7]'; % From CTRNN_FPA
% Initial_radius = 0.01;
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% R0_initial = interval(lb, ub);
% U = zonotope([0, 0]);
% 
% params.R0 = zonotope(R0_initial);
% params.U = U;
% params.timeStep = 0.01; % Match options.timeStep
% params.tFinal = 2; % Adjusted to match simulation final time
% 
% % Verify that tFinal / timeStep is an integer
% if mod(params.tFinal / params.timeStep, 1) ~= 0
%     error('tFinal / timeStep must be an integer. Adjust tFinal or timeStep.');
% end
% 
% % Compute total steps for reference
% steps = floor(params.tFinal / params.timeStep) + 1;
% disp(['Total time points computed: ' num2str(steps)]);
% 
% % Define nonlinear system
% sys = nonlinearSys(@CTRNN_FPA, 5, 1); % CORA nonlinearSys class
% 
% tic; % Timer for full reachability analysis
% fprintf('Computing main reachable set R...\n');
% 
% % Perform reachability analysis
% R = reach(sys, params, options);
% 
% % Stop timer and display time
% fullReachTime = toc;
% disp(['Full reachability analysis computation time: ' num2str(fullReachTime) ' seconds']);
% 
% % Debug: Inspect R
% disp('Structure of R:');
% disp(R);
% disp('Contents of R.timePoint:');
% disp(R.timePoint);
% 
% % Check if timePoint and set are accessible
% try
%     temp = R.timePoint; % Access timePoint
%     if ~isfield(temp, 'set') % Check if set exists in the structure
%         error('R.timePoint.set is missing.');
%     end
% catch
%     error('Reachability analysis failed: R.timePoint is not accessible.');
% end
% 
% steps = length(R.timePoint.set); 
% disp(['Number of time points: ' num2str(steps)]);
% 
% % Plot reachable sets for each projection
% dim_pairs = [1 2; 3 4; 4 5];
% projections = {'x1-x2', 'x3-x4', 'x4-x5'};
% for p = 1:size(dim_pairs,1)
%     figure('Name', sprintf('CTRNN FPA Reachability Analysis - Projection %s', projections{p}), 'Visible', 'on');
%     hold on;
%     d1 = dim_pairs(p,1);
%     d2 = dim_pairs(p,2);
% 
%     % Plot based on user-selected mode
%     if strcmp(plot_mode, 'full')
%         for j = 1:steps
%             plot(R.timePoint.set{j}, [d1, d2], 'r');
%         end
%     elseif strcmp(plot_mode, 'final')
%         plot(R.timePoint.set{end}, [d1, d2], 'r');
%     else
%         error('Invalid plot_mode. Choose ''full'' or ''final''.');
%     end
% 
%     % Generate 1000 sampled black points (computed once, outside loop)
%     if p == 1  % Compute samples only once
%         num_samples = 1000;
%         final_states_sampled = zeros(dim, num_samples); % dim dimensions
% 
%         % Define the ODE solver for simulation (assuming 'ode45' is available in MATLAB)
%         ode_solver = @ode45;
% 
%         % Define time phases (for simulation of sampled points)
%         initial_time = 0;
%         final_time = 2;
% 
%         % Simulate for each sampled initial point
%         for i = 1:num_samples
%             % Sample initial state within the initial set R0_initial
%             x0_sample = lb + rand(dim,1) .* (ub - lb); 
% 
%             % Simulate the system from initial_time to final_time
%             [~, x_traj] = ode_solver(@(t, x) CTRNN_FPA(x, 0), [initial_time final_time], x0_sample);
%             final_states_sampled(:,i) = x_traj(end,:)';
%         end
%     end
% 
%     % Plot the sampled final states as black points
%     plot(final_states_sampled(d1,:), final_states_sampled(d2,:), 'k.', 'MarkerSize', 8);
% 
%     % Plot final reachable set (after sampled points to ensure visibility)
%     final_reachable_set = R.timePoint.set{end};
%     plot(final_reachable_set, [d1, d2], 'FaceColor', 'r', 'FaceAlpha', 0.3); % Added FaceAlpha for transparency
% 
%     % Figure
%     grid on;
%     xlabel(sprintf('x_{%d}', d1));
%     ylabel(sprintf('x_{%d}', d2));
%     hold off;
% end
% 
% % Compute and display lower and upper bounds for each reachable set
% min_x_overall = inf(dim, 1); % For overall reachable tube bounds
% max_x_overall = -inf(dim, 1); % For overall reachable tube bounds
% 
% fprintf('\nLower and Upper Bounds for Each Reachable Set:\n');
% for i = 1:length(R.timePoint.set)
%     current_zonotope = R.timePoint.set{i};
%     current_interval = interval(current_zonotope); % Convert zonotope to interval
%     lb_current = current_interval.inf;
%     ub_current = current_interval.sup;
% 
%     fprintf('Set %d: lb = [%s], ub = [%s]\n', i, num2str(lb_current'), num2str(ub_current'));
% 
%     min_x_overall = min(min_x_overall, lb_current);
%     max_x_overall = max(max_x_overall, ub_current);
% end
% 
% fprintf('\nOverall Reachable Tube Bounds:\n');
% fprintf('Lower bounds: [%s]\n', num2str(min_x_overall'));
% fprintf('Upper bounds: [%s]\n', num2str(max_x_overall'));
% 
% % Tightness Metric (Volume Ratio) Calculation
% % Compute 5D volume of the final reachable set
% final_reachable_set = R.timePoint.set{end};
% zonotope_volume = volume(final_reachable_set);
% 
% % Compute actual 5D volume from sampled points (bounding box)
% actual_volume = 1;
% for d = 1:dim
%     x_min = min(final_states_sampled(d, :));
%     x_max = max(final_states_sampled(d, :));
%     actual_volume = actual_volume * (x_max - x_min);
% end
% 
% % Compute tightness ratio
% tightness_ratio = zonotope_volume / actual_volume;
% 
% % Display results in TIRA-like format
% fprintf('\nTightness Metric (Volume Ratio) for CORA Reachability (5D):\n');
% fprintf('Method CORA: %.2f\n', tightness_ratio);
% fprintf('  Zonotope Volume = %.6f, Actual Volume = %.6f\n', zonotope_volume, actual_volume);

%% 5D volume of the final zonotope using its bounding box consistent with NNV2.0 and TIRA Adopted for 1000 time points 
% % User switch for plotting mode: 'full' for all reachable sets, 'final' for last set only
plot_mode = 'final'; % Options: 'full' or 'final'

dim = 5;

% Change default options
options.timeStep = 2 / 999; % Adjusted for 1000 time points (tFinal / (points - 1))
options.taylorTerms = 3;
options.zonotopeOrder = 10;
options.alg = 'lin';
options.tensorOrder = 2;

% Initial conditions
x0 = [0, -0.58587, 0.8, 0.52323, 0.7]'; % From CTRNN_FPA
Initial_radius = 0.01;
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

R0_initial = interval(lb, ub);
U = zonotope([0, 0]);

params.R0 = zonotope(R0_initial);
params.U = U;
params.timeStep = options.timeStep; % Match options.timeStep
params.tFinal = 2; % Adjusted to match simulation final time

% Verify that tFinal / timeStep is an integer number of intervals
steps = floor(params.tFinal / params.timeStep) + 1;
if steps ~= 1000
    warning('Expected 1000 time points, but computed %d. Adjusting timeStep.', steps);
    params.timeStep = params.tFinal / 999; % Force exactly 1000 points
    options.timeStep = params.timeStep;
    steps = 1000;
end
disp(['Total time points computed: ' num2str(steps)]);

% Define nonlinear system
sys = nonlinearSys(@CTRNN_FPA, 5, 1); % CORA nonlinearSys class

tic; % Timer for full reachability analysis
fprintf('Computing main reachable set R...\n');

% Perform reachability analysis
R = reach(sys, params, options);

% Stop timer and display time
reachTime = toc;
disp(['Reachability analysis computation time: ' num2str(reachTime) ' seconds']);

% Debug: Inspect R
disp('Structure of R:');
disp(R);
disp('Contents of R.timePoint:');
disp(R.timePoint);

% Check if timePoint and set are accessible
try
    temp = R.timePoint; % Access timePoint
    if ~isfield(temp, 'set') % Check if set exists in the structure
        error('R.timePoint.set is missing.');
    end
catch
    error('Reachability analysis failed: R.timePoint is not accessible.');
end

steps = length(R.timePoint.set); 
disp(['Number of time points: ' num2str(steps)]);

% Check if the number of time points matches expectation
if steps ~= 1000
    warning('Expected 1000 time points, but got %d. This may cause issues with plotting.', steps);
end

% Compute succ_low_all_methods and succ_up_all_methods for plotting compatibility
succ_low_all_methods = zeros(dim, steps, 1);
succ_up_all_methods = zeros(dim, steps, 1);
for k = 1:steps
    current_zonotope = R.timePoint.set{k};
    current_interval = interval(current_zonotope);
    succ_low_all_methods(:, k, 1) = current_interval.inf;
    succ_up_all_methods(:, k, 1) = current_interval.sup;
end

% Set other variables for save compatibility
t_points = (0:params.timeStep:params.tFinal)';
if length(t_points) ~= steps
    warning('t_points length (%d) does not match steps (%d). Adjusting t_points.', length(t_points), steps);
    t_points = linspace(0, params.tFinal, steps)';
end
n_x = dim;
n_methods = 1;
methods = [6]; % Assuming 6 corresponds to 'CORA Zonotope' in plotting function

% Generate 1000 sampled black points (computed once)
num_samples = 1000;
final_states_sampled = zeros(dim, num_samples); % dim dimensions

% Define the ODE solver for simulation (assuming 'ode45' is available in MATLAB)
ode_solver = @ode45;

% Define time phases (for simulation of sampled points)
initial_time = 0;
final_time = 2;

% Simulate for each sampled initial point
for i = 1:num_samples
    % Sample initial state within the initial set R0_initial
    x0_sample = lb + rand(dim,1) .* (ub - lb); 

    % Simulate the system from initial_time to final_time
    [~, x_traj] = ode_solver(@(t, x) CTRNN_FPA(x, 0), [initial_time final_time], x0_sample);
    final_states_sampled(:,i) = x_traj(end,:)';
end
rand_succ = final_states_sampled;

% Plot reachable sets for each projection
dim_pairs = [1 2; 3 4; 4 5];
projections = {'x1-x2', 'x3-x4', 'x4-x5'};
plot_pairs = dim_pairs; % For consistency
for p = 1:size(dim_pairs,1)
    figure('Name', sprintf('CTRNN FPA Reachability Analysis - Projection %s', projections{p}), 'Visible', 'on');
    hold on;
    d1 = dim_pairs(p,1);
    d2 = dim_pairs(p,2);

    % Plot based on user-selected mode
    if strcmp(plot_mode, 'full')
        for j = 1:steps
            plot(R.timePoint.set{j}, [d1, d2], 'r');
        end
    elseif strcmp(plot_mode, 'final')
        plot(R.timePoint.set{end}, [d1, d2], 'r');
    else
        error('Invalid plot_mode. Choose ''full'' or ''final''.');
    end

    % Plot the sampled final states as black points
    plot(final_states_sampled(d1,:), final_states_sampled(d2,:), 'k.', 'MarkerSize', 8);

    % Plot final reachable set (after sampled points to ensure visibility)
    final_reachable_set = R.timePoint.set{end};
    plot(final_reachable_set, [d1, d2], 'FaceColor', 'r', 'FaceAlpha', 0.3); % Added FaceAlpha for transparency

    % Figure
    grid on;
    xlabel(sprintf('x_{%d}', d1));
    ylabel(sprintf('x_{%d}', d2));
    hold off;
end

% Compute and display lower and upper bounds for each reachable set
min_x_overall = inf(dim, 1); % For overall reachable tube bounds
max_x_overall = -inf(dim, 1); % For overall reachable tube bounds

fprintf('\nLower and Upper Bounds for Each Reachable Set:\n');
for i = 1:length(R.timePoint.set)
    current_zonotope = R.timePoint.set{i};
    current_interval = interval(current_zonotope); % Convert zonotope to interval
    lb_current = current_interval.inf;
    ub_current = current_interval.sup;

    fprintf('Set %d: lb = [%s], ub = [%s]\n', i, num2str(lb_current'), num2str(ub_current'));

    min_x_overall = min(min_x_overall, lb_current);
    max_x_overall = max(max_x_overall, ub_current);
end

fprintf('\nOverall Reachable Tube Bounds:\n');
fprintf('Lower bounds: [%s]\n', num2str(min_x_overall'));
fprintf('Upper bounds: [%s]\n', num2str(max_x_overall'));

% Tightness Metric (Volume Ratio) Calculation
% Compute 5D volume of the final reachable set using bounding box
final_reachable_set = R.timePoint.set{end};
final_interval = interval(final_reachable_set);
lb_final = final_interval.inf;
ub_final = final_interval.sup;
zonotope_volume = prod(ub_final - lb_final); % Product of ranges across all dimensions

% Compute actual 5D volume from sampled points (bounding box)
actual_volume = 1;
for d = 1:dim
    x_min = min(final_states_sampled(d, :));
    x_max = max(final_states_sampled(d, :));
    actual_volume = actual_volume * (x_max - x_min);
end

% Compute tightness ratio
tightness_ratio = zonotope_volume / actual_volume;

% Display results in TIRA-like format
fprintf('\nTightness Metric (Volume Ratio) for CORA Reachability (5D):\n');
fprintf('Method CORA: %.2f\n', tightness_ratio);
fprintf('  Zonotope Bounding Box Volume = %.6f, Actual Volume = %.6f\n', zonotope_volume, actual_volume);

% Adapted tightness metric per plane to match TIRA
method_names = {'CORA Zonotope'};
tightness_metrics = zeros(1, size(plot_pairs, 1));
for pp = 1:size(plot_pairs, 1)
    idx1 = plot_pairs(pp, 1);
    idx2 = plot_pairs(pp, 2);

    % Calculate minimum bounding box for black dots
    x_min_actual = min(rand_succ(idx1, :));
    x_max_actual = max(rand_succ(idx1, :));
    y_min_actual = min(rand_succ(idx2, :));
    y_max_actual = max(rand_succ(idx2, :));
    area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);

    % Calculate tightness metric for the final time step (t_final)
    x_min_final = succ_low_all_methods(idx1, end, 1);
    y_min_final = succ_low_all_methods(idx2, end, 1);
    width_final = succ_up_all_methods(idx1, end, 1) - x_min_final;
    height_final = succ_up_all_methods(idx2, end, 1) - y_min_final;
    area_overapprox = width_final * height_final;
    tightness_metrics(pp) = area_overapprox / area_actual; % Ratio of areas

    % Display tightness metrics
    fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
            ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
    fprintf('Method %s: %.2f\n', method_names{1}, tightness_metrics(pp));
end

% Save the reachable sets and random successors in TIRA-compatible format
% Generate a unique timestamp for this run
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
filename_base = sprintf('reachability_data_sys14_CORA_%s', timestamp);
save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', '-v7.3');

disp(['Data saved to ' filename_base '.mat']);

%% 5D plot trial --> not working yet to save the %D zonotope [10-08]
% % User switch for plotting mode: 'full' for all reachable sets, 'final' for last set only
% plot_mode = 'final'; % Options: 'full' or 'final'
% 
% dim = 5;
% 
% % Change default options
% options.timeStep = 2 / 999; % Adjusted for 1000 time points (tFinal / (points - 1))
% options.taylorTerms = 3;
% options.zonotopeOrder = 10;
% options.alg = 'lin-adaptive';
% options.tensorOrder = 2;
% 
% % Initial conditions
% x0 = [0, -0.58587, 0.8, 0.52323, 0.7]'; % From CTRNN_FPA
% Initial_radius = 0.01;
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% R0_initial = interval(lb, ub);
% U = zonotope([0, 0]);
% 
% params.R0 = zonotope(R0_initial);
% params.U = U;
% params.timeStep = options.timeStep; % Match options.timeStep
% params.tFinal = 2; % Adjusted to match simulation final time
% 
% % Verify that tFinal / timeStep is an integer number of intervals
% steps = floor(params.tFinal / params.timeStep) + 1;
% if steps ~= 1000
%     warning('Expected 1000 time points, but computed %d. Adjusting timeStep.', steps);
%     params.timeStep = params.tFinal / 999; % Force exactly 1000 points
%     options.timeStep = params.timeStep;
%     steps = 1000;
% end
% disp(['Total time points computed: ' num2str(steps)]);
% 
% % Define nonlinear system
% sys = nonlinearSys(@CTRNN_FPA, 5, 1); % CORA nonlinearSys class
% 
% tic; % Timer for full reachability analysis
% fprintf('Computing main reachable set R...\n');
% 
% % Perform reachability analysis
% R = reach(sys, params, options);
% 
% % Stop timer and display time
% reachTime = toc;
% disp(['Reachability analysis computation time: ' num2str(reachTime) ' seconds']);
% 
% % Debug: Inspect R
% disp('Structure of R:');
% disp(R);
% disp('Contents of R.timePoint:');
% disp(R.timePoint);
% 
% % Check if timePoint and set are accessible
% try
%     temp = R.timePoint; % Access timePoint
%     if ~isfield(temp, 'set') % Check if set exists in the structure
%         error('R.timePoint.set is missing.');
%     end
% catch
%     error('Reachability analysis failed: R.timePoint is not accessible.');
% end
% 
% steps = length(R.timePoint.set); 
% disp(['Number of time points: ' num2str(steps)]);
% 
% % Check if the number of time points matches expectation
% if steps ~= 1000
%     warning('Expected 1000 time points, but got %d. This may cause issues with plotting.', steps);
% end
% 
% % Compute succ_low_all_methods and succ_up_all_methods for plotting compatibility
% succ_low_all_methods = zeros(dim, steps, 1);
% succ_up_all_methods = zeros(dim, steps, 1);
% for k = 1:steps
%     current_zonotope = R.timePoint.set{k};
%     current_interval = interval(current_zonotope);
%     succ_low_all_methods(:, k, 1) = current_interval.inf;
%     succ_up_all_methods(:, k, 1) = current_interval.sup;
% end
% 
% % Set other variables for save compatibility
% t_points = (0:params.timeStep:params.tFinal)';
% if length(t_points) ~= steps
%     warning('t_points length (%d) does not match steps (%d). Adjusting t_points.', length(t_points), steps);
%     t_points = linspace(0, params.tFinal, steps)';
% end
% n_x = dim;
% n_methods = 1;
% methods = [6]; % Assuming 6 corresponds to 'CORA Zonotope' in plotting function
% 
% % Generate 1000 sampled black points (computed once)
% num_samples = 1000;
% final_states_sampled = zeros(dim, num_samples); % dim dimensions
% 
% % Define the ODE solver for simulation (assuming 'ode45' is available in MATLAB)
% ode_solver = @ode45;
% 
% % Define time phases (for simulation of sampled points)
% initial_time = 0;
% final_time = 2;
% 
% % Simulate for each sampled initial point
% for i = 1:num_samples
%     % Sample initial state within the initial set R0_initial
%     x0_sample = lb + rand(dim,1) .* (ub - lb); 
% 
%     % Simulate the system from initial_time to final_time
%     [~, x_traj] = ode_solver(@(t, x) CTRNN_FPA(x, 0), [initial_time final_time], x0_sample);
%     final_states_sampled(:,i) = x_traj(end,:)';
% end
% rand_succ = final_states_sampled;
% 
% % Plot reachable sets for each projection
% dim_pairs = [1 2; 3 4; 4 5];
% projections = {'x1-x2', 'x3-x4', 'x4-x5'};
% plot_pairs = dim_pairs; % For consistency
% for p = 1:size(dim_pairs,1)
%     figure('Name', sprintf('CTRNN FPA Reachability Analysis - Projection %s', projections{p}), 'Visible', 'on');
%     hold on;
%     d1 = dim_pairs(p,1);
%     d2 = dim_pairs(p,2);
% 
%     % Plot based on user-selected mode
%     if strcmp(plot_mode, 'full')
%         for j = 1:steps
%             plot(R.timePoint.set{j}, [d1, d2], 'r');
%         end
%     elseif strcmp(plot_mode, 'final')
%         plot(R.timePoint.set{end}, [d1, d2], 'r');
%     else
%         error('Invalid plot_mode. Choose ''full'' or ''final''.');
%     end
% 
%     % Plot the sampled final states as black points
%     plot(final_states_sampled(d1,:), final_states_sampled(d2,:), 'k.', 'MarkerSize', 8);
% 
%     % Plot final reachable set (after sampled points to ensure visibility)
%     final_reachable_set = R.timePoint.set{end};
%     plot(final_reachable_set, [d1, d2], 'FaceColor', 'r', 'FaceAlpha', 0.3); % Added FaceAlpha for transparency
% 
%     % Figure
%     grid on;
%     xlabel(sprintf('x_{%d}', d1));
%     ylabel(sprintf('x_{%d}', d2));
%     hold off;
% end
% 
% % Compute and display lower and upper bounds for each reachable set
% min_x_overall = inf(dim, 1); % For overall reachable tube bounds
% max_x_overall = -inf(dim, 1); % For overall reachable tube bounds
% 
% fprintf('\nLower and Upper Bounds for Each Reachable Set:\n');
% for i = 1:length(R.timePoint.set)
%     current_zonotope = R.timePoint.set{i};
%     current_interval = interval(current_zonotope); % Convert zonotope to interval
%     lb_current = current_interval.inf;
%     ub_current = current_interval.sup;
% 
%     fprintf('Set %d: lb = [%s], ub = [%s]\n', i, num2str(lb_current'), num2str(ub_current'));
% 
%     min_x_overall = min(min_x_overall, lb_current);
%     max_x_overall = max(max_x_overall, ub_current);
% end
% 
% fprintf('\nOverall Reachable Tube Bounds:\n');
% fprintf('Lower bounds: [%s]\n', num2str(min_x_overall'));
% fprintf('Upper bounds: [%s]\n', num2str(max_x_overall'));
% 
% % Tightness Metric (Volume Ratio) Calculation
% % Compute 5D volume of the final reachable set using bounding box
% final_reachable_set = R.timePoint.set{end};
% final_interval = interval(final_reachable_set);
% lb_final = final_interval.inf;
% ub_final = final_interval.sup;
% zonotope_volume = prod(ub_final - lb_final); % Product of ranges across all dimensions
% 
% % Compute actual 5D volume from sampled points (bounding box)
% actual_volume = 1;
% for d = 1:dim
%     x_min = min(final_states_sampled(d, :));
%     x_max = max(final_states_sampled(d, :));
%     actual_volume = actual_volume * (x_max - x_min);
% end
% 
% % Compute tightness ratio
% tightness_ratio = zonotope_volume / actual_volume;
% 
% % Display results in TIRA-like format
% fprintf('\nTightness Metric (Volume Ratio) for CORA Reachability (5D):\n');
% fprintf('Method CORA: %.2f\n', tightness_ratio);
% fprintf('  Zonotope Bounding Box Volume = %.6f, Actual Volume = %.6f\n', zonotope_volume, actual_volume);
% 
% % Adapted tightness metric per plane to match TIRA
% method_names = {'CORA Zonotope'};
% tightness_metrics = zeros(1, size(plot_pairs, 1));
% for pp = 1:size(plot_pairs, 1)
%     idx1 = plot_pairs(pp, 1);
%     idx2 = plot_pairs(pp, 2);
% 
%     % Calculate minimum bounding box for black dots
%     x_min_actual = min(rand_succ(idx1, :));
%     x_max_actual = max(rand_succ(idx1, :));
%     y_min_actual = min(rand_succ(idx2, :));
%     y_max_actual = max(rand_succ(idx2, :));
%     area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);
% 
%     % Calculate tightness metric for the final time step (t_final)
%     x_min_final = succ_low_all_methods(idx1, end, 1);
%     y_min_final = succ_low_all_methods(idx2, end, 1);
%     width_final = succ_up_all_methods(idx1, end, 1) - x_min_final;
%     height_final = succ_up_all_methods(idx2, end, 1) - y_min_final;
%     area_overapprox = width_final * height_final;
%     tightness_metrics(pp) = area_overapprox / area_actual; % Ratio of areas
% 
%     % Display tightness metrics
%     fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
%             ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
%     fprintf('Method %s: %.2f\n', method_names{1}, tightness_metrics(pp));
% end
% 
% % Save the reachable sets and random successors in TIRA-compatible format
% % Generate a unique timestamp for this run
% timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% filename_base = sprintf('reachability_data_sys14_CORA_%s', timestamp);
% final_reachable_set = R.timePoint.set{end};
% save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', 'final_reachable_set', '-v7.3');
% 
% disp(['Data saved to ' filename_base '.mat']);


%------------- END OF CODE --------------