%% To get the Reachable Tube Bounds



%% 1st approach (single-step)

% %------------- BEGIN 1st approach CODE --------------


% Change default options
options.timeStep = 0.01;         % Time step for discretization
options.taylorTerms = 2;         % Taylor terms for nonlinear approximation
options.zonotopeOrder = 20;      % Zonotope order for set representation
options.alg = 'lin';             % Linear approximation
options.tensorOrder = 2;         % Tensor order for nonlinear dynamics

% Initial states
x0 = [2.0; 0.0];
Initial_radius = 0.2;
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

R0_initial = interval(lb, ub);   % Initial set as an interval
U = zonotope([0, 0]);            % No input

% Parameters for reachability
params.R0 = zonotope(R0_initial); % Convert initial set to zonotope
params.U = U;                    % Input set
params.timeStep=0.01;
params.tFinal = 2;               % Final time

% Define time phases (for simulation of sampled points)
initial_time = 0;
final_time = 2;

% Define the nonlinear system
sys = nonlinearSys(@spiral_non, 2, 1); % Ensure @spiral_non is defined

tic; % Timer for the full reachability analysis

% Perform reachability analysis for the full set (t=0 to t=2)
R = reach(sys, params, options);

% Stop the timer and display computation time for the full set
fullReachTime = toc;
disp(['Full reachability analysis computation time: ' num2str(fullReachTime) ' seconds']);

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

% Plot reachable sets
figure;
hold on;
for j = 1:steps
    plot(R.timePoint.set{j}, [1, 2], 'r');
end


% Generate 1000 sampled black points
num_samples = 1000;
final_states_sampled = zeros(2, num_samples); % 2 dimensions for x and y

% Define the ODE solver for simulation (assuming 'ode45' is available in MATLAB)
ode_solver = @ode45;

% Simulate for each sampled initial point
for i = 1:num_samples
    % Sample initial state within the initial set R0_initial
    x0_sample = lb + rand(2,1) .* (ub - lb); 

    % Simulate the system from initial_time to final_time
    [~, x_traj] = ode_solver(@(t, x) spiral_non(x, []), [initial_time final_time], x0_sample);
    final_states_sampled(:,i) = x_traj(end,:)';
end

% Plot the sampled final states as black points
plot(final_states_sampled(1,:), final_states_sampled(2,:), 'k.', 'MarkerSize', 8);

% Plot final reachable set (after sampled points to ensure visibility)
final_reachable_set = R.timePoint.set{end};
plot(final_reachable_set, [1, 2], 'FaceColor', 'r', 'FaceAlpha', 0.3); % Added FaceAlpha for transparency

% Figure
grid on;
xlabel('y_{1}');
ylabel('y_{2}');
hold off;

% Compute and display lower and upper bounds for each reachable set
min_x_overall = inf(2, 1); % For overall reachable tube bounds
max_x_overall = -inf(2, 1); % For overall reachable tube bounds

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

% Tightness Metric (Area Ratio) Calculation

% Define 2D projections (matching TIRA - assuming 2D system, so only one projection)
dim_pairs = [1 2]; % x1-x2
projections = {"x1-x2"};

nnv_bounding_areas = zeros(size(dim_pairs,1),1);
area_actual = zeros(size(dim_pairs,1),1);

% Get the bounding box of the final reachable set
final_reachable_set_interval = interval(final_reachable_set);
lb_final = final_reachable_set_interval.inf;
ub_final = final_reachable_set_interval.sup;

% Compute areas for each projection
for p = 1:size(dim_pairs,1)
    dim1 = dim_pairs(p,1);
    dim2 = dim_pairs(p,2);

    % Bounding box area (CORA Reachable Set)
    width = ub_final(dim1) - lb_final(dim1);
    height = ub_final(dim2) - lb_final(dim2);
    nnv_bounding_areas(p) = width * height;

    % Actual spread area (simulated final states)
    x_min = min(final_states_sampled(dim1, :));
    x_max = max(final_states_sampled(dim1, :));
    y_min = min(final_states_sampled(dim2, :));
    y_max = max(final_states_sampled(dim2, :));
    area_actual(p) = (x_max - x_min) * (y_max - y_min);
end

% Compute tightness ratios
nnv_tightness_ratios = nnv_bounding_areas ./ area_actual;

% Display results in TIRA-like format
fprintf("\nTightness Metric (Area Ratio) for CORA Reachability:\n");
for p = 1:size(dim_pairs,1)
    fprintf("Projection %s:\n", projections{p});
    fprintf("Method CORA: %.2f\n", nnv_tightness_ratios(p));
    fprintf("  Bounding Box Area = %.6f, Area Actual = %.6f\n", nnv_bounding_areas(p), area_actual(p));
end

% %------------- END OF 1st approach CODE --------------



%% 2nd approach (incremental step)

% %------------- BEGIN 2nd approach CODE --------------

% % Change default options
% options.timeStep = 0.01;         % Time step for discretization
% options.taylorTerms = 2;         % Taylor terms for nonlinear approximation
% options.zonotopeOrder = 20;      % Zonotope order for set representation
% options.alg = 'lin';             % Linear approximation
% options.tensorOrder = 2;         % Tensor order for nonlinear dynamics
% 
% % Initial states
% x0 = [2.0; 0.0];
% Initial_radius = 0.2;
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% R0_initial = interval(lb, ub);   % Initial set as an interval
% U = zonotope([0, 0]);            % No input
% 
% % Parameters for reachability
% params.U = U;                    % Input set
% params.timeStep=0.01;
% 
% % Define time phases
% initial_time = 0;
% intermediate_time = 1;
% final_time = 2;
% 
% % Define the nonlinear system
% sys = nonlinearSys(@spiral_non, 2, 1); % Ensure @spiral_non is defined
% 
% %% Phase 1: Reachability from t=0 to t=1
% params.R0 = zonotope(R0_initial);
% params.tFinal = intermediate_time;
% 
% tic; % Timer for Phase 1
% R_tube1 = reach(sys, params, options);
% time_phase1 = toc;
% disp(['Phase 1 computation time (t=0 to t=1): ' num2str(time_phase1) ' seconds']);
% 
% % Extract reachable set at intermediate_time (t=1)
% % Find the set corresponding to intermediate_time
% % Assuming time points are evenly spaced, find the index for intermediate_time
% steps_phase1 = length(R_tube1.timePoint.set);
% init_set_t1 = R_tube1.timePoint.set{steps_phase1};
% 
% %% Phase 2: Reachability from t=1 to t=2
% params.R0 = zonotope(init_set_t1); % Use the final set of Phase 1 as initial set for Phase 2
% params.tFinal = final_time - intermediate_time;
% 
% tic; % Timer for Phase 2
% R_tube2 = reach(sys, params, options);
% time_phase2 = toc;
% disp(['Phase 2 computation time (t=1 to t=2): ' num2str(time_phase2) ' seconds']);
% 
% % Plot reachable sets from Phase 1
% figure;
% hold on;
% for j = 1:steps_phase1
%     plot(R_tube1.timePoint.set{j}, [1, 2], 'r');
% end
% 
% % Plot reachable sets from Phase 2
% for j = 1:length(R_tube2.timePoint.set)
%     plot(R_tube2.timePoint.set{j}, [1, 2], 'r');
% end
% 
% % --- Start of new code for sampling and over-approximation ---
% 
% % Generate 1000 sampled black points
% num_samples = 1000;
% final_states_sampled = zeros(2, num_samples); % 2 dimensions for x and y
% 
% % Define the ODE solver for simulation (assuming 'ode45' is available in MATLAB)
% ode_solver = @ode45;
% 
% % Simulate for each sampled initial point
% for i = 1:num_samples
%     % Sample initial state within the initial set R0_initial
%     x0_sample = lb + rand(2,1) .* (ub - lb); 
% 
%     % Simulate the system from initial_time to final_time
%     [~, x_traj] = ode_solver(@(t, x) spiral_non(x, []), [initial_time final_time], x0_sample);
%     final_states_sampled(:,i) = x_traj(end,:)';
% end
% 
% % Plot the sampled final states as black points
% plot(final_states_sampled(1,:), final_states_sampled(2,:), 'k.', 'MarkerSize', 8);
% 
% % Plot final reachable set (after sampled points to ensure visibility)
% final_reachable_set = R_tube2.timePoint.set{end};
% plot(final_reachable_set, [1, 2], 'FaceColor', 'r', 'FaceAlpha', 0.3); % Added FaceAlpha for transparency
%
% 
% % Formatting
% grid on;
% xlabel('y_{1}');
% ylabel('y_{2}');
% hold off;
% 
% %% Compute and display lower and upper bounds for each reachable set
% min_x_overall = inf(2, 1); % For overall reachable tube bounds
% max_x_overall = -inf(2, 1); % For overall reachable tube bounds
% 
% fprintf('\nLower and Upper Bounds for Each Reachable Set (Phase 1):\n');
% for i = 1:length(R_tube1.timePoint.set)
%     current_zonotope = R_tube1.timePoint.set{i};
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
% fprintf('\nLower and Upper Bounds for Each Reachable Set (Phase 2):\n');
% for i = 1:length(R_tube2.timePoint.set)
%     current_zonotope = R_tube2.timePoint.set{i};
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
% fprintf('\nOverall Reachable Tube Bounds (Combined Phases):\n');
% fprintf('Lower bounds: [%s]\n', num2str(min_x_overall'));
% fprintf('Upper bounds: [%s]\n', num2str(max_x_overall'));
% 
% %% Tightness Metric (Area Ratio) Calculation
% 
% % Define 2D projections (matching TIRA - assuming 2D system, so only one projection)
% dim_pairs = [1 2]; % x1-x2
% projections = {"x1-x2"};
% 
% nnv_bounding_areas = zeros(size(dim_pairs,1),1);
% area_actual = zeros(size(dim_pairs,1),1);
% 
% % Get the bounding box of the final reachable set
% final_reachable_set_interval = interval(final_reachable_set);
% lb_final = final_reachable_set_interval.inf;
% ub_final = final_reachable_set_interval.sup;
% 
% % Compute areas for each projection
% for p = 1:size(dim_pairs,1)
%     dim1 = dim_pairs(p,1);
%     dim2 = dim_pairs(p,2);
% 
%     % Bounding box area (CORA Reachable Set)
%     width = ub_final(dim1) - lb_final(dim1);
%     height = ub_final(dim2) - lb_final(dim2);
%     nnv_bounding_areas(p) = width * height;
% 
%     % Actual spread area (simulated final states)
%     x_min = min(final_states_sampled(dim1, :));
%     x_max = max(final_states_sampled(dim1, :));
%     y_min = min(final_states_sampled(dim2, :));
%     y_max = max(final_states_sampled(dim2, :));
%     area_actual(p) = (x_max - x_min) * (y_max - y_min);
% end
% 
% % Compute tightness ratios
% nnv_tightness_ratios = nnv_bounding_areas ./ area_actual;
% 
% % Display results in TIRA-like format
% fprintf("\nTightness Metric (Area Ratio) for CORA Reachability:\n");
% for p = 1:size(dim_pairs,1)
%     fprintf("Projection %s:\n", projections{p});
%     fprintf("Method CORA: %.2f\n", nnv_tightness_ratios(p));
%     fprintf("  Bounding Box Area = %.6f, Area Actual = %.6f\n", nnv_bounding_areas(p), area_actual(p));
% end

% %------------- END OF 2nd approach CODE --------------