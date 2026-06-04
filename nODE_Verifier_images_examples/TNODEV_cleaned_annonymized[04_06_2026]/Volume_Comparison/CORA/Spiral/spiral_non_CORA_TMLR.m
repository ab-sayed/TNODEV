
%------------- BEGIN CODE --------------

plot_mode = 'final'; % Options: 'final' or 'full'

dim = 2; % 2D for Spiral

% Change default options
options.timeStep = 2 / 999; 
options.taylorTerms = 2;
options.zonotopeOrder = 20;
options.alg = 'lin';
options.tensorOrder = 2;

% Initial states (2D)
x0 = [2.0; 0.0];
Initial_radius = 0.2;
lb = x0 - Initial_radius; 
ub = x0 + Initial_radius; 

R0 = interval(lb, ub);
U = zonotope([0, 0]);

params.R0 = zonotope(R0);
params.U = U;
params.timeStep = options.timeStep;
params.tFinal = 1;

% Verify time steps
steps = floor(params.tFinal / params.timeStep) + 1;
if steps ~= 1000
    warning('Expected 1000 time points, but computed %d. Adjusting timeStep.', steps);
    params.timeStep = params.tFinal / 999;
    options.timeStep = params.timeStep;
    steps = 1000;
end
disp(['Number of steps: ' num2str(steps)]);

% Define nonlinear system
sys = nonlinearSys(@spiral_non, 2, 1);

%% === Full reachable set OA ===
tic;
try
    R = reach(sys, params, options);
    disp('Full reachable set OA computed successfully.');
catch e
    disp('Error computing full reachable set OA:');
    disp(e.message);
    R = [];
end
fullSetTime = toc;
fprintf('Computation time for full reachable set OA: %.2f seconds\n', fullSetTime);

%% === Generate 1000 sampled black points ===
num_samples = 1000;
rand_succ = zeros(dim, num_samples);
for i = 1:num_samples
    x0_sample = lb + rand(dim,1) .* (ub - lb);
    [~, x_traj] = ode45(@(t, x) spiral_non(x, []), [0 params.tFinal], x0_sample);
    rand_succ(:,i) = x_traj(end,:)';
end

%% Plotting main figure with legend
figure('Name', 'Spiral Reachability Analysis - $x_1 - x_2$', 'Visible', 'on');
hold on;
plot_handles = [];
plot_labels = {};
if ~isempty(R) && strcmp(plot_mode, 'full')
    for j = 1:steps
        h = plot(R.timePoint.set{j}, [1, 2], 'r');
        if j == 1
            plot_handles(end+1) = h;
            plot_labels{end+1} = 'Full Reachable Set';
        end
    end
elseif ~isempty(R) && strcmp(plot_mode, 'final')
    h = plot(R.timePoint.set{end}, [1, 2], 'r', 'FaceAlpha', 0.3);
    plot_handles(end+1) = h;
    plot_labels{end+1} = 'Full Reachable Set (Final)';
end
h = plot(rand_succ(1,:), rand_succ(2,:), 'k.', 'MarkerSize', 8);
plot_handles(end+1) = h;
plot_labels{end+1} = 'Sampled Points';
legend(plot_handles, plot_labels, 'Location', 'best');
grid on;
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('$x_1$ vs. $x_2$', 'Interpreter', 'latex');
hold off;

%% Save main figure as EPS
[save_dir, save_name, ~] = fileparts(sprintf('reachability_data_sys_spiral_CORA_%s', datestr(now, 'yyyy-mm-dd_HH-MM-SS')));
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
main_fig_eps = fullfile(save_dir, [save_name '_main.eps']);
print('-depsc', main_fig_eps);
disp(['Main figure saved as ' main_fig_eps]);

%% Compute and display bounds for full reachable set
min_x_overall = inf(dim, 1);
max_x_overall = -inf(dim, 1);
if ~isempty(R)
    for i = 1:steps
        current_zonotope = R.timePoint.set{i};
        current_interval = interval(current_zonotope);
        lb_current = current_interval.inf;
        ub_current = current_interval.sup;
        min_x_overall = min(min_x_overall, lb_current);
        max_x_overall = max(max_x_overall, ub_current);
    end
    fprintf('\nOverall Reachable Tube Bounds:\n');
    fprintf('Lower bounds: [%s]\n', num2str(min_x_overall'));
    fprintf('Upper bounds: [%s]\n', num2str(max_x_overall'));
end

%% Volume-based metric for full reachable set (CORA)
if ~isempty(R)
    final_reachable_set = R.timePoint.set{end};

    % CORA exposes a volume function on zonotopes
    zonotope_volume_full = volume(final_reachable_set);

    % Compute n-th root of the volume to get a dimension-comparable metric
    % This is the "equivalent box side length" or "geometric mean width"
    volume_per_dim_full = zonotope_volume_full^(1/dim);

    fprintf('\nVolume Metric for Full Reachable Set (CORA):\n');
    fprintf('  Dimension n = %d\n', dim);
    fprintf('  Zonotope Volume = %.6e\n', zonotope_volume_full);
    fprintf('  Volume^(1/n) (per-dim metric) = %.6f\n', volume_per_dim_full);
end

%------------- END OF CODE --------------