
%------------- BEGIN CODE --------------

plot_mode = 'final'; % 'full' or 'final'

dim = 5; % 5D for FPA

% Change default options
options.timeStep    = 2 / 999;    
options.taylorTerms = 3;
options.zonotopeOrder = 5;         
options.alg = 'lin';
options.tensorOrder = 2;

% Initial states (5D)
x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';  
Initial_radius = 0.01;
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

R0 = interval(lb, ub);
U  = zonotope([0, 0]);

params.R0     = zonotope(R0);
params.U      = U;
params.timeStep = options.timeStep;
params.tFinal   = 2; 

% Verify time steps 
steps = floor(params.tFinal / params.timeStep) + 1;
if steps ~= 1000
    warning('Expected 1000 time points, computed %d. Adjusting timeStep.', steps);
    params.timeStep = params.tFinal / 999;
    options.timeStep = params.timeStep;
    steps = 1000;
end
disp(['Number of time points: ' num2str(steps)]);

% Define nonlinear system
sys = nonlinearSys(@CTRNN_FPA, dim, 1);

%% === Full reachable set OA ===
full_start = tic;
try
    R = reach(sys, params, options);
    fullSetTime = toc(full_start);
    fprintf('Computation time (Full OA): %.3f s\n', fullSetTime);
catch e
    disp('Error computing Full OA:'); disp(e.message);
    R = []; fullSetTime = NaN;
end

%% === Generate 1000 sampled black points ===
num_samples = 1000;
rand_succ = zeros(dim, num_samples);
for i = 1:num_samples
    x0_sample = lb + rand(dim,1) .* (ub - lb);
    [~, x_traj] = ode45(@(t, x) CTRNN_FPA(x, []), [0 params.tFinal], x0_sample);
    rand_succ(:,i) = x_traj(end,:)';
end

%% === Volume-based metric for the full reachable-set OA ===
fprintf('\n=== Volume Metric for Final Reachable-Set OA at t = %.2f ===\n', params.tFinal);
fprintf('  Dimension n = %d\n', dim);

volume_full                 = NaN;
volume_per_dim_full         = NaN;
volume_full_boxhull         = NaN;
volume_per_dim_full_boxhull = NaN;

if ~isempty(R)
    final_zonotope = R.timePoint.set{end};

    % (a) True zonotope volume via CORA's built-in volume()
    try
        volume_full         = volume(final_zonotope);
        volume_per_dim_full = volume_full^(1/dim);
        fprintf('\n[Full Reachable Set - CORA zonotope volume]\n');
        fprintf('  Zonotope Volume       = %.6e\n', volume_full);
        fprintf('  Volume^(1/n)          = %.6f\n', volume_per_dim_full);
    catch e
        warning('CORA volume(zonotope) failed in 5D: %s', e.message);
    end

    % (b) Box-hull volume of the same zonotope
    final_interval = interval(final_zonotope);
    widths_full    = final_interval.sup - final_interval.inf;
    if all(isfinite(widths_full)) && all(widths_full >= 0)
        volume_full_boxhull         = prod(widths_full);
        volume_per_dim_full_boxhull = volume_full_boxhull^(1/dim);
        fprintf('\n[Full Reachable Set - box hull]\n');
        fprintf('  Box-Hull Volume       = %.6e\n', volume_full_boxhull);
        fprintf('  Volume^(1/n)          = %.6f\n', volume_per_dim_full_boxhull);
    end
end

%------------- END OF CODE --------------