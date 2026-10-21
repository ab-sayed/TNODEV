% Paper:
%   Mixed Monotonicity Reachability Analysis of Neural ODE: A Trade-Off Between Tightness and Efficiency
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 11th of August 2025
% Last update: 30th of May 2026
% Last revision: 30th of May 2026


%------------- BEGIN CODE --------------

% User switch for plotting mode: 'full' for all reachable sets, 'final' for last set only
plot_mode = 'final'; % 'full' or 'final'

dim = 5;

% Change default options
options.timeStep    = 2 / 999;     % ~1000 time points over t in [0,1] --> This should be 0.05 to match with the 2 other tools
options.taylorTerms = 3;
options.zonotopeOrder = 5;         % higher may crash for this model
options.alg = 'lin';
options.tensorOrder = 2;

% Initial states (5D)
x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';  % CTRNN_FPA nominal
Initial_radius = 0.01;
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

R0 = interval(lb, ub);
U  = zonotope([0, 0]);

params.R0     = zonotope(R0);
params.U      = U;
params.timeStep = options.timeStep;
params.tFinal   = 2;  % 1 or 2 based on preference and results from TIRA and NNV 2.0

% Verify time steps (target = 1000)
steps = floor(params.tFinal / params.timeStep) + 1;
if steps ~= 1000
    warning('Expected 1000 time points, computed %d. Adjusting timeStep.', steps);
    params.timeStep = params.tFinal / 999;
    options.timeStep = params.timeStep;
    steps = 1000;
end
disp(['Number of time points: ' num2str(steps)]);

% Define nonlinear system
% Make sure CTRNN_FPA(x,u) is on path and returns xdot for CORA's nonlinearSys wrapper.
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

%% === Ground-truth sampling (1000 points) ===
% NOTE: currently unused downstream (FPA figures dropped, tightness metric
% commented out). Retained for optional inspection; delete to save runtime.
num_samples = 1000;
rand_succ = zeros(dim, num_samples);
for i = 1:num_samples
    x0_sample = lb + rand(dim,1) .* (ub - lb);
    [~, x_traj] = ode45(@(t, x) CTRNN_FPA(x, []), [0 params.tFinal], x0_sample);
    rand_succ(:,i) = x_traj(end,:)';
end

%% === Volume-based metric for the full reachable-set OA ===
% Following PJ: report the n-th root of the volume so the number is
% comparable across benchmarks of different dimension.
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

    % (b) Box-hull volume of the same zonotope, for apples-to-apples
    %     comparison with TNODEV's interval OA
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