% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 23 October 2025
% Last update: 15 March 2026
% Last revision: October 2025

%------------- BEGIN CODE --------------

function reachable_tube_bounds = fpa_CORA_RT(params)
% fpa_CORA_RT.m Performs CORA reachability analysis for FPA (CTRNN) system
% and returns overall reachable tube bounds (lower and upper).
%
% Input:
%   params - struct with:
%     .timeStep       : time step for discretization
%     .tFinal         : final time
%     .taylorTerms    : Taylor terms for nonlinear approximation
%     .zonotopeOrder  : zonotope reduction order
%     .alg            : 'lin' or 'con'
%     .tensorOrder    : tensor order for nonlinear dynamics
%     .x0             : initial state center [5x1]
%     .Initial_radius : initial uncertainty radius
%
% Output:
%   reachable_tube_bounds - struct with:
%     .lb : [5x1] overall lower bound
%     .ub : [5x1] overall upper bound

% === Validate required fields ===
required_fields = {'timeStep', 'tFinal', 'taylorTerms', 'zonotopeOrder', ...
                   'alg', 'tensorOrder', 'x0', 'Initial_radius'};
for i = 1:length(required_fields)
    if ~isfield(params, required_fields{i})
        error('Missing required parameter: params.%s', required_fields{i});
    end
end

% === Extract parameters ===
timeStep       = params.timeStep;
tFinal         = params.tFinal;
taylorTerms    = params.taylorTerms;
zonotopeOrder  = params.zonotopeOrder;
alg            = params.alg;
tensorOrder    = params.tensorOrder;
x0             = params.x0;
Initial_radius = params.Initial_radius;

% === CORA options ===
options.timeStep      = timeStep;
options.taylorTerms   = taylorTerms;
options.zonotopeOrder = zonotopeOrder;
options.alg           = alg;
options.tensorOrder   = tensorOrder;

% === Initial set ===
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;
R0_initial = interval(lb, ub);
U = zonotope([0, 0]);  % no input

% === Reachability parameters ===
reach_params.R0       = zonotope(R0_initial);
reach_params.U        = U;
reach_params.timeStep = timeStep;
reach_params.tFinal   = tFinal;

% === Define system ===
sys = nonlinearSys(@CTRNN_FPA, 5, 1);  % 5 states, 1 input (unused)

% === Perform reachability ===
% fprintf('Performing CORA reachability analysis for FPA (5D)...\n');      % Commented for clearner print flow in the command window
% fprintf('  Initial center x0 = [%s]\n', num2str(x0', '%.6f '));          % Commented for clearner print flow in the command window
% fprintf('  Initial radius = %.4f\n', Initial_radius);                    % Commented for clearner print flow in the command window
% fprintf('  Time interval = [0, %.2f], step = %.5f\n', tFinal, timeStep); % Commented for clearner print flow in the command window
tic;

R = reach(sys, reach_params, options);

reachTime = toc;
fprintf('Reachability analysis completed in %.4f seconds\n', reachTime);

% === Validate output structure ===
try
    if ~isfield(R.timePoint, 'set') || isempty(R.timePoint.set)
        error('R.timePoint.set is missing or empty.');
    end
catch
    error('Reachability analysis failed: R.timePoint.set not accessible.');
end

% === Compute overall tube bounds ===
dim = length(x0);
min_x_overall = inf(dim, 1);
max_x_overall = -inf(dim, 1);

steps = length(R.timePoint.set);
fprintf('  Processing %d time steps for tube bounds...\n', steps);

for i = 1:steps
    current_zonotope = R.timePoint.set{i};
    current_interval = interval(current_zonotope);
    lb_current = current_interval.inf;
    ub_current = current_interval.sup;
    
    min_x_overall = min(min_x_overall, lb_current);
    max_x_overall = max(max_x_overall, ub_current);
end

% === Store and return ===
reachable_tube_bounds.lb = min_x_overall;
reachable_tube_bounds.ub = max_x_overall;

% === Display final bounds ===
% fprintf('\n=== Overall Reachable Tube Bounds (FPA) ===\n');                   % Commented for clearner print flow in the command window
% fprintf('Lower bounds: [%s]\n', num2str(reachable_tube_bounds.lb', '%.6f ')); % Commented for clearner print flow in the command window
% fprintf('Upper bounds: [%s]\n', num2str(reachable_tube_bounds.ub', '%.6f ')); % Commented for clearner print flow in the command window

end


%------------- END OF CODE --------------