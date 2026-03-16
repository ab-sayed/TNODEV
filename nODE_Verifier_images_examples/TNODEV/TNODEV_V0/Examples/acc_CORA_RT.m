% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 10 March 2026
% Last update: 15 March 2026 
% Last revision: XX XX 2026

%------------- BEGIN CODE --------------
function reachable_tube_bounds = acc_CORA_RT(params)
% acc_CORA_RT.m Performs CORA reachability analysis for ACC Neural ODE system
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
%     .x0             : initial state center [8x1]
%     .Initial_radius : initial uncertainty radius (scalar or [8x1] vector)
%
% Output:
%   reachable_tube_bounds - struct with:
%     .lb : [8x1] overall lower bound
%     .ub : [8x1] overall upper bound

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
% Note: If Initial_radius is a vector, it allows for asymmetric bounding boxes
% (like the exact NNV benchmark). If scalar, it applies uniform uncertainty.
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;
R0_initial = interval(lb, ub);
U = zonotope([0, 0]);  % no input required for autonomous ACC system

% === Reachability parameters ===
reach_params.R0       = zonotope(R0_initial);
reach_params.U        = U;
reach_params.timeStep = timeStep;
reach_params.tFinal   = tFinal;

% === Define system ===
% The ACC system has 8 states. 1 dummy input.
sys = nonlinearSys(@acc_node_cora, 8, 1); 

% === Load Global Weights (Required before CORA evaluates the system) ===
global w1_acc b1_acc w2_acc b2_acc
if isempty(w1_acc)
    fprintf('Loading Neural ODE weights for CORA...\n');
    pl = load("plant_3rd_order_tanh.mat");
    w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
    b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
    w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
    b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
    b1_acc = b1_acc(:); 
    b2_acc = b2_acc(:);
end

% === Perform reachability ===
% fprintf('Performing CORA reachability analysis for ACC (8D)...\n');      % Commented for clearner print flow in the command window
% fprintf('  Initial center x0 = [%s]\n', num2str(x0', '%.4f '));          % Commented for clearner print flow in the command window
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
dim = length(x0); % 8
min_x_overall = inf(dim, 1);
max_x_overall = -inf(dim, 1);

steps = length(R.timePoint.set);
fprintf('  Processing %d time steps for tube bounds...\n', steps);

% CORA sometimes stores the initial set in index 1, so we evaluate all
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
% fprintf('\n=== Overall Reachable Tube Bounds (ACC) ===\n');                   % Commented for clearner print flow in the command window
% fprintf('Lower bounds: [%s]\n', num2str(reachable_tube_bounds.lb', '%.4f ')); % Commented for clearner print flow in the command window
% fprintf('Upper bounds: [%s]\n', num2str(reachable_tube_bounds.ub', '%.4f ')); % Commented for clearner print flow in the command window

end
%------------- END OF CODE --------------