%% PART 1: CORA computation of Reachable Tube Bounds
% Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 21 October 2025
% Last revision: October 2025


function reachable_tube_bounds = spiral_non_CORA_RT(params)
% spiral_non_CORA_RT.m Performs CORA reachability analysis and returns overall reachable tube bounds
% Input:
%   params - struct containing:
%     .timeStep: Time step for discretization
%     .tFinal: Final time
%     .taylorTerms: Taylor terms for nonlinear approximation  
%     .zonotopeOrder: Zonotope order for set representation
%     .alg: Linear approximation method
%     .tensorOrder: Tensor order for nonlinear dynamics
%     .x0: Initial state center [2x1]
%     .Initial_radius: Initial uncertainty radius
% Output:
%   reachable_tube_bounds - struct with .lb and .ub for lower and upper reachable tube bounds

% Validate required parameters
required_fields = {'timeStep', 'tFinal', 'taylorTerms', 'zonotopeOrder', ...
                   'alg', 'tensorOrder', 'x0', 'Initial_radius'};
for i = 1:length(required_fields)
    if ~isfield(params, required_fields{i})
        error('Missing required parameter: params.%s', required_fields{i});
    end
end


% Extract parameters from input struct
timeStep = params.timeStep;
tFinal = params.tFinal;
taylorTerms = params.taylorTerms;
zonotopeOrder = params.zonotopeOrder;
alg = params.alg;
tensorOrder = params.tensorOrder;
x0 = params.x0;
Initial_radius = params.Initial_radius;

% Set CORA options
options.timeStep = timeStep;         % Time step for discretization
options.taylorTerms = taylorTerms;   % Taylor terms for nonlinear approximation
options.zonotopeOrder = zonotopeOrder; % Zonotope order for set representation
options.alg = alg;                   % Linear approximation
options.tensorOrder = tensorOrder;   % Tensor order for nonlinear dynamics

% Initial states
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

R0_initial = interval(lb, ub);       % Initial set as an interval
U = zonotope([0, 0]);                % No input

% Parameters for reachability
reach_params.R0 = zonotope(R0_initial);    % Convert initial set to zonotope
reach_params.U = U;                        % Input set
reach_params.timeStep = timeStep;
reach_params.tFinal = tFinal;              % Final time (from params)

% Define the nonlinear system
sys = nonlinearSys(@spiral_non, 2, 1); % Ensure @spiral_non is defined

fprintf('Performing CORA reachability analysis...\n');
fprintf('  Initial center x0 = [%s]\n', num2str(x0', '%.4f '));
fprintf('  Initial radius = %.4f\n', Initial_radius);
fprintf('  Time interval = [0, %.4f], step = %.4f\n', tFinal, timeStep);
tic; % Timer for the full reachability analysis

% Perform reachability analysis
R = reach(sys, reach_params, options);

% Stop the timer
fullReachTime = toc;
fprintf('Reachability analysis completed in %.4f seconds\n', fullReachTime);

% Check if timePoint and set are accessible
try
    temp = R.timePoint; % Access timePoint
    if ~isfield(temp, 'set') % Check if set exists in the structure
        error('R.timePoint.set is missing.');
    end
catch
    error('Reachability analysis failed: R.timePoint is not accessible.');
end

% Compute Overall Reachable Tube Bounds
min_x_overall = inf(2, 1); % For overall reachable tube bounds
max_x_overall = -inf(2, 1); % For overall reachable tube bounds

fprintf('Computing overall reachable tube bounds...\n');
steps = length(R.timePoint.set);
fprintf('  Processing %d time steps...\n', steps);

for i = 1:steps
    current_zonotope = R.timePoint.set{i};
    current_interval = interval(current_zonotope); % Convert zonotope to interval
    lb_current = current_interval.inf;
    ub_current = current_interval.sup;

    % Update overall bounds
    min_x_overall = min(min_x_overall, lb_current);
    max_x_overall = max(max_x_overall, ub_current);
end

% Store overall bounds
reachable_tube_bounds.lb = min_x_overall;
reachable_tube_bounds.ub = max_x_overall;

% Display results
fprintf('\n=== Overall Reachable Tube Bounds ===\n');
fprintf('Lower bounds: [%s]\n', num2str(reachable_tube_bounds.lb', '%.4f '));
fprintf('Upper bounds: [%s]\n', num2str(reachable_tube_bounds.ub', '%.4f '));

% fprintf('\nOverall reachable tube bounds computation completed.\n');

end