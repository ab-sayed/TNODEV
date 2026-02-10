%------------- BEGIN CODE V1.3 For MNIST  -------------- |--> % Modified for MNIST Example: Uses CTMM only and accepts A/B matrices.


function reachability_results = NODE_RA_New(params)
% NODE_RA_New - Reachability analysis for neural ODE using TIRA.
% This is a pure computational function that returns reachability bounds.

% Input:
%   params - struct with fields:
%       .system_choice  : 14 (FPA) or 15 (Spiral_non)
%       .x0             : Initial center of the set
%       .Initial_radius : Radius of the initial set
%       .t_final        : Final time for analysis
%       .timeStep       : Discretization step (not used by TIRA but good practice)
%
% Output:
%   reachability_results - struct containing the computed over-approximations.

% Updates:
%   - Handles System 16 (MNIST) by loading A/B globals.
%   - For System 16: Uses ONLY Continuous Time Mixed Monotonicity (Method 3)
%     and skips boundary analysis (too expensive for 676 dims).
%   - For Systems 14/15: Retains original behavior (Methods 3 & 4 + Boundary).

%% Initialization
% addpath('../OA_methods')
% addpath('../Utilities')  

if exist('../OA_methods', 'dir')
    addpath('../OA_methods');
end
if exist('../Utilities', 'dir')
    addpath('../Utilities');
end

% Initialize the global sensitivity method choice to a scalar NaN.
% This ensures that when TIRA's sub-functions check this global variable,
% they will always find a valid scalar value, preventing the '||' operator error.
%
global g_sensitivity_bounds_method
g_sensitivity_bounds_method = nan;


%% Definition of the systems and reachability problem

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
% n_p = length(p_low);      % Delter it if there's no use for it

% === DEFINE 4 METHODS (Single-step + Boundary) ===
run_boundary_analysis = true; % Default yes
methods = [3, 4];  % TIRA codes: 3 = CT MMM, 4 = SD MM

% MNIST example case
if system_choice == 16
    % --- MNIST CONFIGURATION ---
    % 1. Load Globals for Dynamics
    global Aout_global Bout_global
    Aout_global = get_from_params(params, 'A');
    Bout_global = get_from_params(params, 'B');
    
    % 2. Restrict to CTMM only
    methods = [3]; 
    
    % 3. Disable Boundary Analysis (too expensive for 676 dims)
    run_boundary_analysis = false;
end

n_methods_single_step = length(methods);  % 2
% n_methods_total = 4;  % 2 Single-step OAs + 2 Boundary OAs

% Preallocate for all 4 methods |--> For allocation, we assume max 4 slots but will only use what's in 'methods'
% succ_low_all_methods = NaN(n_x, 1, n_methods_total); % 1: RA @ Final time
% succ_up_all_methods = NaN(n_x, 1, n_methods_total);

succ_low_all_methods = NaN(n_x, 1, n_methods_single_step); 
succ_up_all_methods  = NaN(n_x, 1, n_methods_single_step);

%% Boundary Decompositions (Only if enabled)
num_splits = 0;
Initial_set_boundaries = {};
succ_low_boundary = {};
succ_up_boundary  = {};

if run_boundary_analysis
    num_splits = 2*n_x;
    Initial_set_boundaries = cell(num_splits, 1);
    succ_low_boundary = cell(num_splits, 1);
    succ_up_boundary  = cell(num_splits, 1);

    % Define the boundary sets for TIRA input
    for i = 1:n_x
        % Low bounds
        temp_low = x_low; temp_up = x_up; temp_low(i) = x_up(i);
        Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
        % Upper bounds
        temp_low = x_low; temp_up = x_up; temp_up(i) = x_low(i);
        Initial_set_boundaries{2*i} = interval(temp_low(:), temp_up(:));
    end
end

% num_splits = 2*n_x;
% Initial_set_boundaries = cell(num_splits, 1);
% succ_low_boundary = cell(num_splits, 1);
% succ_up_boundary  = cell(num_splits, 1);
% 
% % Define the boundary sets for TIRA input
% for i = 1:n_x
%     % Low bounds
%     temp_low = x_low; temp_up = x_up; temp_low(i) = x_up(i);
%     Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
%     % Upper bounds
%     temp_low = x_low; temp_up = x_up; temp_up(i) = x_low(i);
%     Initial_set_boundaries{2*i} = interval(temp_low(:), temp_up(:));
% end

%% Main Reachability Computation
try
    % 1. Single-step methods (Always run)
    for m = 1:n_methods_single_step
        [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
        succ_low_all_methods(:, 1, m) = succ_low;
        succ_up_all_methods(:, 1, m)  = succ_up;
    end

    % 2. Boundary methods (Conditional)
    if run_boundary_analysis
        for i = 1:num_splits
            succ_low_boundary{i} = NaN(n_x, 1, n_methods_single_step);  
            succ_up_boundary{i}  = NaN(n_x, 1, n_methods_single_step);
            for m = 1:n_methods_single_step
                [succ_low, succ_up] = TIRA([t_init, t_final], ...
                    Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
                succ_low_boundary{i}(:, 1, m) = succ_low;
                succ_up_boundary{i}(:, 1, m)  = succ_up;
             end
         end
    end
catch e
    fprintf('Error during TIRA computation in NODE_RA.m\n');
    rethrow(e);
end

% try
%     % Initialize the timing variables before the loops.
%     time_initial = zeros(1, n_methods_single_step);
%     time_boundary = zeros(num_splits, n_methods_single_step);
% 
%     % 1. Single-step methods
%     for m = 1:n_methods_single_step
%         [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
%         succ_low_all_methods(:, 1, m) = succ_low;
%         succ_up_all_methods(:, 1, m)  = succ_up;
%     end
% 
%     % 2. Boundary methods
%     for i = 1:num_splits
%         succ_low_boundary{i} = NaN(n_x, 1, n_methods_single_step);  
%         succ_up_boundary{i}  = NaN(n_x, 1, n_methods_single_step);
%         for m = 1:n_methods_single_step
%             [succ_low, succ_up] = TIRA([t_init, t_final], ...
%                 Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
%             succ_low_boundary{i}(:, 1, m) = succ_low;
%             succ_up_boundary{i}(:, 1, m)  = succ_up;
%          end
%      end
% catch e
%     fprintf('Error during TIRA computation in NODE_RA_New.m\n');
%     rethrow(e);
% end

%% Package Results --> # Commented for now the ones that aren't used but left incase we will need some of these O/Ps later
reachability_results = struct();
reachability_results.succ_low_all_methods = succ_low_all_methods;
reachability_results.succ_up_all_methods  = succ_up_all_methods;
reachability_results.succ_low_boundary    = succ_low_boundary;
reachability_results.succ_up_boundary     = succ_up_boundary;
% reachability_results.time_initial         = time_initial; 
% reachability_results.time_boundary        = time_boundary;
reachability_results.num_splits           = num_splits;
% reachability_results.system_choice        = system_choice;
% reachability_results.n_x                  = n_x;


%% Compute Intersection of All 4 Over-Approximations (NEW) 

% k = size(reachability_results.succ_low_all_methods, 2);
k = 1; % Index for the final time step

check_dims = n_x; 
% check_dims = size(reachability_results.succ_low_all_methods, 1); % Or determined from n_x

intersect_low = -Inf(check_dims,1);
intersect_up  =  Inf(check_dims,1);

if system_choice == 16
    % === MNIST SPECIFIC INTERSECTION ===
    % We only have 1 method (CTMM) and no boundaries.
    % The intersection is simply the result of that single method.
    intersect_low = succ_low_all_methods(:, 1, 1);
    intersect_up  = succ_up_all_methods(:, 1, 1);
    
else
    % === ORIGINAL INTERSECTION LOGIC (Systems 14 & 15) ===
    % Assumes methods = [3, 4] and boundary analysis exists.
    
    % Single-step methods (m=1 and m=2)
    for m = 1:n_methods_single_step 
        intersect_low = max(intersect_low, succ_low_all_methods(1:check_dims,k,m));
        intersect_up  = min(intersect_up,  succ_up_all_methods(1:check_dims,k,m));
    end

    % Boundary methods
    if run_boundary_analysis
        for m = 1:n_methods_single_step 
            Ls = Inf(check_dims, num_splits); 
            Us = -Inf(check_dims, num_splits);
            for i = 1:num_splits
                Ls(:,i) = succ_low_boundary{i}(1:check_dims,k,m);
                Us(:,i) = succ_up_boundary{i}(1:check_dims,k,m);
            end
            intersect_low = max(intersect_low, min(Ls,[],2));
            intersect_up  = min(intersect_up,  max(Us,[],2));
        end
    end
end

% % Single-step methods (m=1 and m=2)
% for m = 1:2 
%     intersect_low = max(intersect_low, reachability_results.succ_low_all_methods(1:check_dims,k,m));
%     intersect_up  = min(intersect_up,  reachability_results.succ_up_all_methods(1:check_dims,k,m));
% end
% 
% % Boundary methods (m=1 and m=2 of succ_low_boundary/succ_up_boundary)
% for m = 1:2 
%     % Ls and Us capture the lower/upper bounds across all splits for method m
%     Ls = Inf(check_dims, reachability_results.num_splits); 
%     Us = -Inf(check_dims, reachability_results.num_splits);
%     for i = 1:reachability_results.num_splits
%         Ls(:,i) = reachability_results.succ_low_boundary{i}(1:check_dims,k,m);
%         Us(:,i) = reachability_results.succ_up_boundary{i}(1:check_dims,k,m);
%     end
%     % The bound of the union is min(Ls) and max(Us). 
%     % The intersection of the *union* bounds is taken with the overall intersection.
%     intersect_low = max(intersect_low, min(Ls,[],2));
%     intersect_up  = min(intersect_up,  max(Us,[],2));
% end

% Store the intersection in the output structure --> The main Package Results
reachability_results.intersect_low = intersect_low;
reachability_results.intersect_up = intersect_up;

end

% =========================================================================
% === Helper Function ===
% =========================================================================
function val = get_from_params(params, field)
    % Safely extract a field from the params struct.
    if isfield(params, field) && ~isempty(params.(field))
        val = params.(field);
    else
        error('NODE_RA_New: Missing required parameter "%s".', field);
    end
end

%------------- END OF CODE V1.3  --------------