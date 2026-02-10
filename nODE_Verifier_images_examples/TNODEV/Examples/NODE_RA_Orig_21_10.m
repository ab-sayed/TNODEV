% Paper:
%   Mixed Monotonicity Reachability Analysis of Neural ODE: A Trade-Off Between Tightness and Efficiency
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 17th of July 2025
% Last update: 20th of October 2025
% Last revision: 13th of October 2025


% % It should be like this as a function%%

% function results = NODE_RA(params)
% % NODE_RA  Mixed-monotonicity reachability analysis for Neural ODEs
% %
% %   results = NODE_RA(params)
% %
% %   INPUT:
% %       params.system_choice      : 14 (FPA) or 15 (Spiral_non)
% %       params.x0                 : initial center (vector)
% %       params.Initial_radius     : uncertainty radius
% %       params.t_final            : final time
% %       params.timeStep           : time discretization step
% %       params.plot_last_only     : true/false (plot only final set)
% %       params.sample_succ_number : number of random samples (default: 1000)
% %
% %   OUTPUT (struct):
% %       results.succ_low_all_methods, results.succ_up_all_methods
% %       results.succ_low_boundary, results.succ_up_boundary
% %       results.rand_succ, results.t_points
% %       results.time_initial, results.time_boundary
% %       results.save_dir
% %
% %   Requires: TIRA.m, UP_Jacobian_Bounds.m, System_description.m, Solver_parameters.m
% %   Uses globals: Jacobian_x_low, Jacobian_x_up (set by Verifier.m)
% 
% %% ------------- BEGIN CODE --------------
% 
% %% Default parameters
% if ~isfield(params, 'plot_last_only'), params.plot_last_only = true; end
% if ~isfield(params, 'sample_succ_number'), params.sample_succ_number = 1000; end
% 
% %% Initialization
% close all;
% bool_discrete_time = 0;
% addpath('../OA_methods')
% addpath('../Utilities')
% 
% %% System choice
% global system_choice
% system_choice = params.system_choice;
% 
% system_names = containers.Map({14, 15}, {'fpa', 'spiral_non'});
% if ~isKey(system_names, system_choice)
%     error('Unknown system_choice %d', system_choice);
% end
% system_name = system_names(system_choice);
% 
% %% Optional submethod
% global g_sensitivity_bounds_method
% g_sensitivity_bounds_method = NaN;
% 
% %% Problem definition
% switch system_choice
%     case 14  % FPA
%         x0 = params.x0;
%         t_final = params.t_final;
%         steps = params.timeStep;
%     case 15  % Spiral_non
%         x0 = params.x0;
%         t_final = params.t_final;
%         steps = params.timeStep;
% end
% 
% Initial_radius = params.Initial_radius;
% x_low = x0 - Initial_radius;
% x_up  = x0 + Initial_radius;
% 
% t_init = 0;
% t_points = t_init:steps:t_final;
% 
% p_low = zeros(length(x0),1);
% p_up  = zeros(length(x0),1);
% 
% n_x = length(x_low);
% n_p = length(p_low);
% n_t = length(t_points);
% 
% % Dimension checks
% if system_choice == 14 && (n_x ~= 5 || n_p ~= 5)
%     error('FPA: expected 5D, got n_x=%d', n_x);
% elseif system_choice == 15 && (n_x ~= 2 || n_p ~= 2)
%     error('Spiral_non: expected 2D, got n_x=%d', n_x);
% end
% 
% %% Methods
% methods = [3, 4];  % 3: CT MM, 4: CT sampled-data MM
% n_methods = length(methods);
% succ_low_all_methods = NaN(n_x, n_t, n_methods);
% succ_up_all_methods  = NaN(n_x, n_t, n_methods);
% 
% %% Boundary splits
% num_splits = 2*n_x;
% Initial_set_boundaries = cell(num_splits, 1);
% for i = 1:n_x
%     tmp = x_low; tmp(i) = x_up(i);
%     Initial_set_boundaries{2*i-1} = interval(tmp, x_up);
%     tmp = x_up;  tmp(i) = x_low(i);
%     Initial_set_boundaries{2*i}   = interval(x_low, tmp);
% end
% 
% %% TIRA reachability
% try
%     time_initial = zeros(1, n_methods);
%     time_boundary = zeros(num_splits, n_methods);
% 
%     % --- Single-step ---
%     for m = 1:n_methods
%         fprintf('NODE_RA: Single-step method %d (t=%.2f→%.2f)\n', methods(m), t_init, t_final);
%         tStart = tic;
%         [low, up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
%         time_initial(m) = toc(tStart);
%         succ_low_all_methods(:, :, m) = repmat(low, 1, n_t);
%         succ_up_all_methods(:, :, m)  = repmat(up , 1, n_t);
%     end
% 
%     % --- Boundary splits ---
%     succ_low_boundary = cell(num_splits, 1);
%     succ_up_boundary  = cell(num_splits, 1);
%     for i = 1:num_splits
%         succ_low_boundary{i} = NaN(n_x, n_t, n_methods);
%         succ_up_boundary{i}  = NaN(n_x, n_t, n_methods);
%         for m = 1:n_methods
%             if params.plot_last_only
%                 tStart = tic;
%                 [low, up] = TIRA([t_init, t_final], ...
%                     Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, ...
%                     p_low, p_up, methods(m));
%                 time_boundary(i,m) = toc(tStart);
%                 succ_low_boundary{i}(:, end, m) = low;
%                 succ_up_boundary{i}(:, end, m)  = up;
%             else
%                 for k = 1:n_t
%                     t_f = t_points(k);
%                     tStart = tic;
%                     [low, up] = TIRA([t_init, t_f], ...
%                         Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, ...
%                         p_low, p_up, methods(m));
%                     time_boundary(i,m) = time_boundary(i,m) + toc(tStart);
%                     succ_low_boundary{i}(:, k, m) = low;
%                     succ_up_boundary{i}(:, k, m)  = up;
%                 end
%             end
%         end
%     end
% 
% catch e
%     error('NODE_RA: TIRA failed: %s', e.message);
% end
% 
% %% Random successors
% try
%     run('Solver_parameters.m');
%     ode_solver = parameters.ode_solver;
%     ode_options = parameters.ode_options;
% 
%     sample_succ_number = params.sample_succ_number;
%     rand_succ = NaN(n_x, sample_succ_number);
%     for i = 1:sample_succ_number
%         x0i = x_low + rand(n_x,1).*(x_up-x_low);
%         p = p_low + rand(n_p,1).*(p_up-p_low);
%         [~, traj] = ode_solver(@(t,x) System_description(t,x,p), [t_init t_final], x0i, ode_options);
%         rand_succ(:,i) = traj(end,:)';
%     end
% catch e
%     error('NODE_RA: Random sampling failed: %s', e.message);
% end
% 
% %% Plotting & Saving
% timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% save_dir = fullfile(pwd, sprintf('results_%s_%s', system_name, timestamp));
% mkdir(save_dir);
% 
% plot_pairs = (system_choice==14)*[1 2;3 4;4 5] + (system_choice==15)*[1 2];
% method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data MM'};
% colors = [0 0 1; 1 0 0];  % blue, red
% 
% for p = 1:size(plot_pairs,1)
%     idx1 = plot_pairs(p,1); idx2 = plot_pairs(p,2);
%     k = params.plot_last_only * n_t + (~params.plot_last_only)*1;
% 
%     % --- Main reachability plot ---
%     figure('Name',sprintf('Reach x%d vs x%d',idx1,idx2)); hold on; grid on;
%     plot(rand_succ(idx1,:), rand_succ(idx2,:), 'k.', 'MarkerSize',5);
% 
%     % Actual bounding box
%     area_actual = (max(rand_succ(idx1,:))-min(rand_succ(idx1,:))) * ...
%                   (max(rand_succ(idx2,:))-min(rand_succ(idx2,:)));
% 
%     % Single-step
%     for m = 1:n_methods
%         x1 = succ_low_all_methods(idx1,k,m); x2 = succ_up_all_methods(idx1,k,m);
%         y1 = succ_low_all_methods(idx2,k,m); y2 = succ_up_all_methods(idx2,k,m);
%         w = x2-x1; h = y2-y1;
%         if w>0 && h>0
%             rectangle('Position',[x1,y1,w,h], 'EdgeColor',colors(m,:), ...
%                       'LineWidth',1.5,'LineStyle','-','FaceAlpha',0.1);
%         end
%     end
% 
%     % Boundary hull
%     for m = 1:n_methods
%         xb = cellfun(@(c) c(idx1,end,m), succ_low_boundary);
%         yb = cellfun(@(c) c(idx2,end,m), succ_low_boundary);
%         xu = cellfun(@(c) c(idx1,end,m), succ_up_boundary);
%         yu = cellfun(@(c) c(idx2,end,m), succ_up_boundary);
%         rectangle('Position',[min(xb), min(yb), max(xu)-min(xb), max(yu)-min(yb)], ...
%                   'EdgeColor',colors(m,:),'LineWidth',1.5,'LineStyle','--','FaceAlpha',0.1);
%     end
% 
%     legend({'Samples', ...
%             [method_names{1} ' (single)'], [method_names{2} ' (single)'], ...
%             [method_names{1} ' (boundary)'], [method_names{2} ' (boundary)']}, ...
%            'Location','southeast');
%     xlabel(sprintf('$x_%d$',idx1),'Interpreter','latex','FontSize',20);
%     ylabel(sprintf('$x_%d$',idx2),'Interpreter','latex','FontSize',20);
%     title(sprintf('$x_%d$ vs $x_%d$',idx1,idx2),'Interpreter','latex');
% 
%     fn = fullfile(save_dir, sprintf('reach_%d_%d.png', idx1, idx2));
%     saveas(gcf, fn);
% end
% 
% %% Save results
% save(fullfile(save_dir, 'NODE_RA_results.mat'), ...
%      'succ_low_all_methods', 'succ_up_all_methods', ...
%      'succ_low_boundary', 'succ_up_boundary', ...
%      'rand_succ', 't_points', 'time_initial', 'time_boundary');
% 
% %% Output struct
% results = struct(...
%     'succ_low_all_methods', succ_low_all_methods, ...
%     'succ_up_all_methods',  succ_up_all_methods, ...
%     'succ_low_boundary',    succ_low_boundary, ...
%     'succ_up_boundary',     succ_up_boundary, ...
%     'rand_succ',            rand_succ, ...
%     't_points',             t_points, ...
%     'time_initial',         time_initial, ...
%     'time_boundary',        time_boundary, ...
%     'save_dir',             save_dir);
% 
% fprintf('NODE_RA: Done. Results saved in: %s\n', save_dir);
% 
% end
% %------------- END OF CODE --------------




%% OLD Way 
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
% system_choice = 14;      % FPA (continuous-time)
system_choice = 15;     % Spiral_non (continuous-time)

% Map system choice to system name for directory naming
system_names = containers.Map({14, 15}, {'fpa', 'spiral_non'});
if ~isKey(system_names, system_choice)
    error('Unknown system_choice %d. Please define system name in system_names map.', system_choice);
end
system_name = system_names(system_choice);

%% Optional submethod choice for the sampled-data mixed-monotonicity method
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
methods = [3, 4]; % 3: Continuous-time mixed-monotonicity, 4: Continuous-time sampled-data mixed-monotonicity
n_methods = length(methods);
succ_low_all_methods = NaN(n_x, n_t, n_methods); % For initial approach
succ_up_all_methods = NaN(n_x, n_t, n_methods);

%% Boundary reachability parameters
SplitsB = 1; % Number of splits per dimension (2 boundaries: lower and upper)
num_splits = 2*n_x; % Total number of boundary splits (2*5 = 10 for 5D) and (2*2 = 4 for 2D)
Initial_set_boundaries = cell(num_splits, 1); % Boundary splits
succ_low_boundary = cell(num_splits, 1); % Lower bounds for boundary splits
succ_up_boundary = cell(num_splits, 1); % Upper bounds for boundary splits
boundary_times = zeros(num_splits, n_methods); % Computation times for boundary splits

%% Generate all 32 boundary combinations --> PJ solution for the making the lb != Ub 
for i = 1:n_x
    % Change dimension x_low(i) to x_up(i)
    temp_low = x_low;
    temp_up = x_up;
    temp_low(i) = x_up(i);
    Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
    % Change dimension x_up(i) to x_low(i)
    temp_low = x_low;
    temp_up = x_up;
    temp_up(i) = x_low(i);
    Initial_set_boundaries{2*i} = interval(temp_low(:), temp_up(:));
end

%% Call of the main over-approximation function
try
    if bool_discrete_time
        [succ_low, succ_up] = TIRA(t_init, x_low, x_up, p_low, p_up);
    else
        time_initial = zeros(1, n_methods);
        time_boundary = zeros(num_splits, n_methods);

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

        % Boundary approach: Compute reachable sets for all boundary splits
        for i = 1:num_splits
            succ_low_boundary{i} = NaN(n_x, n_t, n_methods);
            succ_up_boundary{i} = NaN(n_x, n_t, n_methods);
            for m = 1:n_methods
                if plot_last_only
                    fprintf('Computing boundary split %d, method %d, time %.2f, x_low=%s, x_up=%s\n', ...
                            i, methods(m), t_final, mat2str(Initial_set_boundaries{i}.inf), mat2str(Initial_set_boundaries{i}.sup));
                    if ~isvector(Initial_set_boundaries{i}.inf) || length(Initial_set_boundaries{i}.inf) ~= n_x || ...
                       ~isvector(Initial_set_boundaries{i}.sup) || length(Initial_set_boundaries{i}.sup) ~= n_x
                        error('Invalid input dimensions for boundary split %d: x_low=%s, x_up=%s', ...
                              i, mat2str(size(Initial_set_boundaries{i}.inf)), mat2str(size(Initial_set_boundaries{i}.sup)));
                    end
                    tStart = tic;
                    [succ_low, succ_up] = TIRA([t_init, t_final], Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
                    time_boundary(i, m) = time_boundary(i, m) + toc(tStart);
                    if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                        error('TIRA returned invalid dimensions for boundary split %d, method %d: succ_low=%s, succ_up=%s', ...
                              i, methods(m), mat2str(size(succ_low)), mat2str(size(succ_up)));
                    end
                    succ_low_boundary{i}(:, end, m) = succ_low;
                    succ_up_boundary{i}(:, end, m) = succ_up;
                else
                    for k = 1:n_t
                        t_final_k = t_points(k);
                        fprintf('Computing boundary split %d, method %d, time %.2f, x_low=%s, x_up=%s\n', ...
                                i, methods(m), t_final_k, mat2str(Initial_set_boundaries{i}.inf), mat2str(Initial_set_boundaries{i}.sup));
                        if ~isvector(Initial_set_boundaries{i}.inf) || length(Initial_set_boundaries{i}.inf) ~= n_x || ...
                           ~isvector(Initial_set_boundaries{i}.sup) || length(Initial_set_boundaries{i}.sup) ~= n_x
                            error('Invalid input dimensions for boundary split %d: x_low=%s, x_up=%s', ...
                                  i, mat2str(size(Initial_set_boundaries{i}.inf)), mat2str(size(Initial_set_boundaries{i}.sup)));
                        end
                        tStart = tic;
                        [succ_low, succ_up] = TIRA([t_init, t_final_k], Initial_set_boundaries{i}.inf, Initial_set_boundaries{i}.sup, p_low, p_up, methods(m));
                        time_boundary(i, m) = time_boundary(i, m) + toc(tStart); % Accumulate time for each time step
                        if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                            error('TIRA returned invalid dimensions for boundary split %d, method %d, time %.2f: succ_low=%s, succ_up=%s', ...
                                  i, methods(m), t_final_k, mat2str(size(succ_low)), mat2str(size(succ_up)));
                        end
                        succ_low_boundary{i}(:, k, m) = succ_low;
                        succ_up_boundary{i}(:, k, m) = succ_up;
                    end
                end
            end
            fprintf('Boundary Split %d computed successfully for all methods.\n', i);
        end
    end

    % Display timing results
    fprintf('\n=== Reachability Computation Times at t = %.2f (seconds) ===\n', t_final);
    for m = 1:n_methods
        fprintf('Initial Method %d (CT Mixed-Monotonicity): %.6f\n', methods(m), time_initial(m));
        if plot_last_only
            for i = 1:num_splits
                fprintf('Boundary Split %d, Method %d (CT Mixed-Monotonicity): %.6f\n', i, methods(m), time_boundary(i, m));
            end
        end
    end
    for m = 1:n_methods
        fprintf('Initial Method %d (CT Sampled-Data Mixed-Monotonicity): %.6f\n', methods(m), time_initial(m));
        if plot_last_only
            for i = 1:num_splits
                fprintf('Boundary Split %d, Method %d (CT Sampled-Data Mixed-Monotonicity): %.6f\n', i, methods(m), time_boundary(i, m));
            end
        end
    end
catch e
    fprintf('Error in reachability computation: %s\n', e.message);
    rethrow(e);
end

%% Optional choice of the over-approximation method to be used
% The choice of the over-approximation method can be specified directly in
% the call of the TIRA function by defining the integer 'OA_method' to one 
% of the methods below:
%     3 - Continuous-time mixed-monotonicity
%     4 - Continuous-time sampled-data mixed-monotonicity
% then adding 'OA_method' as the last argument of function TIRA:
% [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, OA_method);
% [succ_low, succ_up] = TIRA(t_init, x_low, x_up, p_low, p_up, OA_method);

% If not specified as above, the TIRA function checks whether the choice of
% the over-approximation method is provided in the file Solver_parameters.m
% *More details on each over-approximation method and their requirements
% are provided in the file Solver_parameters.m

% If the desired over-approximation method is defined neither in the TIRA
% function call, nor in the file Solver_parameters.m, then function TIRA.m 
% will pick the most suitable method that can be applied to the system 
% based on the additional system descriptions provided by the user in the 
% 'UP_...' files of the current folder.

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
    colors = [0 0 1; 1 0 0]; % Dark Blue, Red
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

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
        handles = [];
        proxy_handles = [];
        tightness_metrics = zeros(1, 4); % For 4 approaches (Single-step and Boundary only)

        if plot_last_only
            k = size(succ_low_all_methods, 2); % Last time step

            % 1: Mixed-Monotonicity (Boundary) - Dashed Magenta
            all_x_min_bound = [];
            all_x_max_bound = [];
            all_y_min_bound = [];
            all_y_max_bound = [];
            for i = 1:num_splits
                fprintf('Boundary data m=1, split %d: low=[%f %f], up=[%f %f]\n', i, ...
                        succ_low_boundary{i}(idx1, k, 1), succ_low_boundary{i}(idx2, k, 1), ...
                        succ_up_boundary{i}(idx1, k, 1), succ_up_boundary{i}(idx2, k, 1));
                all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 1)];
                all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 1)];
                all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 1)];
                all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 1)];
            end
            x_min_bound = min(all_x_min_bound);
            x_max_bound = max(all_x_max_bound);
            y_min_bound = min(all_y_min_bound);
            y_max_bound = max(all_y_max_bound);
            width_bound = x_max_bound - x_min_bound;
            height_bound = y_max_bound - y_min_bound;
            fprintf('Mixed-Monotonicity (Boundary) computed bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    x_min_bound, y_min_bound, width_bound, height_bound);
            if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
                h_bound1 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
                                    'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
                h_proxy_bound1 = plot(NaN, NaN, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
                handles = [handles, h_bound1];
                proxy_handles = [proxy_handles, h_proxy_bound1];
                tightness_metrics(1) = (width_bound * height_bound) / area_actual;
            else
                warning('Mixed-Monotonicity (Boundary) has invalid bounds. Using actual range.');
                h_bound1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
                                   'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1, 'LineStyle', ':');
                h_proxy_bound1 = plot(NaN, NaN, 'm:', 'LineWidth', 1.5, 'Visible', 'off');
                handles = [handles, h_bound1];
                proxy_handles = [proxy_handles, h_proxy_bound1];
                tightness_metrics(1) = 1.0;
            end

            % 2: Sampled-Data Mixed-Monotonicity (Boundary) - Dashed Green
            all_x_min_bound = [];
            all_x_max_bound = [];
            all_y_min_bound = [];
            all_y_max_bound = [];
            for i = 1:num_splits
                fprintf('Boundary data m=2, split %d: low=[%f %f], up=[%f %f]\n', i, ...
                        succ_low_boundary{i}(idx1, k, 2), succ_low_boundary{i}(idx2, k, 2), ...
                        succ_up_boundary{i}(idx1, k, 2), succ_up_boundary{i}(idx2, k, 2));
                all_x_min_bound = [all_x_min_bound, succ_low_boundary{i}(idx1, k, 2)];
                all_x_max_bound = [all_x_max_bound, succ_up_boundary{i}(idx1, k, 2)];
                all_y_min_bound = [all_y_min_bound, succ_low_boundary{i}(idx2, k, 2)];
                all_y_max_bound = [all_y_max_bound, succ_up_boundary{i}(idx2, k, 2)];
            end
            x_min_bound = min(all_x_min_bound);
            x_max_bound = max(all_x_max_bound);
            y_min_bound = min(all_y_min_bound);
            y_max_bound = max(all_y_max_bound);
            width_bound = x_max_bound - x_min_bound;
            height_bound = y_max_bound - y_min_bound;
            fprintf('Sampled-Data Mixed-Monotonicity (Boundary) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    x_min_bound, y_min_bound, width_bound, height_bound);
            if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
                h_bound2 = rectangle('Position', [x_min_bound, y_min_bound, max(width_bound, 0.001), max(height_bound, 0.001)], ...
                                    'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--', 'FaceAlpha', 0.1);
                h_proxy_bound2 = plot(NaN, NaN, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
                handles = [handles, h_bound2];
                proxy_handles = [proxy_handles, h_proxy_bound2];
                tightness_metrics(2) = (width_bound * height_bound) / area_actual;
            else
                warning('Skipping Sampled-Data Mixed-Monotonicity (Boundary) due to invalid parameters');
            end

            % 3: Mixed-Monotonicity (Single-step) - Dark Blue Solid
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
                handles = [handles, h_single1];
                proxy_handles = [proxy_handles, h_proxy_single1];
                tightness_metrics(3) = (width_single * height_single) / area_actual;
            else
                warning('Mixed-Monotonicity (Single-step) has invalid bounds. Using actual range.');
                h_single1 = rectangle('Position', [x_min_actual, y_min_actual, x_max_actual - x_min_actual, y_max_actual - y_min_actual], ...
                                   'EdgeColor', colors(1,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1, 'LineStyle', ':');
                h_proxy_single1 = plot(NaN, NaN, ':', 'Color', colors(1,:), 'LineWidth', 1.5, 'Visible', 'off');
                handles = [handles, h_single1];
                proxy_handles = [proxy_handles, h_proxy_single1];
                tightness_metrics(3) = 1.0;
            end

            % 4: Sampled-Data Mixed-Monotonicity (Single-step) - Red Solid
            x_min_single = succ_low_all_methods(idx1, k, 2); % Method 2
            y_min_single = succ_low_all_methods(idx2, k, 2);
            width_single = succ_up_all_methods(idx1, k, 2) - x_min_single;
            height_single = succ_up_all_methods(idx2, k, 2) - y_min_single;
            fprintf('Sampled-Data Mixed-Monotonicity (Single-step) bounds: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    x_min_single, y_min_single, width_single, height_single);
            if all(isfinite([x_min_single, y_min_single, width_single, height_single])) && width_single >= 0 && height_single >= 0
                h_single2 = rectangle('Position', [x_min_single, y_min_single, max(width_single, 0.001), max(height_single, 0.001)], ...
                                    'EdgeColor', colors(2,:), 'LineWidth', 1.5, 'LineStyle', '-', 'FaceAlpha', 0.1);
                h_proxy_single2 = plot(NaN, NaN, '-', 'Color', colors(2,:), 'LineWidth', 1.5, 'Visible', 'off');
                handles = [handles, h_single2];
                proxy_handles = [proxy_handles, h_proxy_single2];
                tightness_metrics(4) = (width_single * height_single) / area_actual;
            else
                warning('Skipping Sampled-Data Mixed-Monotonicity (Single-step) due to invalid parameters');
            end
        end

        % Adjust axis to include all bounds
        x_min_all = min([x_min_actual, x_min_bound, x_min_single]);
        x_max_all = max([x_max_actual, x_max_bound + width_bound, x_min_single + width_single]);
        y_min_all = min([y_min_actual, y_min_bound, y_min_single]);
        y_max_all = max([y_max_actual, y_max_bound + height_bound, y_min_single + height_single]);
        x_range = x_max_all - x_min_all;
        y_range = y_max_all - y_min_all;
        buffer = 0.2; % 10% padding
        axis([x_min_all - buffer*x_range, x_max_all + buffer*x_range, y_min_all - buffer*y_range, y_max_all + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add single legend with correct method assignments
        all_proxy_handles = [h_proxy_single1, h_proxy_single2, h_proxy_bound1, h_proxy_bound2];
        all_legend_labels = {'Mixed-Monotonicity (Single-step)', 'Sampled-Data Mixed-Monotonicity (Single-step)', ...
                             'Mixed-Monotonicity (Boundary)', 'Sampled-Data Mixed-Monotonicity (Boundary)'};
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

        fprintf('Tightness Metrics (Area Ratio) for x_%d vs x_%d plane:\n', idx1, idx2);
        fprintf('Mixed-Monotonicity (Single-step): %.2f\n', tightness_metrics(3));
        fprintf('Mixed-Monotonicity (Boundary): %.2f\n', tightness_metrics(1));
        fprintf('Sampled-Data Mixed-Monotonicity (Single-step): %.2f\n', tightness_metrics(4));
        fprintf('Sampled-Data Mixed-Monotonicity (Boundary): %.2f\n', tightness_metrics(2));
    end
catch e
    fprintf('Error in plotting: %s\n', e.message);
    rethrow(e);
end

%% Plot the Boundary reachability splits (10~5D or 4~2D)
try
    % Define plot pairs based on system choice
    plot_pairs = [];
    if system_choice == 14
        plot_pairs = [1 2; 3 4; 4 5]; % For 5D FPA system: x1 vs x2, x3 vs x4, x4 vs x5
    elseif system_choice == 15
        plot_pairs = [1 2]; % For 2D Spiral system: x1 vs x2
    end
    colors = jet(num_splits); % Generate a range of colors for boundary splits
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    for p = 1:size(plot_pairs, 1)
        % Plot boundary points and union interval hull
        figure('Name', sprintf('Boundary Points - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
        hold on;
        grid on;
        idx1 = plot_pairs(p, 1);
        idx2 = plot_pairs(p, 2);

        % Plot the successors from random initial states
        h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Calculate initial axis limits with padding
        x_min = min(rand_succ(idx1, :));
        x_max = max(rand_succ(idx1, :));
        y_min = min(rand_succ(idx2, :));
        y_max = max(rand_succ(idx2, :));

        if plot_last_only
            k = size(succ_low_boundary{1}, 2); % Use actual size of second dimension
        else
            k = n_t; % Use all time steps
        end

        % Debug: Check the size of the boundary arrays
        fprintf('Size of succ_low_boundary{1}: %s\n', mat2str(size(succ_low_boundary{1})));

        % Plot boundary points (vertices of boundary splits) with unified colors
        handles = [];
        for i = 1:num_splits
            for m = 1:n_methods
                x_min_bound = succ_low_boundary{i}(idx1, k, m);
                x_max_bound = succ_up_boundary{i}(idx1, k, m);
                y_min_bound = succ_low_boundary{i}(idx2, k, m);
                y_max_bound = succ_up_boundary{i}(idx2, k, m);
                % Plot vertices of the boundary rectangle
                vertices_x = [x_min_bound, x_max_bound, x_max_bound, x_min_bound];
                vertices_y = [y_min_bound, y_min_bound, y_max_bound, y_max_bound];
                if all(isfinite([vertices_x, vertices_y]))
                    if m == 1
                        plot_color = 'b';
                        method_label = 'CT Mixed-Monotonicity';
                    else
                        plot_color = 'r';
                        method_label = 'CT Sampled-Data Mixed-Monotonicity';
                    end
                    h = plot(vertices_x, vertices_y, 'o', 'Color', plot_color, 'MarkerSize', 5, ...
                             'MarkerFaceColor', plot_color, 'DisplayName', method_label);
                    rectangle('Position', [x_min_bound, y_min_bound, x_max_bound - x_min_bound, y_max_bound - y_min_bound]);
                    if i == 1
                        handles = [handles, h(1)];
                    end
                else
                    fprintf('Warning: Invalid bounds for split %d, method %d: skipping points\n', i, m);
                end
                x_min = min(x_min, x_min_bound);
                x_max = max(x_max, x_max_bound);
                y_min = min(y_min, y_min_bound);
                y_max = max(y_max, y_max_bound);
            end
        end

        % Adjust axis with larger padding
        x_range = x_max - x_min;
        y_range = y_max - y_min;
        buffer = 0.4; % Increased to 50% padding for better zoom out
        axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['Boundary Points - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add legend
        if ~isempty(handles)
            legend([h_random, handles], {'Random Successors', 'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'}, 'Location', 'southeast');
        else
            fprintf('Warning: No valid handles for legend, skipping legend.\n');
        end

        % Save the boundary points plot
        if exist(save_dir, 'dir')
            try
                filename_base = sprintf('boundary_points_%d_%d_%s', idx1, idx2, timestamp);
                saveas(gcf, fullfile(save_dir, [filename_base '.png']), 'png');
                fprintf('Boundary points plot saved as PNG: %s\n', fullfile(save_dir, [filename_base '.png']));
                saveas(gcf, fullfile(save_dir, [filename_base '.eps']), 'epsc');
                fprintf('Boundary points plot saved as EPS: %s\n', fullfile(save_dir, [filename_base '.eps']));
            catch e
                warning('Failed to save boundary points plot: %s', e.message);
            end
        else
            warning('Directory %s does not exist. Skipping boundary points plot save.', save_dir);
        end

        % Plot Interval Hulls for each OA method
        figure('Name', sprintf('Interval Hull - x%d vs x%d', plot_pairs(p,1), plot_pairs(p,2)));
        hold on;
        grid on;

        % Plot random successors again
        h_random = plot(NaN, NaN, 'k.', 'MarkerSize', 5); % Proxy for legend
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Plot per-method interval hulls
        x_min = min(rand_succ(idx1, :));
        x_max = max(rand_succ(idx1, :));
        y_min = min(rand_succ(idx2, :));
        y_max = max(rand_succ(idx2, :));
        handles = [];
        for m = 1:n_methods
            x_min_hull_m = Inf;
            x_max_hull_m = -Inf;
            y_min_hull_m = Inf;
            y_max_hull_m = -Inf;
            for i = 1:num_splits
                x_min_hull_m = min(x_min_hull_m, succ_low_boundary{i}(idx1, k, m));
                x_max_hull_m = max(x_max_hull_m, succ_up_boundary{i}(idx1, k, m));
                y_min_hull_m = min(y_min_hull_m, succ_low_boundary{i}(idx2, k, m));
                y_max_hull_m = max(y_max_hull_m, succ_up_boundary{i}(idx2, k, m));
            end
            width_hull_m = x_max_hull_m - x_min_hull_m;
            height_hull_m = y_max_hull_m - y_min_hull_m;
            if all(isfinite([x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m])) && width_hull_m >= 0 && height_hull_m >= 0
                if m == 1
                    plot_color = 'b';
                else
                    plot_color = 'r';
                end
                h_method = rectangle('Position', [x_min_hull_m, y_min_hull_m, width_hull_m, height_hull_m], ...
                                    'EdgeColor', plot_color, 'LineWidth', 2, 'LineStyle', '-', 'FaceAlpha', 0.1);
                % Create proxy for legend
                h_method_proxy = plot(NaN, NaN, plot_color, 'LineWidth', 2);
                handles = [handles, h_method_proxy];
            else
                fprintf('Warning: Invalid bounds for interval hull, method %d: skipping rectangle\n', m);
            end
            x_min = min(x_min, x_min_hull_m);
            x_max = max(x_max, x_max_hull_m);
            y_min = min(y_min, y_min_hull_m);
            y_max = max(y_max, y_max_hull_m);
        end

        % Adjust axis
        x_range = x_max - x_min;
        y_range = y_max - y_min;
        axis([x_min - buffer*x_range, x_max + buffer*x_range, y_min - buffer*y_range, y_max + buffer*y_range]);

        % Add labels
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['Interval Hull - $x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);

        % Add legend
        if ~isempty(handles)
            legend([h_random, handles], {'Random Successors', method_names{:}}, 'Location', 'southeast');
        else
            fprintf('Warning: No valid handles for legend, skipping legend.\n');
        end

        % Save the figure
        saveas(gcf, fullfile(save_dir, sprintf('interval_hull_%d_%d_%s.png', idx1, idx2, timestamp)), 'png');
    end
catch e
    fprintf('Error in plotting: %s\n', e.message);
    rethrow(e);
end

% Display timing results
fprintf('Computation Times (seconds) for all plots:\n');
for m = 1:n_methods
    fprintf('Initial Method %s: %.6f\n', method_names{m}, time_initial(m));
    fprintf('Boundary Method %s: %.6f\n', method_names{m}, sum(time_boundary(:, m)));
end

%% Save the reachable sets and random successors
try
    timestamp = datestr(now, 'yyyy-mm-dd');
    filename_base = sprintf('reachability_data_sys_TIRA%d_all_%s', system_choice, timestamp);
    save(fullfile(save_dir, [filename_base '.mat']), 'succ_low_all_methods', 'succ_up_all_methods', ...
         'succ_low_boundary', 'succ_up_boundary', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', ...
         'time_initial', 'time_boundary');
    disp(['Data saved to ' fullfile(save_dir, [filename_base '.mat'])]);
catch e
    fprintf('Error in saving data: %s\n', e.message);
    rethrow(e);
end

%% -------------------------------------------------
%  Return everything that Verifier needs
%%% -------------------------------------------------
results = struct();
results.succ_low_all_methods = succ_low_all_methods;
results.succ_up_all_methods  = succ_up_all_methods;
results.succ_low_boundary    = succ_low_boundary;
results.succ_up_boundary     = succ_up_boundary;
results.rand_succ            = rand_succ;
results.t_points             = t_points;
results.time_initial         = time_initial;
results.time_boundary        = time_boundary;
results.save_dir             = save_dir;
results.plot_pairs           = plot_pairs;      % needed for plotting in Verifier
results.plot_last_only       = plot_last_only;
results.n_x                  = n_x;
results.n_methods            = n_methods;
results.methods              = methods;
results.system_choice        = system_choice;

% (optional) also return the handles of the figures that were created
%   – not needed for the safety check but handy for debugging
results.fig_handles = findobj('Type','figure');

%------------- END OF CODE --------------