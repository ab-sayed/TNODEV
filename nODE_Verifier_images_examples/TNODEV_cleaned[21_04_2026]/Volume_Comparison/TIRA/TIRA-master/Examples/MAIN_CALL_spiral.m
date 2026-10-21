%% Initialization
close all
clear
bool_discrete_time = 0;
% Folder containing the various over-approximation methods
addpath('../OA_methods')
% Folder containing useful tools and functions
addpath('../Utilities')    

%% User-defined approach and plotting selection
% Set approach to 'initial' for the initial step approach or 'incremental' for the incremental step approach
approach = 'initial'; % Options: 'initial' or 'incremental'
% Set plot_last_only to true to plot only the last reachable set, false for full reachable sets
plot_last_only = true; % Options: true or false

%% Choice of the example system
% To find a description of each of these systems (and references), 
% look at their corresponding definitions in System_description.m.

global system_choice
% system_choice = 1;      % Unicycle (continuous-time)
% system_choice = 2;      % Traffic diverge 3 link (continuous-time)
% system_choice = 3;      % Ship (continuous-time)
% system_choice = 4;      % Traffic diverge 3 links (discrete-time)
% system_choice = 5;      % Temperature variations in circular building (discrete time)
% system_choice = 6;      % Time-varying system (continuous-time)
% system_choice = 7;      % Bicycle (continuous-time)
% system_choice = 8;      % Protein interaction (continuous-time)
% system_choice = 9;      % Traffic diverge n_x links (continuous-time)
% system_choice = 10;     % Longitudinal Quadrotor Model (continuous-time)
% system_choice = 11;     % 2 unicycles pursuer-evader (continuous-time)
% system_choice = 12;     % Simple oscillator (continuous-time)
% system_choice = 13;     % Tunnel Diode Oscillator (continuous-time)
% system_choice = 14;      % FPA (continuous-time)
system_choice = 15;     % Spiral_non (continuous-time)

%% Optional submethod choice for the sampled-data mixed-monotonicity method
global g_sensitivity_bounds_method
g_sensitivity_bounds_method = nan;

%% Definition of the reachability problem (for each example system)
global u        % Some systems need an external control input
switch system_choice
    case 1
        %% Unicycle (continuous-time)
        % Interval of initial states (defined by two column vectors)
        % (x(3) initial set needs to be in [-pi,pi])
        x_up = [1;1;pi/8];
        x_low = [0;0;-pi/8];

        % Control
        u = [0.25;0.15];

        % Interval of allowed input values (disturbances)
        p_up = [0.05;0.05;0.03];
        p_low = -p_up;

        % Time interval
        t_init = 0;         % Initial time
        t_final = 100;        % Final time

    case 2
        %% Traffic diverge 3 link (continuous-time)
        % Defined as a system with additive input: dx=f(t,x)+p

        T = 100; % Time step

        % Interval of initial states (defined by two column vectors)
        x_low = [150;180;100];
        x_up = [200;300;220];

        % Interval of allowed input values (input flow into link 1)
        p_low = [40;0;0]/T;
        p_up = [60;0;0]/T;

        % Time interval
        t_init = 0;                 % Initial time
        t_final = t_init + T;       % Final time

    case 3
        %% Ship (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_up = [1;1;pi/8;0.15;0.05;0.2];
        x_low = [0;0;-pi/8;0.05;-0.05;-0.2];

        % Interval of allowed input values (velocity/heading setpoints)
        p_up = [0.1;pi/2];
        p_low = [0.1;pi/2];

        % Time interval
        t_init = 0;         % Initial time
        t_final = 10;        % Final time

    case 4
        %% Traffic diverge 3 link (discrete-time)
        % Interval of initial states (defined by two column vectors)
        x_low = [150;180;100];
        x_up = [200;300;220];

        % Interval of allowed input values (input flow into link 1)
        p_low = 40;
        p_up = 60;

        % Time interval
        t_init = 0;         % Initial time
        bool_discrete_time = 1;

    case 5
        %% Temperature variations in circular building (discrete time)
        % Interval of initial states (defined by two column vectors)
        x_low = [18;20;20;19;21;17];
        x_up = [19;20;21;21;22;19];

        % Interval of allowed input values (heater control)
        heater_max = 0.6;   % <=0.6 to have a monotone system ; >0.6 for a general system (non-monotone, non sign-stable)
        p_low = zeros(length(x_low),1);
        p_up = heater_max*ones(length(x_low),1);

        % Time interval
        t_init = 0;         % Initial time
        bool_discrete_time = 1;

    case 6
        %% Time-varying system (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_low = [1;2;3];
        x_up = [2;4;4];

        % Interval of allowed input values (defined by two column vectors)
        p_up = 4*ones(length(x_low),1);
        p_low = -p_up;

        % Time interval
        t_init = 1;
        t_final = 2;

    case 7
        %% Bicycle (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_up = [1;1;pi/8];
        x_low = [0;0;-pi/8];

        % Control
        u = [0.25;0.15];

        % Interval of allowed input values (disturbances)
        p_up = 0.3*ones(length(x_low),1);
        p_low = -p_up;

        % Time interval
        t_init = 0;
        t_final = 2;

    case 8
        %% Protein interaction (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_low = [0;0];
        x_up = [1;1];

        % Interval of allowed input values
        % (system has no input: set both p_up and p_low to 0)
        p_up = 0;
        p_low = 0;

        % Time interval
        t_init = 0;
        t_final = 1;

    case 9
        %% Traffic diverge nx-link (continuous-time ; needs n_x >= 5)
        % Defined as a system with additive input: dx=f(t,x)+p

        % Number of links in traffic network (needs to be >=5)
        n_x = 10;

        T = 30; % Time step

        % Interval of initial states (defined by two column vectors)
        x_low = 100*ones(n_x,1);
        x_up = 200*ones(n_x,1);

        % Interval of allowed input values (input flow into link 1)
        p_low = zeros(n_x,1);
        p_low(1) = 40/T;
        p_up = zeros(n_x,1);
        p_up(1) = 60/T;

        % Time interval
        t_init = 0;         % Initial time
        t_final = t_init + T;        % Final time

    case 10
        %% Nonlinear Longitudinal Model of a Quadrotor (continuous-time)
        % Parameters
        K = 0.89/1.4;
        g = 9.81;

        % State constraints
        X_up = [1.7;2;0.8;1;pi/12;pi/2];
        X_low = [-1.7;0.3;-0.8;-1;-pi/12;-pi/2];

        % Interval of initial states (defined by two column vectors)
        x_up = [0.2;1.2;0.3;0.1;pi/20;pi/10];
        x_low = [-0.2;1;0.1;-0.2;0;-pi/10];

        % Control
        u_up = [1.5+g/K;pi/12];
        u_low = [-1.5+g/K;-pi/12];
        u = u_low + rand(size(u_low)).*(u_up-u_low);

        % Interval of allowed input values
        % (system has no input: set both p_up and p_low to 0)
        p_up = zeros(length(x_low),1);
        p_low = zeros(length(x_low),1);

        % Time interval
        t_init = 0;         % Initial time
        t_final = 2;        % Final time

    case 11
        %% Pursuer-evader game with 2 Dubin's vehicles (continuous-time)
        % Interval of initial states (defined by two column vectors)
        % (x(3) and x(6) initial sets need to be in [-pi,pi])
        x_up = [1;1;pi/8;2.5;2.5;3*pi/8];
        x_low = [0;0;-pi/8;2;2;pi/4];

        % Control
        u = [0.25;0.15];

        % Interval of allowed input values (disturbances)
        p_up = [0.5;0.3];
        p_low = -p_up;

        % Time interval
        t_init = 0;         % Initial time
        t_final = 100;        % Final time

    case 12
        %% Simple oscillator model (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_low = [0;0];
        x_up = [1;1];

        % Interval of allowed input values
        % (system has no input: set both p_up and p_low to 0)
        p_up = [0;0];
        p_low = [0;0];

        % Time interval
        t_init = 0;
        t_final = 25;
    case 13
        %% Tunnel Diode Oscillator (continuous-time)
        % Interval of initial states (defined by two column vectors)
        x_low = [0.45; 0.6/1000];
        x_up = [0.5; 0.6/1000];

        % Interval of allowed input values
        % (system has no input: set both p_up and p_low to 0)
        p_low = [0; 0];
        p_up = [0; 0];

        % Time interval
        t_init = 0;
        t_final = 1 * 10^-9;

    case 14
        %% FPA System (continuous-time)
        % % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
        Initial_radius = 0.01; % Uncertainty in dynamics
        x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
        x_low = x0 - Initial_radius;
        x_up = x0 + Initial_radius;

        % Define time and input bounds
        t_init = 0;     % Initial time
        t_final = 2;    % Final time
        steps = 0.05;    % Discretization steps / multiple subintervals
        t_points = t_init:steps:t_final;    % Time points: 0, 0.1, 0.2, ..., 1.0
        p_low = zeros(5,1); 
        p_up = zeros(5,1);

    case 15
        %% Spiral_non System (continuous-time)
        % % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
        Initial_radius = 0.2; % Uncertainty in dynamics
        x0 = [2.0; 0.0];
        x_low = x0 - Initial_radius;
        x_up = x0 + Initial_radius;

        % Define time and input bounds
        t_init = 0;     % Initial time
        t_final = 2;    % Final time
        steps = 0.01;    % Discretization steps
        t_points = t_init:steps:t_final;    % Time points
        p_low = zeros(2,1); 
        p_up = zeros(2,1);
end

% State and input dimensions
n_x = length(x_low);
n_p = length(p_low);
n_t = length(t_points);

% Define over-approximation of different methods to compare between them
% methods = [2, 3, 4, 7]; % 2: Continuous-time contraction and growth bound, 3: Continuous-time mixed-monotonicity, 4: Continuous-time sampled-data mixed-monotonicity, 7: Continuous-time Monte Carlo (for comparison only)
methods = [3, 4]; % 3: Continuous-time mixed-monotonicity, 4 - Continuous-time sampled-data mixed-monotonicity
n_methods = length(methods);
succ_low_all_methods = NaN(n_x, n_t, n_methods);
succ_up_all_methods = NaN(n_x, n_t, n_methods);

%% Call of the main over-approximation function
% succ_low is the lower bound of the over-approximation interval
% succ_up is the upper bound of the over-approximation interval

if bool_discrete_time
    % Call for discrete-time systems for one step starting from time t_init
    [succ_low,succ_up] = TIRA(t_init,x_low,x_up,p_low,p_up);
else
    % Timing storage for final-time reachability
    reach_times = zeros(1, n_methods);

    if strcmp(approach, 'initial')
        % 1- Call for continuous-time systems at multiple time points 
        % i.e. to compute each reachable set directly from the initial set to each 
        % time point, not step-by-step then restarting from that box to compute the next step.
        for m = 1:n_methods
            for k = 1:n_t
                t_final_k = t_points(k);

                if t_final_k == t_final
                    % Start timing only for the final time computation
                    tStartReach = tic;
                    [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
                    reach_times(m) = toc(tStartReach);
                else
                    % No timing, just compute normally
                    [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
                end

                succ_low_all_methods(:, k, m) = succ_low;
                succ_up_all_methods(:, k, m) = succ_up;
            end
        end

    elseif strcmp(approach, 'incremental')
        % 2- Call for continuous-time systems with incremental computation
        % i.e. to compute the reachable set incrementally for each time step, using 
        % the previous step's reachable set as the initial set for the next step.
        for m = 1:n_methods
            % Initialize reachable set at t=0
            succ_low_all_methods(:,1,m) = x_low;
            succ_up_all_methods(:,1,m) = x_up;
            % Compute reachable sets step-by-step
            for k = 1:n_t-1
                t_start = t_points(k);
                t_end = t_points(k+1);
                current_low = succ_low_all_methods(:,k,m);
                current_up = succ_up_all_methods(:,k,m);

                if t_end == t_final
                    % Time only the final incremental step
                    tStartReach = tic;
                    [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
                    reach_times(m) = toc(tStartReach);
                else
                    [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
                end

                succ_low_all_methods(:,k+1,m) = succ_low;
                succ_up_all_methods(:,k+1,m) = succ_up;
            end
        end
    else
        error('Invalid approach. Use ''initial'' or ''incremental''.');
    end

    % Display timing results
    fprintf('\n=== Reachability Computation Times at t = %.2f ===\n', t_final);
    for m = 1:n_methods
        fprintf('Method %d: %.6f seconds\n', methods(m), reach_times(m));
    end
end

%% Optional choice of the over-approximation method to be used
% The choice of the over-approximation method can be specified directly in
% the call of the TIRA function by defining the integer 'OA_method' to one 
% of the methods below:
%     1 - Continuous-time monotonicity
%     2 - Continuous-time contraction and growth bound
%     3 - Continuous-time mixed-monotonicity
%     4 - Continuous-time sampled-data mixed-monotonicity
%     5 - Discrete-time mixed-monotonicity
%     6 - Continuous-time and discrete-time Quasi-Monte Carlo
%     7 - Continuous-time and discrete-time Monte Carlo
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
        rand_succ(:,i) = System_description(t_init,x0,p);
    else
        [~,x_traj] = ode_solver(@(t,x) System_description(t,x,p),[t_init t_final],x0,ode_options);
        rand_succ(:,i) = x_traj(end,:)';
    end
end
t_rand_succs = toc(t_start);
log.runtime('Time to generate random successors: %f seconds\n', t_rand_succs);
%% Plot the reachable set and over-approximations
% Adjust plotting for 2D systems
if n_x == 2
    plot_pairs = [1 2]; % Plot x1 vs x2 for 2D system
    colors = lines(n_methods);
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    for i = 1:size(plot_pairs, 1)
        figure
        hold on
        grid on
        idx1 = plot_pairs(i, 1);
        idx2 = plot_pairs(i, 2);

        % Plot the successors from random initial states
        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        % Calculate minimum bounding box for black dots
        x_min_actual = min(rand_succ(idx1, :));
        x_max_actual = max(rand_succ(idx1, :));
        y_min_actual = min(rand_succ(idx2, :));
        y_max_actual = max(rand_succ(idx2, :));
        area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);

        % Plot over-approximation rectangles for each method
        handles = [];
        proxy_handles = [];
        for m = 1:n_methods
            h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
            proxy_handles = [proxy_handles, h_proxy];

            if plot_last_only
                k = n_t;
                x_min = succ_low_all_methods(idx1, k, m);
                y_min = succ_low_all_methods(idx2, k, m);
                width = succ_up_all_methods(idx1, k, m) - x_min;
                height = succ_up_all_methods(idx2, k, m) - y_min;
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
            else
                for k = 1:n_t
                    x_min = succ_low_all_methods(idx1, k, m);
                    y_min = succ_low_all_methods(idx2, k, m);
                    width = succ_up_all_methods(idx1, k, m) - x_min;
                    height = succ_up_all_methods(idx2, k, m) - y_min;
                    h = rectangle('Position', [x_min, y_min, width, height], ...
                                  'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                    handles = [handles, h];
                end
            end
            % Calculate tightness metric for the final time step
            x_min_final = succ_low_all_methods(idx1, end, m);
            y_min_final = succ_low_all_methods(idx2, end, m);
            width_final = succ_up_all_methods(idx1, end, m) - x_min_final;
            height_final = succ_up_all_methods(idx2, end, m) - y_min_final;
            area_overapprox = width_final * height_final;
            tightness_metrics(m) = area_overapprox / area_actual;
        end

        % Add labels and legend
        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
        legend(proxy_handles, method_names, 'Location', 'southeast');

        % Display tightness metrics
        fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
                ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
        for m = 1:n_methods
            fprintf('Method %s: %.2f\n', method_names{m}, tightness_metrics(m));
        end
    end
elseif n_x == 5
    plot_pairs = [1 2; 3 4; 4 5];
    colors = lines(n_methods);
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    for i = 1:size(plot_pairs, 1)
        figure
        hold on
        grid on
        idx1 = plot_pairs(i, 1);
        idx2 = plot_pairs(i, 2);

        for j = 1:sample_succ_number
            plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
        end

        x_min_actual = min(rand_succ(idx1, :));
        x_max_actual = max(rand_succ(idx1, :));
        y_min_actual = min(rand_succ(idx2, :));
        y_max_actual = max(rand_succ(idx2, :));
        area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);

        handles = [];
        proxy_handles = [];
        for m = 1:n_methods
            h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
            proxy_handles = [proxy_handles, h_proxy];

            if plot_last_only
                k = n_t;
                x_min = succ_low_all_methods(idx1, k, m);
                y_min = succ_low_all_methods(idx2, k, m);
                width = succ_up_all_methods(idx1, k, m) - x_min;
                height = succ_up_all_methods(idx2, k, m) - y_min;
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
            else
                for k = 1:n_t
                    x_min = succ_low_all_methods(idx1, k, m);
                    y_min = succ_low_all_methods(idx2, k, m);
                    width = succ_up_all_methods(idx1, k, m) - x_min;
                    height = succ_up_all_methods(idx2, k, m) - y_min;
                    h = rectangle('Position', [x_min, y_min, width, height], ...
                                  'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                    handles = [handles, h];
                end
            end
            x_min_final = succ_low_all_methods(idx1, end, m);
            y_min_final = succ_low_all_methods(idx2, end, m);
            width_final = succ_up_all_methods(idx1, end, m) - x_min_final;
            height_final = succ_up_all_methods(idx2, end, m) - y_min_final;
            area_overapprox = width_final * height_final;
            tightness_metrics(m) = area_overapprox / area_actual;
        end

        xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
        title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
        legend(proxy_handles, method_names, 'Location', 'southeast');

        fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
                ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
        for m = 1:n_methods
            fprintf('Method %s: %.2f\n', method_names{m}, tightness_metrics(m));
        end
    end
else
    warning('Plotting is not implemented for systems with %d states. Update plot_pairs for this dimension.', n_x);
end

% Ensure save operation occurs regardless of plotting
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods');
disp(['Data saved to ' filename_base '.mat']);


%% Using the Volume instead of the Tightness Matrices

% %% Initialization
% close all
% clear
% bool_discrete_time = 0;
% % Folder containing the various over-approximation methods
% addpath('../OA_methods')
% % Folder containing useful tools and functions
% addpath('../Utilities')    
% 
% %% User-defined approach and plotting selection
% % Set approach to 'initial' for the initial step approach or 'incremental' for the incremental step approach
% approach = 'initial'; % Options: 'initial' or 'incremental'
% % Set plot_last_only to true to plot only the last reachable set, false for full reachable sets
% plot_last_only = true; % Options: true or false
% 
% %% Choice of the example system
% % To find a description of each of these systems (and references), 
% % look at their corresponding definitions in System_description.m.
% 
% global system_choice
% % system_choice = 1;      % Unicycle (continuous-time)
% % system_choice = 2;      % Traffic diverge 3 link (continuous-time)
% % system_choice = 3;      % Ship (continuous-time)
% % system_choice = 4;      % Traffic diverge 3 links (discrete-time)
% % system_choice = 5;      % Temperature variations in circular building (discrete time)
% % system_choice = 6;      % Time-varying system (continuous-time)
% % system_choice = 7;      % Bicycle (continuous-time)
% % system_choice = 8;      % Protein interaction (continuous-time)
% % system_choice = 9;      % Traffic diverge n_x links (continuous-time)
% % system_choice = 10;     % Longitudinal Quadrotor Model (continuous-time)
% % system_choice = 11;     % 2 unicycles pursuer-evader (continuous-time)
% % system_choice = 12;     % Simple oscillator (continuous-time)
% % system_choice = 13;     % Tunnel Diode Oscillator (continuous-time)
% % system_choice = 14;      % FPA (continuous-time)
% system_choice = 15;     % Spiral_non (continuous-time)
% 
% %% Optional submethod choice for the sampled-data mixed-monotonicity method
% global g_sensitivity_bounds_method
% g_sensitivity_bounds_method = nan;
% 
% %% Definition of the reachability problem (for each example system)
% global u        % Some systems need an external control input
% switch system_choice
%     case 1
%         %% Unicycle (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         % (x(3) initial set needs to be in [-pi,pi])
%         x_up = [1;1;pi/8];
%         x_low = [0;0;-pi/8];
% 
%         % Control
%         u = [0.25;0.15];
% 
%         % Interval of allowed input values (disturbances)
%         p_up = [0.05;0.05;0.03];
%         p_low = -p_up;
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         t_final = 100;        % Final time
% 
%     case 2
%         %% Traffic diverge 3 link (continuous-time)
%         % Defined as a system with additive input: dx=f(t,x)+p
% 
%         T = 100; % Time step
% 
%         % Interval of initial states (defined by two column vectors)
%         x_low = [150;180;100];
%         x_up = [200;300;220];
% 
%         % Interval of allowed input values (input flow into link 1)
%         p_low = [40;0;0]/T;
%         p_up = [60;0;0]/T;
% 
%         % Time interval
%         t_init = 0;                 % Initial time
%         t_final = t_init + T;       % Final time
% 
%     case 3
%         %% Ship (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_up = [1;1;pi/8;0.15;0.05;0.2];
%         x_low = [0;0;-pi/8;0.05;-0.05;-0.2];
% 
%         % Interval of allowed input values (velocity/heading setpoints)
%         p_up = [0.1;pi/2];
%         p_low = [0.1;pi/2];
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         t_final = 10;        % Final time
% 
%     case 4
%         %% Traffic diverge 3 link (discrete-time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [150;180;100];
%         x_up = [200;300;220];
% 
%         % Interval of allowed input values (input flow into link 1)
%         p_low = 40;
%         p_up = 60;
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         bool_discrete_time = 1;
% 
%     case 5
%         %% Temperature variations in circular building (discrete time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [18;20;20;19;21;17];
%         x_up = [19;20;21;21;22;19];
% 
%         % Interval of allowed input values (heater control)
%         heater_max = 0.6;   % <=0.6 to have a monotone system ; >0.6 for a general system (non-monotone, non sign-stable)
%         p_low = zeros(length(x_low),1);
%         p_up = heater_max*ones(length(x_low),1);
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         bool_discrete_time = 1;
% 
%     case 6
%         %% Time-varying system (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [1;2;3];
%         x_up = [2;4;4];
% 
%         % Interval of allowed input values (defined by two column vectors)
%         p_up = 4*ones(length(x_low),1);
%         p_low = -p_up;
% 
%         % Time interval
%         t_init = 1;
%         t_final = 2;
% 
%     case 7
%         %% Bicycle (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_up = [1;1;pi/8];
%         x_low = [0;0;-pi/8];
% 
%         % Control
%         u = [0.25;0.15];
% 
%         % Interval of allowed input values (disturbances)
%         p_up = 0.3*ones(length(x_low),1);
%         p_low = -p_up;
% 
%         % Time interval
%         t_init = 0;
%         t_final = 2;
% 
%     case 8
%         %% Protein interaction (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [0;0];
%         x_up = [1;1];
% 
%         % Interval of allowed input values
%         % (system has no input: set both p_up and p_low to 0)
%         p_up = 0;
%         p_low = 0;
% 
%         % Time interval
%         t_init = 0;
%         t_final = 1;
% 
%     case 9
%         %% Traffic diverge nx-link (continuous-time ; needs n_x >= 5)
%         % Defined as a system with additive input: dx=f(t,x)+p
% 
%         % Number of links in traffic network (needs to be >=5)
%         n_x = 10;
% 
%         T = 30; % Time step
% 
%         % Interval of initial states (defined by two column vectors)
%         x_low = 100*ones(n_x,1);
%         x_up = 200*ones(n_x,1);
% 
%         % Interval of allowed input values (input flow into link 1)
%         p_low = zeros(n_x,1);
%         p_low(1) = 40/T;
%         p_up = zeros(n_x,1);
%         p_up(1) = 60/T;
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         t_final = t_init + T;        % Final time
% 
%     case 10
%         %% Nonlinear Longitudinal Model of a Quadrotor (continuous-time)
%         % Parameters
%         K = 0.89/1.4;
%         g = 9.81;
% 
%         % State constraints
%         X_up = [1.7;2;0.8;1;pi/12;pi/2];
%         X_low = [-1.7;0.3;-0.8;-1;-pi/12;-pi/2];
% 
%         % Interval of initial states (defined by two column vectors)
%         x_up = [0.2;1.2;0.3;0.1;pi/20;pi/10];
%         x_low = [-0.2;1;0.1;-0.2;0;-pi/10];
% 
%         % Control
%         u_up = [1.5+g/K;pi/12];
%         u_low = [-1.5+g/K;-pi/12];
%         u = u_low + rand(size(u_low)).*(u_up-u_low);
% 
%         % Interval of allowed input values
%         % (system has no input: set both p_up and p_low to 0)
%         p_up = zeros(length(x_low),1);
%         p_low = zeros(length(x_low),1);
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         t_final = 2;        % Final time
% 
%     case 11
%         %% Pursuer-evader game with 2 Dubin's vehicles (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         % (x(3) and x(6) initial sets need to be in [-pi,pi])
%         x_up = [1;1;pi/8;2.5;2.5;3*pi/8];
%         x_low = [0;0;-pi/8;2;2;pi/4];
% 
%         % Control
%         u = [0.25;0.15];
% 
%         % Interval of allowed input values (disturbances)
%         p_up = [0.5;0.3];
%         p_low = -p_up;
% 
%         % Time interval
%         t_init = 0;         % Initial time
%         t_final = 100;        % Final time
% 
%     case 12
%         %% Simple oscillator model (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [0;0];
%         x_up = [1;1];
% 
%         % Interval of allowed input values
%         % (system has no input: set both p_up and p_low to 0)
%         p_up = [0;0];
%         p_low = [0;0];
% 
%         % Time interval
%         t_init = 0;
%         t_final = 25;
%     case 13
%         %% Tunnel Diode Oscillator (continuous-time)
%         % Interval of initial states (defined by two column vectors)
%         x_low = [0.45; 0.6/1000];
%         x_up = [0.5; 0.6/1000];
% 
%         % Interval of allowed input values
%         % (system has no input: set both p_up and p_low to 0)
%         p_low = [0; 0];
%         p_up = [0; 0];
% 
%         % Time interval
%         t_init = 0;
%         t_final = 1 * 10^-9;
% 
%     case 14
%         %% FPA System (continuous-time)
%         % % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
%         Initial_radius = 0.01; % Uncertainty in dynamics
%         x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
%         x_low = x0 - Initial_radius;
%         x_up = x0 + Initial_radius;
% 
%         % Define time and input bounds
%         t_init = 0;     % Initial time
%         t_final = 2;    % Final time
%         steps = 0.05;    % Discretization steps / multiple subintervals
%         t_points = 0:steps:t_final;    % Time points: 0, 0.1, 0.2, ..., 1.0
%         p_low = zeros(5,1); 
%         p_up = zeros(5,1);
% 
%     case 15
%         %% Spiral_non System (continuous-time)
%         % % Initial states same as the ones defined in NNV2.0 'fpa_reach.m'
%         Initial_radius = 0.2; % Uncertainty in dynamics
%         x0 = [2.0; 0.0];
%         x_low = x0 - Initial_radius;
%         x_up = x0 + Initial_radius;
% 
%         % Define time and input bounds
%         t_init = 0;     % Initial time
%         t_final = 2;    % Final time
%         steps = 0.01;    % Discretization steps
%         t_points = t_init:steps:t_final;    % Time points
%         p_low = zeros(2,1); 
%         p_up = zeros(2,1);
% end
% 
% % State and input dimensions
% n_x = length(x_low);
% n_p = length(p_low);
% n_t = length(t_points);
% 
% % Define over-approximation of different methods to compare between them
% % methods = [2, 3, 4, 7]; % 2: Continuous-time contraction and growth bound, 3: Continuous-time mixed-monotonicity, 4: Continuous-time sampled-data mixed-monotonicity, 7: Continuous-time Monte Carlo (for comparison only)
% methods = [3, 4]; % 3: Continuous-time mixed-monotonicity, 4 - Continuous-time sampled-data mixed-monotonicity
% n_methods = length(methods);
% succ_low_all_methods = NaN(n_x, n_t, n_methods);
% succ_up_all_methods = NaN(n_x, n_t, n_methods);
% 
% %% Call of the main over-approximation function
% % succ_low is the lower bound of the over-approximation interval
% % succ_up is the upper bound of the over-approximation interval
% 
% if bool_discrete_time
%     % Call for discrete-time systems for one step starting from time t_init
%     [succ_low,succ_up] = TIRA(t_init,x_low,x_up,p_low,p_up);
% else
%     % Timing storage for final-time reachability
%     reach_times = zeros(1, n_methods);
% 
%     if strcmp(approach, 'initial')
%         % 1- Call for continuous-time systems at multiple time points 
%         for m = 1:n_methods
%             for k = 1:n_t
%                 t_final_k = t_points(k);
% 
%                 if t_final_k == t_final
%                     % Start timing only for the final time computation
%                     tStartReach = tic;
%                     [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
%                     reach_times(m) = toc(tStartReach);
%                 else
%                     % No timing, just compute normally
%                     [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
%                 end
% 
%                 succ_low_all_methods(:, k, m) = succ_low;
%                 succ_up_all_methods(:, k, m) = succ_up;
%             end
%         end
% 
%     elseif strcmp(approach, 'incremental')
%         % 2- Call for continuous-time systems with incremental computation
%         for m = 1:n_methods
%             % Initialize reachable set at t=0
%             succ_low_all_methods(:,1,m) = x_low;
%             succ_up_all_methods(:,1,m) = x_up;
%             % Compute reachable sets step-by-step
%             for k = 1:n_t-1
%                 t_start = t_points(k);
%                 t_end = t_points(k+1);
%                 current_low = succ_low_all_methods(:,k,m);
%                 current_up = succ_up_all_methods(:,k,m);
% 
%                 if t_end == t_final
%                     % Time only the final incremental step
%                     tStartReach = tic;
%                     [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
%                     reach_times(m) = toc(tStartReach);
%                 else
%                     [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
%                 end
% 
%                 succ_low_all_methods(:,k+1,m) = succ_low;
%                 succ_up_all_methods(:,k+1,m) = succ_up;
%             end
%         end
%     else
%         error('Invalid approach. Use ''initial'' or ''incremental''.');
%     end
% 
%     % Display timing results
%     fprintf('\n=== Reachability Computation Times at t = %.2f ===\n', t_final);
%     for m = 1:n_methods
%         fprintf('Method %d: %.6f seconds\n', methods(m), reach_times(m));
%     end
% end
% 
% %% Optional choice of the over-approximation method to be used
% % The choice of the over-approximation method can be specified directly in
% % the call of the TIRA function by defining the integer 'OA_method' to one 
% % of the methods below:
% %     1 - Continuous-time monotonicity
% %     2 - Continuous-time contraction and growth bound
% %     3 - Continuous-time mixed-monotonicity
% %     4 - Continuous-time sampled-data mixed-monotonicity
% %     5 - Discrete-time mixed-monotonicity
% %     6 - Continuous-time and discrete-time Quasi-Monte Carlo
% %     7 - Continuous-time and discrete-time Monte Carlo
% % then adding 'OA_method' as the last argument of function TIRA:
% % [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, OA_method);
% % [succ_low, succ_up] = TIRA(t_init, x_low, x_up, p_low, p_up, OA_method);
% 
% % If not specified as above, the TIRA function checks whether the choice of
% % the over-approximation method is provided in the file Solver_parameters.m
% % *More details on each over-approximation method and their requirements
% % are provided in the file Solver_parameters.m
% 
% % If the desired over-approximation method is defined neither in the TIRA
% % function call, nor in the file Solver_parameters.m, then function TIRA.m 
% % will pick the most suitable method that can be applied to the system 
% % based on the additional system descriptions provided by the user in the 
% % 'UP_...' files of the current folder.
% 
% %% Compute successors from random initial states and disturbances
% sample_succ_number = 1000;
% log = logger.get_logger();
% 
% % Extract ODE solver to use 
% run('Solver_parameters.m');
% ode_solver = parameters.ode_solver;
% ode_options = parameters.ode_options;
% 
% log.info('Compute successors from %d random initial states and parameters ...\n', sample_succ_number)
% t_start = tic;
% rand_succ = NaN(n_x,sample_succ_number);
% for i = 1:sample_succ_number
%     x0 = x_low + rand(n_x,1).*(x_up-x_low);
%     p = p_low + rand(n_p,1).*(p_up-p_low);
%     if bool_discrete_time
%         rand_succ(:,i) = System_description(t_init,x0,p);
%     else
%         [~,x_traj] = ode_solver(@(t,x) System_description(t,x,p),[t_init t_final],x0,ode_options);
%         rand_succ(:,i) = x_traj(end,:)';
%     end
% end
% t_rand_succs = toc(t_start);
% log.runtime('Time to generate random successors: %f seconds\n', t_rand_succs);
% 
% %% Compute volumes of the last reachable sets
% method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% volumes = zeros(n_methods, 1);
% for m = 1:n_methods
%     lb_last = succ_low_all_methods(:, end, m);
%     ub_last = succ_up_all_methods(:, end, m);
%     volumes(m) = prod(ub_last - lb_last);
% end
% 
% % Compute volume of the bounding box of sampled points
% lb_sampled = min(rand_succ, [], 2);
% ub_sampled = max(rand_succ, [], 2);
% volume_sampled = prod(ub_sampled - lb_sampled);
% 
% % Display volumes
% fprintf('\nVolumes of the Last Reachable Sets:\n');
% for m = 1:n_methods
%     fprintf('Method %s: %.6f\n', method_names{m}, volumes(m));
% end
% fprintf('Volume of Sampled Points Bounding Box: %.6f\n', volume_sampled);
% 
% %% Plot the reachable set and over-approximations
% % Adjust plotting for 2D systems
% if n_x == 2
%     plot_pairs = [1 2]; % Plot x1 vs x2 for 2D system
%     colors = lines(n_methods);
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         % Plot the successors from random initial states
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Plot over-approximation rectangles for each method
%         handles = [];
%         proxy_handles = [];
%         for m = 1:n_methods
%             h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%             proxy_handles = [proxy_handles, h_proxy];
% 
%             if plot_last_only
%                 k = n_t;
%                 x_min = succ_low_all_methods(idx1, k, m);
%                 y_min = succ_low_all_methods(idx2, k, m);
%                 width = succ_up_all_methods(idx1, k, m) - x_min;
%                 height = succ_up_all_methods(idx2, k, m) - y_min;
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%             else
%                 for k = 1:n_t
%                     x_min = succ_low_all_methods(idx1, k, m);
%                     y_min = succ_low_all_methods(idx2, k, m);
%                     width = succ_up_all_methods(idx1, k, m) - x_min;
%                     height = succ_up_all_methods(idx2, k, m) - y_min;
%                     h = rectangle('Position', [x_min, y_min, width, height], ...
%                                   'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                     handles = [handles, h];
%                 end
%             end
%         end
% 
%         % Add labels and legend
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%         legend(proxy_handles, method_names, 'Location', 'southeast');
%     end
% elseif n_x == 5
%     plot_pairs = [1 2; 3 4; 4 5];
%     colors = lines(n_methods);
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         handles = [];
%         proxy_handles = [];
%         for m = 1:n_methods
%             h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%             proxy_handles = [proxy_handles, h_proxy];
% 
%             if plot_last_only
%                 k = n_t;
%                 x_min = succ_low_all_methods(idx1, k, m);
%                 y_min = succ_low_all_methods(idx2, k, m);
%                 width = succ_up_all_methods(idx1, k, m) - x_min;
%                 height = succ_up_all_methods(idx2, k, m) - y_min;
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%             else
%                 for k = 1:n_t
%                     x_min = succ_low_all_methods(idx1, k, m);
%                     y_min = succ_low_all_methods(idx2, k, m);
%                     width = succ_up_all_methods(idx1, k, m) - x_min;
%                     height = succ_up_all_methods(idx2, k, m) - y_min;
%                     h = rectangle('Position', [x_min, y_min, width, height], ...
%                                   'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                     handles = [handles, h];
%                 end
%             end
%         end
% 
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%         legend(proxy_handles, method_names, 'Location', 'southeast');
%     end
% else
%     warning('Plotting is not implemented for systems with %d states. Update plot_pairs for this dimension.', n_x);
% end
% 
% % Ensure save operation occurs regardless of plotting
% timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
% save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods');
% disp(['Data saved to ' filename_base '.mat']);




%% Plot the reachable set and over-approximations for the 2D case of the spiral
% if n_x == 2
%     plot_pairs = [1 2]; % Plot x1 vs x2 for 2D system
%     colors = lines(n_methods);
%     % method_names = {'CT Contraction and Growth Bound', 'CT Mixed-Monotonicity', ...
%     %                'CT Sampled-Data Mixed-Monotonicity', 'CT Monte Carlo'};
%     method_names = {'CT Mixed-Monotonicity','CT Sampled-Data Mixed-Monotonicity'};
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         % Plot the successors from random initial states
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Calculate minimum bounding box for black dots
%         x_min_actual = min(rand_succ(idx1, :));
%         x_max_actual = max(rand_succ(idx1, :));
%         y_min_actual = min(rand_succ(idx2, :));
%         y_max_actual = max(rand_succ(idx2, :));
%         area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);
% 
%         % Plot over-approximation rectangles for each method
%         handles = [];
%         proxy_handles = [];
%         volumes = zeros(n_methods, 1);
%         for m = 1:n_methods
%             h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%             proxy_handles = [proxy_handles, h_proxy];
% 
%             if plot_last_only
%                 k = n_t;
%                 x_min = succ_low_all_methods(idx1, k, m);
%                 y_min = succ_low_all_methods(idx2, k, m);
%                 width = succ_up_all_methods(idx1, k, m) - x_min;
%                 height = succ_up_all_methods(idx2, k, m) - y_min;
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 volumes(m) = width * height; % Approximate volume as bounding box area
%             else
%                 for k = 1:n_t
%                     x_min = succ_low_all_methods(idx1, k, m);
%                     y_min = succ_low_all_methods(idx2, k, m);
%                     width = succ_up_all_methods(idx1, k, m) - x_min;
%                     height = succ_up_all_methods(idx2, k, m) - y_min;
%                     h = rectangle('Position', [x_min, y_min, width, height], ...
%                                   'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                     handles = [handles, h];
%                 end
%                 x_min_final = succ_low_all_methods(idx1, end, m);
%                 y_min_final = succ_low_all_methods(idx2, end, m);
%                 width_final = succ_up_all_methods(idx1, end, m) - x_min_final;
%                 height_final = succ_up_all_methods(idx2, end, m) - y_min_final;
%                 volumes(m) = width_final * height_final;
%             end
%         end
% 
%         % Add labels and legend
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%         legend(proxy_handles, method_names, 'Location', 'southeast');
% 
%         % Display volumes
%         fprintf('Volume of the Last Reachable Set for %s vs. %s plane:\n', ...
%                 ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
%         for m = 1:n_methods
%             fprintf('Method %s: %.6f\n', method_names{m}, volumes(m));
%         end
% 
%         warning('Plotting is not implemented for systems with %d states. Update plot_pairs for this dimension.', n_x);
%     end
% 
% % Ensure save operation occurs regardless of plotting
% timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
% save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods');
% disp(['Data saved to ' filename_base '.mat']);

% elseif n_x == 5
%     plot_pairs = [1 2; 3 4; 4 5];
%     colors = lines(n_methods);
%     method_names = {'CT Contraction and Growth Bound', 'CT Mixed-Monotonicity', ...
%                    'CT Sampled-Data Mixed-Monotonicity', 'CT Monte Carlo'}; % Updated for 4 methods
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         x_min_actual = min(rand_succ(idx1, :));
%         x_max_actual = max(rand_succ(idx1, :));
%         y_min_actual = min(rand_succ(idx2, :));
%         y_max_actual = max(rand_succ(idx2, :));
%         area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);
% 
%         handles = [];
%         proxy_handles = [];
%         for m = 1:n_methods
%             h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%             proxy_handles = [proxy_handles, h_proxy];
% 
%             if plot_last_only
%                 k = n_t;
%                 x_min = succ_low_all_methods(idx1, k, m);
%                 y_min = succ_low_all_methods(idx2, k, m);
%                 width = succ_up_all_methods(idx1, k, m) - x_min;
%                 height = succ_up_all_methods(idx2, k, m) - y_min;
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%             else
%                 for k = 1:n_t
%                     x_min = succ_low_all_methods(idx1, k, m);
%                     y_min = succ_low_all_methods(idx2, k, m);
%                     width = succ_up_all_methods(idx1, k, m) - x_min;
%                     height = succ_up_all_methods(idx2, k, m) - y_min;
%                     h = rectangle('Position', [x_min, y_min, width, height], ...
%                                   'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                     handles = [handles, h];
%                 end
%             end
%             x_min_final = succ_low_all_methods(idx1, end, m);
%             y_min_final = succ_low_all_methods(idx2, end, m);
%             width_final = succ_up_all_methods(idx1, end, m) - x_min_final;
%             height_final = succ_up_all_methods(idx2, end, m) - y_min_final;
%             area_overapprox = width_final * height_final;
%             tightness_metrics(m) = area_overapprox / area_actual;
%         end
% 
%         xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%         title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%         legend(proxy_handles, method_names, 'Location', 'southeast');
% 
%         fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
%                 ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
%         for m = 1:n_methods
%             fprintf('Method %s: %.2f\n', method_names{m}, tightness_metrics(m));
%         end
%     end
% else
%     warning('Plotting is not implemented for systems with %d states. Update plot_pairs for this dimension.', n_x);
% end
% 
% % Ensure save operation occurs regardless of plotting
% timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
% save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods');
% disp(['Data saved to ' filename_base '.mat']);

%% Based on PJ comment for the angle of the spiral

% %% Initialization
% close all
% clear
% bool_discrete_time = 0;
% % Folder containing the various over-approximation methods
% addpath('../OA_methods')
% % Folder containing useful tools and functions
% addpath('../Utilities')    
% 
% %% User-defined approach and plotting selection
% % Set approach to 'initial' for the initial step approach or 'incremental' for the incremental step approach
% approach = 'incremental'; % Options: 'initial' or 'incremental'
% % Set plot_last_only to true to plot only the last reachable set, false for full reachable sets
% plot_last_only = true; % Options: true or false
% 
% %% Choice of the example system
% global system_choice
% system_choice = 15;     % Spiral_non (continuous-time)
% 
% %% Optional submethod choice for the sampled-data mixed-monotonicity method
% global g_sensitivity_bounds_method
% g_sensitivity_bounds_method = nan;
% 
% %% Definition of the reachability problem (for each example system)
% global u        % Some systems need an external control input
% switch system_choice
%     case 15
%         %% Spiral_non System (continuous-time)
%         Initial_radius = 0.2; % Uncertainty in dynamics
%         x0 = [2.0; 0.0];
%         x_low = x0 - Initial_radius;
%         x_up = x0 + Initial_radius;
% 
%         % Define time and input bounds
%         t_init = 0;     % Initial time
%         t_final = 2;    % Final time
%         steps = 0.01;   % Discretization steps
%         t_points = t_init:steps:t_final;    % Time points
%         p_low = zeros(2,1); 
%         p_up = zeros(2,1);
% end
% 
% % State and input dimensions
% n_x = length(x_low);
% n_p = length(p_low);
% n_t = length(t_points);
% 
% % Define over-approximation methods to compare
% methods = [3, 4]; % 3: Continuous-time mixed-monotonicity, 4: Continuous-time sampled-data mixed-monotonicity
% n_methods = length(methods);
% succ_low_all_methods = NaN(n_x, n_t, n_methods);
% succ_up_all_methods = NaN(n_x, n_t, n_methods);
% 
% %% Call of the main over-approximation function
% if bool_discrete_time
%     [succ_low,succ_up] = TIRA(t_init,x_low,x_up,p_low,p_up);
% else
%     if strcmp(approach, 'initial')
%         for m = 1:n_methods
%             for k = 1:n_t
%                 t_final_k = t_points(k);
%                 [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
%                 succ_low_all_methods(:, k, m) = succ_low;
%                 succ_up_all_methods(:, k, m) = succ_up;
%             end
%         end
%     elseif strcmp(approach, 'incremental')
%         for m = 1:n_methods
%             succ_low_all_methods(:,1,m) = x_low;
%             succ_up_all_methods(:,1,m) = x_up;
%             for k = 1:n_t-1
%                 t_start = t_points(k);
%                 t_end = t_points(k+1);
%                 current_low = succ_low_all_methods(:,k,m);
%                 current_up = succ_up_all_methods(:,k,m);
%                 [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
%                 succ_low_all_methods(:,k+1,m) = succ_low;
%                 succ_up_all_methods(:,k+1,m) = succ_up;
%             end
%         end
%     else
%         error('Invalid approach. Use ''initial'' or ''incremental''.');
%     end
% end
% 
% %% Compute successors from random initial states and disturbances
% sample_succ_number = 1000;
% log = logger.get_logger();
% 
% % Extract ODE solver to use 
% run('Solver_parameters.m');
% ode_solver = parameters.ode_solver;
% ode_options = parameters.ode_options;
% 
% log.info('Compute successors from %d random initial states and parameters ...\n', sample_succ_number)
% t_start = tic;
% rand_succ = NaN(n_x,sample_succ_number);
% for i = 1:sample_succ_number
%     x0 = x_low + rand(n_x,1).*(x_up-x_low);
%     p = p_low + rand(n_p,1).*(p_up-p_low);
%     if bool_discrete_time
%         rand_succ(:,i) = System_description(t_init,x0,p);
%     else
%         [~,x_traj] = ode_solver(@(t,x) System_description(t,x,p),[t_init t_final],x0,ode_options);
%         rand_succ(:,i) = x_traj(end,:)';
%     end
% end
% t_rand_succs = toc(t_start);
% log.runtime('Time to generate random successors: %f seconds\n', t_rand_succs);
% 
% %% Plot the reachable set and over-approximations for the 2D case of the spiral
% if n_x == 2
%     plot_pairs = [1 2]; % Plot x1 vs x2 for 2D system
%     colors = lines(n_methods);
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         % Plot the successors from random initial states
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Calculate minimum bounding box for black dots
%         x_min_actual = min(rand_succ(idx1, :));
%         x_max_actual = max(rand_succ(idx1, :));
%         y_min_actual = min(rand_succ(idx2, :));
%         y_max_actual = max(rand_succ(idx2, :));
%         area_actual = (x_max_actual - x_min_actual) * (y_max_actual - y_min_actual);
% 
%         % Plot over-approximation rectangles for each TIRA method
%         handles = [];
%         proxy_handles = [];
%         volumes = zeros(n_methods, 1);
%         for m = 1:n_methods
%             h_proxy = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%             proxy_handles = [proxy_handles, h_proxy];
% 
%             if plot_last_only
%                 k = n_t;
%                 x_min = succ_low_all_methods(idx1, k, m);
%                 y_min = succ_low_all_methods(idx2, k, m);
%                 width = succ_up_all_methods(idx1, k, m) - x_min;
%                 height = succ_up_all_methods(idx2, k, m) - y_min;
%                 % h stondard deviation = rectangle('Position', [x_min, y_min, width, height], ...
%                 %               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 h_standard_deviation = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h_standard_deviation];
%                 volumes(m) = width * height; % Approximate volume as bounding box area
%             else
%                 for k = 1:n_t
%                     x_min = succ_low_all_methods(idx1, k, m);
%                     y_min = succ_low_all_methods(idx2, k, m);
%                     width = succ_up_all_methods(idx1, k, m) - x_min;
%                     height = succ_up_all_methods(idx2, k, m) - y_min;
%                     h = rectangle('Position', [x_min, y_min, width, height], ...
%                                   'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                     handles = [handles, h];
%                 end
%                 x_min_final = succ_low_all_methods(idx1, end, m);
%                 y_min_final = succ_low_all_methods(idx2, end, m);
%                 width_final = succ_up_all_methods(idx1, end, m) - x_min_final;
%                 height_final = succ_up_all_methods(idx2, end, m) - y_min_final;
%                 volumes(m) = width_final * height_final;
%             end
%         end
% 
%         % Compute PCA-based rotated bounding box for tighter over-approximation
%         if plot_last_only
%             % Use the final random successors
%             mean_succ = mean(rand_succ, 2);
%             centered_succ = rand_succ - mean_succ;
%             cov_matrix = cov(centered_succ');
%             [V, ~] = eig(cov_matrix);
%             proj = V' * centered_succ;
%             min_proj = min(proj, [], 2);
%             max_proj = max(proj, [], 2);
%             % Define corners in principal coordinates
%             corners_principal = [min_proj(1), min_proj(2);
%                                  max_proj(1), min_proj(2);
%                                  max_proj(1), max_proj(2);
%                                  min_proj(1), max_proj(2);
%                                  min_proj(1), min_proj(2)]';
%             % Transform back to original coordinates
%             corners_original = V * corners_principal + mean_succ;
%             % Plot the rotated box in black (as per your query)
%             plot(corners_original(1,:), corners_original(2,:), 'k-', 'LineWidth', 2);
%             % Compute area of rotated box
%             area_rotated = (max_proj(1) - min_proj(1)) * (max_proj(2) - min_proj(2));
%             fprintf('Rotated bounding box area: %.6f\n', area_rotated);
%             % Compare with TIRA areas
%             for m = 1:n_methods
%                 fprintf('Method %s: TIRA area = %.6f, Ratio to rotated = %.2f\n', ...
%                         method_names{m}, volumes(m), volumes(m) / area_rotated);
%             end
%         end
% 
%         % Add labels and legend
%         xlabel('$x_1$', 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel('$x_2$', 'Interpreter', 'Latex', 'FontSize', 20);
%         title('$x_1$ vs. $x_2$', 'Interpreter', 'Latex', 'FontSize', 16);
%         % Update legend to include rotated box
%         proxy_handles(end+1) = plot(nan, nan, 'k-', 'LineWidth', 2, 'Visible', 'off');
%         method_names{end+1} = 'Rotated Bounding Box';
%         legend(proxy_handles, method_names, 'Location', 'southeast');
% 
%         % Display volumes
%         fprintf('Volume of the Last Reachable Set for x_%d vs. x_%d plane:\n', idx1, idx2);
%         for m = 1:n_methods
%             fprintf('Method %s: %.6f\n', method_names{m}, volumes(m));
%         end
%     end
% 
%     % Ensure save operation occurs
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
%     save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods');
%     disp(['Data saved to ' filename_base '.mat']);
% else
%     warning('Plotting is not implemented for systems with %d states. Update plot_pairs for this dimension.', n_x);
% end

% %% Initialization
% close all
% clear
% bool_discrete_time = 0;
% % Folder containing the various over-approximation methods
% addpath('../OA_methods')
% % Folder containing useful tools and functions
% addpath('../Utilities')    
% 
% %% User-defined approach and plotting selection
% approach = 'initial'; % Options: 'initial' or 'incremental'
% plot_last_only = true; % Options: true or false
% 
% %% Choice of the example system
% global system_choice rotated R
% system_choice = 15;     % Spiral_non (continuous-time)
% rotated = false;
% 
% %% Optional submethod choice for the sampled-data mixed-monotonicity method
% global g_sensitivity_bounds_method
% g_sensitivity_bounds_method = nan;
% 
% %% Definition of the reachability problem
% global u
% switch system_choice
%     case 15
%         %% Spiral_non System (continuous-time)
%         Initial_radius = 0.2;
%         x0 = [2.0; 0.0];
%         x_low = x0 - Initial_radius;
%         x_up = x0 + Initial_radius;
%         t_init = 0;
%         t_final = 2;
%         steps = 0.01;
%         t_points = t_init:steps:t_final;
%         p_low = zeros(2,1); 
%         p_up = zeros(2,1);
% end
% 
% % State and input dimensions
% n_x = length(x_low);
% n_p = length(p_low);
% n_t = length(t_points);
% 
% % Define over-approximation methods to compare
% methods = [3, 4]; % 3: CT Mixed-Monotonicity, 4: CT Sampled-Data Mixed-Monotonicity
% n_methods = length(methods);
% succ_low_all_methods = NaN(n_x, n_t, n_methods);
% succ_up_all_methods = NaN(n_x, n_t, n_methods);
% 
% %% Compute successors from random initial states and disturbances
% sample_succ_number = 1000;
% log = logger.get_logger();
% run('Solver_parameters.m');
% ode_solver = parameters.ode_solver;
% ode_options = parameters.ode_options;
% 
% log.info('Compute successors from %d random initial states and parameters ...\n', sample_succ_number)
% t_start = tic;
% rand_succ = NaN(n_x,sample_succ_number);
% for i = 1:sample_succ_number
%     x0 = x_low + rand(n_x,1).*(x_up-x_low);
%     p = p_low + rand(n_p,1).*(p_up-p_low);
%     if bool_discrete_time
%         rand_succ(:,i) = System_description(t_init,x0,p);
%     else
%         [~,x_traj] = ode_solver(@(t,x) System_description(t,x,p),[t_init t_final],x0,ode_options);
%         rand_succ(:,i) = x_traj(end,:)';
%     end
% end
% t_rand_succs = toc(t_start);
% log.runtime('Time to generate random successors: %f seconds\n', t_rand_succs);
% 
% % Compute rotation matrix R from PCA of rand_succ at t=2
% mean_succ = mean(rand_succ, 2);
% centered_succ = rand_succ - mean_succ;
% cov_matrix = cov(centered_succ');
% [V, ~] = eig(cov_matrix);
% R = V'; % Rotation matrix aligning principal axes with coordinate axes
% 
% % Compute rotated initial set
% corners_initial = [x_low(1), x_low(1), x_up(1), x_up(1);
%                    x_low(2), x_up(2), x_low(2), x_up(2)];
% corners_rot = R * corners_initial;
% x_low_rot = min(corners_rot, [], 2);
% x_up_rot = max(corners_rot, [], 2);
% 
% %% Call of the main over-approximation function with rotation
% rotated = true;
% if bool_discrete_time
%     [succ_low,succ_up] = TIRA(t_init,x_low_rot,x_up_rot,p_low,p_up);
% else
%     if strcmp(approach, 'initial')
%         for m = 1:n_methods
%             for k = 1:n_t
%                 t_final_k = t_points(k);
%                 [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low_rot, x_up_rot, p_low, p_up, methods(m));
%                 succ_low_all_methods(:, k, m) = succ_low;
%                 succ_up_all_methods(:, k, m) = succ_up;
%             end
%         end
%     elseif strcmp(approach, 'incremental')
%         for m = 1:n_methods
%             succ_low_all_methods(:,1,m) = x_low_rot;
%             succ_up_all_methods(:,1,m) = x_up_rot;
%             for k = 1:n_t-1
%                 t_start = t_points(k);
%                 t_end = t_points(k+1);
%                 current_low = succ_low_all_methods(:,k,m);
%                 current_up = succ_up_all_methods(:,k,m);
%                 [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
%                 succ_low_all_methods(:,k+1,m) = succ_low;
%                 succ_up_all_methods(:,k+1,m) = succ_up;
%             end
%         end
%     else
%         error('Invalid approach. Use ''initial'' or ''incremental''.');
%     end
% end
% rotated = false;
% 
% %% Plot the reachable set and over-approxim([])
% if n_x == 2
%     plot_pairs = [1 2];
%     colors = lines(n_methods);
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     for i = 1:size(plot_pairs, 1)
%         figure
%         hold on
%         grid on
%         idx1 = plot_pairs(i, 1);
%         idx2 = plot_pairs(i, 2);
% 
%         % Plot successors from random initial states
%         for j = 1:sample_succ_number
%             plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%         end
% 
%         % Plot rotated over-approximation boxes
%         for m = 1:n_methods
%             if plot_last_only
%                 k = n_t;
%                 succ_low = succ_low_all_methods(:, k, m);
%                 succ_up = succ_up_all_methods(:, k, m);
%                 % Compute corners in rotated space
%                 c1 = [succ_low(1); succ_low(2)];
%                 c2 = [succ_low(1); succ_up(2)];
%                 c3 = [succ_up(1); succ_low(2)];
%                 c4 = [succ_up(1); succ_up(2)];
%                 % Rotate back to original space
%                 c_original = R' * [c1, c2, c3, c4];
% 95                % Plot the polygon
%                 plot(c_original(1,[1 2 4 3 1]), c_original(2,[1 2 4 3 1]), 'Color', colors(m, :), 'LineWidth', 1.5);
%                 % Compute and display area
%                 area_overapprox = prod(succ_up - succ_low);
%                 fprintf('Method %s: Area of rotated over-approximation = %.6f\n', method_names{m}, area_overapprox);
%             else
%                 for k = 1:n_t
%                     succ_low = succ_low_all_methods(:, k, m);
%                     succ_up = succ_up_all_methods(:, k, m);
%                     c1 = [succ_low(1); succ_low(2)];
%                     c2 = [succ_low(1); succ_up(2)];
%                     c3 = [succ_up(1); succ_low(2)];
%                     c4 = [succ_up(1); succ_up(2)];
%                     c_original = R' * [c1, c2, c3, c4];
%                     plot(c_original(1,[1 2 4 3 1]), c_original(2,[1 2 4 3 1]), 'Color', colors(m, :), 'LineWidth', 1.5);
%                 end
%             end
%         end
% 
%         % Add labels and legend
%         xlabel('$x_1$', 'Interpreter', 'Latex', 'FontSize', 20);
%         ylabel('$x_2$', 'Interpreter', 'Latex', 'FontSize', 20);
%         title('$x_1$ vs. $x_2$', 'Interpreter', 'Latex', 'FontSize', 16);
%         legend(method_names, 'Location', 'southeast');
%     end
% 
%     % Save data
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     filename_base = sprintf('reachability_data_sys_TIRA%d_%s_%s', system_choice, approach, timestamp);
%     save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', 'R');
%     disp(['Data saved to ' filename_base '.mat']);
% else
%     warning('Plotting not implemented for systems with %d states.', n_x);
% end