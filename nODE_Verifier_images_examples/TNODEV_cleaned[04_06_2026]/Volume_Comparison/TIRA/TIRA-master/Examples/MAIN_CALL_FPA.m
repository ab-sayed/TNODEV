% Paper:
%   "Add the paper name here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 13th of August 2025
% Last update: xxth of August 2025
% Last revision: 

%------------- BEGIN CODE --------------


%% Boundary Analysis --> [Working code for Spiral as of 13/08 for plotting Boundary analysis vs single-step~'Full Reachable' set vs incremental approach]

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
system_choice = 14;      % FPA (continuous-time)
% system_choice = 15;     % Spiral_non (continuous-time)

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

% Verify dimensions
if n_x ~= 5 || n_p ~= 5
    error('Expected n_x=5 and n_p=5 for FPA, got n_x=%d, n_p=%d', n_x, n_p);
end
if ~isvector(x_low) || length(x_low) ~= 5 || ~isvector(x_up) || length(x_up) ~= 5
    error('Initial set bounds must be 5x1, got x_low=%s, x_up=%s', mat2str(size(x_low)), mat2str(size(x_up)));
end
if ~isvector(p_low) || length(p_low) ~= 5 || ~isvector(p_up) || length(p_up) ~= 5
    error('Input bounds must be 5x1, got p_low=%s, p_up=%s', mat2str(size(p_low)), mat2str(size(p_up)));
end

% Define over-approximation methods
methods = [3, 4]; % 3: Continuous-time mixed-monotonicity, 4: Continuous-time sampled-data mixed-monotonicity
n_methods = length(methods);
succ_low_all_methods = NaN(n_x, n_t, n_methods); % For initial approach
succ_up_all_methods = NaN(n_x, n_t, n_methods);
succ_low_inc_methods = NaN(n_x, n_t, n_methods); % For incremental approach
succ_up_inc_methods = NaN(n_x, n_t, n_methods);

%% Boundary reachability parameters
SplitsB = 1; % Number of splits per dimension for boundary reachability
num_splits = 2 * n_x * SplitsB; % Total number of boundary splits (10 for 5D)
R0B_splits = cell(num_splits, 1); % Boundary splits
succ_low_boundary = cell(num_splits, 1); % Lower bounds for boundary splits
succ_up_boundary = cell(num_splits, 1); % Upper bounds for boundary splits
boundary_times = zeros(num_splits, n_methods); % Computation times for boundary splits

%% Call of the main over-approximation function

try
    if bool_discrete_time
        [succ_low, succ_up] = TIRA(t_init, x_low, x_up, p_low, p_up);
    else
        time_initial = zeros(1, n_methods);
        time_incremental = zeros(1, n_methods);
        time_boundary = zeros(1, n_methods); % Single time per method for boundary

        % Initial approach: Compute reachable sets from t_init to each t
        for m = 1:n_methods
            tStart = tic;
            for k = 1:n_t
                t_final_k = t_points(k);
                fprintf('Computing initial approach, method %d, time %.2f, x_low=%s, x_up=%s\n', ...
                        methods(m), t_final_k, mat2str(x_low), mat2str(x_up));
                if t_final_k == t_final
                    [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
                else
                    [succ_low, succ_up] = TIRA([t_init, t_final_k], x_low, x_up, p_low, p_up, methods(m));
                end
                if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                    error('TIRA returned invalid dimensions for initial approach, method %d, time %.2f: succ_low=%s, succ_up=%s', ...
                          methods(m), t_final_k, mat2str(size(succ_low)), mat2str(size(succ_up)));
                end
                succ_low_all_methods(:, k, m) = succ_low;
                succ_up_all_methods(:, k, m) = succ_up;
            end
            time_initial(m) = toc(tStart);
        end

        % Incremental approach: Compute reachable sets step-by-step
        for m = 1:n_methods
            tStart = tic;
            succ_low_inc_methods(:,1,m) = x_low;
            succ_up_inc_methods(:,1,m) = x_up;
            for k = 1:n_t-1
                t_start = t_points(k);
                t_end = t_points(k+1);
                current_low = succ_low_inc_methods(:,k,m);
                current_up = succ_up_inc_methods(:,k,m);
                fprintf('Computing incremental approach, method %d, time %.2f to %.2f, x_low=%s, x_up=%s\n', ...
                        methods(m), t_start, t_end, mat2str(current_low), mat2str(current_up));
                if t_end == t_final
                    [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
                else
                    [succ_low, succ_up] = TIRA([t_start, t_end], current_low, current_up, p_low, p_up, methods(m));
                end
                if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                    error('TIRA returned invalid dimensions for incremental approach, method %d, time %.2f to %.2f: succ_low=%s, succ_up=%s', ...
                          methods(m), t_start, t_end, mat2str(size(succ_low)), mat2str(size(succ_up)));
                end
                succ_low_inc_methods(:,k+1,m) = succ_low;
                succ_up_inc_methods(:,k+1,m) = succ_up;
            end
            time_incremental(m) = toc(tStart);
        end

        % Boundary approach: Compute reachable sets for boundary splits
        count = 1;
        intervalStep = (x_up - x_low) / SplitsB;
        for i = 1:n_x
            % Upper boundary for dimension i
            mintemp = x_low;
            maxtemp = x_up;
            mintemp(i) = x_up(i);
            for j = 1:SplitsB
                lbsplit = mintemp;
                ubsplit = maxtemp;
                for d = 1:n_x
                    if d ~= i
                        lbsplit(d) = x_low(d) + (j-1) * intervalStep(d);
                        ubsplit(d) = x_low(d) + j * intervalStep(d);
                    end
                end
                if ~isvector(lbsplit) || length(lbsplit) ~= n_x || ~isvector(ubsplit) || length(ubsplit) ~= n_x
                    error('Invalid boundary split %d dimensions: lbsplit=%s, ubsplit=%s', count, mat2str(size(lbsplit)), mat2str(size(ubsplit)));
                end
                R0B_splits{count} = interval(lbsplit(:), ubsplit(:)); % Ensure column vectors
                fprintf('Boundary split %d: inf=%s, sup=%s\n', count, mat2str(R0B_splits{count}.inf), mat2str(R0B_splits{count}.sup));
                count = count + 1;
            end
        end
        for i = 1:n_x
            % Lower boundary for dimension i
            mintemp = x_low;
            maxtemp = x_up;
            maxtemp(i) = x_low(i);
            for j = 1:SplitsB
                lbsplit = mintemp;
                ubsplit = maxtemp;
                for d = 1:n_x
                    if d ~= i
                        lbsplit(d) = x_low(d) + (j-1) * intervalStep(d);
                        ubsplit(d) = x_low(d) + j * intervalStep(d);
                    end
                end
                if ~isvector(lbsplit) || length(lbsplit) ~= n_x || ~isvector(ubsplit) || length(ubsplit) ~= n_x
                    error('Invalid boundary split %d dimensions: lbsplit=%s, ubsplit=%s', count, mat2str(size(lbsplit)), mat2str(size(ubsplit)));
                end
                R0B_splits{count} = interval(lbsplit(:), ubsplit(:)); % Ensure column vectors
                fprintf('Boundary split %d: inf=%s, sup=%s\n', count, mat2str(R0B_splits{count}.inf), mat2str(R0B_splits{count}.sup));
                count = count + 1;
            end
        end

        % Compute reachable sets for each boundary split
        for m = 1:n_methods
            tStart = tic;
            for i = 1:num_splits
                succ_low_boundary{i} = NaN(n_x, n_t, n_methods);
                succ_up_boundary{i} = NaN(n_x, n_t, n_methods);
                if plot_last_only
                    fprintf('Computing boundary split %d, method %d, time %.2f, x_low=%s, x_up=%s\n', ...
                            i, methods(m), t_final, mat2str(R0B_splits{i}.inf), mat2str(R0B_splits{i}.sup));
                    if ~isvector(R0B_splits{i}.inf) || length(R0B_splits{i}.inf) ~= n_x || ...
                       ~isvector(R0B_splits{i}.sup) || length(R0B_splits{i}.sup) ~= n_x
                        error('Invalid input dimensions for boundary split %d: x_low=%s, x_up=%s', ...
                              i, mat2str(size(R0B_splits{i}.inf)), mat2str(size(R0B_splits{i}.sup)));
                    end
                    [succ_low, succ_up] = TIRA([t_init, t_final], R0B_splits{i}.inf, R0B_splits{i}.sup, p_low, p_up, methods(m));
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
                                i, methods(m), t_final_k, mat2str(R0B_splits{i}.inf), mat2str(R0B_splits{i}.sup));
                        if ~isvector(R0B_splits{i}.inf) || length(R0B_splits{i}.inf) ~= n_x || ...
                           ~isvector(R0B_splits{i}.sup) || length(R0B_splits{i}.sup) ~= n_x
                            error('Invalid input dimensions for boundary split %d: x_low=%s, x_up=%s', ...
                                  i, mat2str(size(R0B_splits{i}.inf)), mat2str(size(R0B_splits{i}.sup)));
                        end
                        if t_final_k == t_final
                            [succ_low, succ_up] = TIRA([t_init, t_final_k], R0B_splits{i}.inf, R0B_splits{i}.sup, p_low, p_up, methods(m));
                        else
                            [succ_low, succ_up] = TIRA([t_init, t_final_k], R0B_splits{i}.inf, R0B_splits{i}.sup, p_low, p_up, methods(m));
                        end
                        if ~isvector(succ_low) || length(succ_low) ~= n_x || ~isvector(succ_up) || length(succ_up) ~= n_x
                            error('TIRA returned invalid dimensions for boundary split %d, method %d, time %.2f: succ_low=%s, succ_up=%s', ...
                                  i, methods(m), t_final_k, mat2str(size(succ_low)), mat2str(size(succ_up)));
                        end
                        succ_low_boundary{i}(:, k, m) = succ_low;
                        succ_up_boundary{i}(:, k, m) = succ_up;
                    end
                end
            end
            time_boundary(m) = toc(tStart) / num_splits; % Average time per split
            fprintf('Boundary Split computation for method %d completed, total time: %.6f seconds\n', methods(m), time_boundary(m) * num_splits);
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
            fprintf('Computing random successor %d, x0=%s, p=%s\n', i, mat2str(x0), mat2str(p));
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
    % Define pairs of dimensions to plot for 5D system
    plot_pairs = [[1 2]; [3 4]; [4 5]]; % 3 separate figures
    colors = [0 0 1; 1 0 0; 0 1 1; 1 1 0]; % Dark Blue, Red, Cyan (Light Blue), Yellow
    method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};

    % Initialize tightness metrics arrays
    tightness_metrics = NaN(n_methods, size(plot_pairs, 1));
    tightness_metrics_inc = NaN(n_methods, size(plot_pairs, 1));
    tightness_metrics_bound = NaN(n_methods, size(plot_pairs, 1));

    % Figure 1: x1 vs x2
    figure
    hold on
    grid on
    idx1 = plot_pairs(1, 1);
    idx2 = plot_pairs(1, 2);

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

    for m = 1:n_methods
        % Proxy handles for initial, incremental, and boundary approaches
        h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        if m == 1
            h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
        else
            h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
        end
        proxy_handles = [proxy_handles, h_proxy_init, h_proxy_inc, h_proxy_bound];
    end

    for m = 1:n_methods
        % Initial approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_all_methods(idx1, k, m);
            y_min = succ_low_all_methods(idx2, k, m);
            width = succ_up_all_methods(idx1, k, m) - x_min;
            height = succ_up_all_methods(idx2, k, m) - y_min;
            fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_overapprox = width * height;
                tightness_metrics(m, 1) = area_overapprox / area_actual;
            else
                warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Incremental approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_inc_methods(idx1, k, m);
            y_min = succ_low_inc_methods(idx2, k, m);
            width = succ_up_inc_methods(idx1, k, m) - x_min;
            height = succ_up_inc_methods(idx2, k, m) - y_min;
            fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_inc = width * height;
                tightness_metrics_inc(m, 1) = area_inc / area_actual;
            else
                warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Boundary approach
        if plot_last_only
            k = n_t;
            x_min_bound = succ_low_boundary{1}(idx1, k, m);
            y_min_bound = succ_low_boundary{1}(idx2, k, m);
            x_max_bound = succ_up_boundary{1}(idx1, k, m);
            y_max_bound = succ_up_boundary{1}(idx2, k, m);
            for j = 2:num_splits
                x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
                y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
                x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
                y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
            end
            width_bound = x_max_bound - x_min_bound;
            height_bound = y_max_bound - y_min_bound;
            fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
                if m == 1
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
                else
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
                end
                handles = [handles, h_bound];
                area_bound = width_bound * height_bound;
                tightness_metrics_bound(m, 1) = area_bound / area_actual;
            else
                warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            end
        end
    end

    % Add labels and legend for Figure 1
    xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
    all_proxy_handles = [proxy_handles(3), proxy_handles(6), proxy_handles(1), proxy_handles(4), proxy_handles(2), proxy_handles(5)];
    all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
                        'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
                        'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
    lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
    set(lgd, 'FontSize', 9, 'FontWeight', 'bold');

    % Figure 2: x3 vs x4
    figure
    hold on
    grid on
    idx1 = plot_pairs(2, 1);
    idx2 = plot_pairs(2, 2);

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

    for m = 1:n_methods
        % Proxy handles for initial, incremental, and boundary approaches
        h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        if m == 1
            h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
        else
            h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
        end
        proxy_handles = [proxy_handles, h_proxy_init, h_proxy_inc, h_proxy_bound];
    end

    for m = 1:n_methods
        % Initial approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_all_methods(idx1, k, m);
            y_min = succ_low_all_methods(idx2, k, m);
            width = succ_up_all_methods(idx1, k, m) - x_min;
            height = succ_up_all_methods(idx2, k, m) - y_min;
            fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_overapprox = width * height;
                tightness_metrics(m, 2) = area_overapprox / area_actual;
            else
                warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Incremental approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_inc_methods(idx1, k, m);
            y_min = succ_low_inc_methods(idx2, k, m);
            width = succ_up_inc_methods(idx1, k, m) - x_min;
            height = succ_up_inc_methods(idx2, k, m) - y_min;
            fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_inc = width * height;
                tightness_metrics_inc(m, 2) = area_inc / area_actual;
            else
                warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Boundary approach
        if plot_last_only
            k = n_t;
            x_min_bound = succ_low_boundary{1}(idx1, k, m);
            y_min_bound = succ_low_boundary{1}(idx2, k, m);
            x_max_bound = succ_up_boundary{1}(idx1, k, m);
            y_max_bound = succ_up_boundary{1}(idx2, k, m);
            for j = 2:num_splits
                x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
                y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
                x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
                y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
            end
            width_bound = x_max_bound - x_min_bound;
            height_bound = y_max_bound - y_min_bound;
            fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
                if m == 1
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
                else
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
                end
                handles = [handles, h_bound];
                area_bound = width_bound * height_bound;
                tightness_metrics_bound(m, 2) = area_bound / area_actual;
            else
                warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            end
        end
    end

    % Add labels and legend for Figure 2
    xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
    all_proxy_handles = [proxy_handles(3), proxy_handles(6), proxy_handles(1), proxy_handles(4), proxy_handles(2), proxy_handles(5)];
    all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
                        'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
                        'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
    lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
    set(lgd, 'FontSize', 9, 'FontWeight', 'bold');

    % Figure 3: x4 vs x5
    figure
    hold on
    grid on
    idx1 = plot_pairs(3, 1);
    idx2 = plot_pairs(3, 2);

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

    for m = 1:n_methods
        % Proxy handles for initial, incremental, and boundary approaches
        h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
        if m == 1
            h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
        else
            h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
        end
        proxy_handles = [proxy_handles, h_proxy_init, h_proxy_inc, h_proxy_bound];
    end

    for m = 1:n_methods
        % Initial approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_all_methods(idx1, k, m);
            y_min = succ_low_all_methods(idx2, k, m);
            width = succ_up_all_methods(idx1, k, m) - x_min;
            height = succ_up_all_methods(idx2, k, m) - y_min;
            fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_overapprox = width * height;
                tightness_metrics(m, 3) = area_overapprox / area_actual;
            else
                warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Incremental approach
        if plot_last_only
            k = n_t;
            x_min = succ_low_inc_methods(idx1, k, m);
            y_min = succ_low_inc_methods(idx2, k, m);
            width = succ_up_inc_methods(idx1, k, m) - x_min;
            height = succ_up_inc_methods(idx2, k, m) - y_min;
            fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min, y_min, width, height);
            if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
                h = rectangle('Position', [x_min, y_min, width, height], ...
                              'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
                handles = [handles, h];
                area_inc = width * height;
                tightness_metrics_inc(m, 3) = area_inc / area_actual;
            else
                warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min, y_min, width, height);
            end
        end

        % Boundary approach
        if plot_last_only
            k = n_t;
            x_min_bound = succ_low_boundary{1}(idx1, k, m);
            y_min_bound = succ_low_boundary{1}(idx2, k, m);
            x_max_bound = succ_up_boundary{1}(idx1, k, m);
            y_max_bound = succ_up_boundary{1}(idx2, k, m);
            for j = 2:num_splits
                x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
                y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
                x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
                y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
            end
            width_bound = x_max_bound - x_min_bound;
            height_bound = y_max_bound - y_min_bound;
            fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
                    m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
                if m == 1
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
                else
                    h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
                                      'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
                end
                handles = [handles, h_bound];
                area_bound = width_bound * height_bound;
                tightness_metrics_bound(m, 3) = area_bound / area_actual;
            else
                warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
                        m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
            end
        end
    end

    % Add labels and legend for Figure 3
    xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
    title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
    all_proxy_handles = [proxy_handles(3), proxy_handles(6), proxy_handles(1), proxy_handles(4), proxy_handles(2), proxy_handles(5)];
    all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
                        'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
                        'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
    lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
    set(lgd, 'FontSize', 9, 'FontWeight', 'bold');

    % Save the figures
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    save_dir = fullfile(pwd, 'figures');
    fprintf('Attempting to save figures to: %s\n', save_dir);
    if ~exist(save_dir, 'dir')
        [status, msg] = mkdir(save_dir);
        if status ~= 1
            warning('Failed to create directory %s: %s', save_dir, msg);
        else
            fprintf('Created directory: %s\n', save_dir);
        end
    end
    if exist(save_dir, 'dir')
        try
            saveas(figure(1), fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.eps', timestamp)), 'epsc');
            fprintf('Figure 1 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.eps', timestamp)));
            saveas(figure(2), fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.eps', timestamp)), 'epsc');
            fprintf('Figure 2 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.eps', timestamp)));
            saveas(figure(3), fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.eps', timestamp)), 'epsc');
            fprintf('Figure 3 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.eps', timestamp)));
        catch e
            warning('Failed to save figures as EPS: %s. Trying PNG format...', e.message);
            saveas(figure(1), fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.png', timestamp)), 'png');
            fprintf('Figure 1 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.png', timestamp)));
            saveas(figure(2), fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.png', timestamp)), 'png');
            fprintf('Figure 2 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.png', timestamp)));
            saveas(figure(3), fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.png', timestamp)), 'png');
            fprintf('Figure 3 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.png', timestamp)));
        end
    end

    % Display tightness metrics
    fprintf('Tightness Metric (Area Ratio) for FPA 5D plane projections:\n');
    for p = 1:size(plot_pairs, 1)
        idx1 = plot_pairs(p, 1);
        idx2 = plot_pairs(p, 2);
        fprintf('Pair [%d %d]:\n', idx1, idx2);
        for m = 1:n_methods
            fprintf('Method %s (Initial): %.4f\n', method_names{m}, tightness_metrics(m, p));
            fprintf('Method %s (Incremental): %.4f\n', method_names{m}, tightness_metrics_inc(m, p));
            fprintf('Method %s (Boundary): %.4f\n', method_names{m}, tightness_metrics_bound(m, p));
        end
    end

catch e
    fprintf('Error in plotting: %s\n', e.message);
    rethrow(e);
end




% try
%     % Define pairs of dimensions to plot for 5D system
%     plot_pairs = [[1 2]; [3 4]; [4 5]]; % 3 separate figures
%     colors = [0 0 1; 1 0 0; 0 1 1; 1 1 0]; % Dark Blue, Red, Cyan (Light Blue), Yellow
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     % Initialize tightness metrics arrays
%     tightness_metrics = NaN(n_methods, size(plot_pairs, 1));
%     tightness_metrics_inc = NaN(n_methods, size(plot_pairs, 1));
%     tightness_metrics_bound = NaN(n_methods, size(plot_pairs, 1));
% 
%     % Figure 1: x1 vs x2
%     figure
%     hold on
%     grid on
%     idx1 = plot_pairs(1, 1);
%     idx2 = plot_pairs(1, 2);
% 
%     % Plot the successors from random initial states
%     for j = 1:sample_succ_number
%         plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%     end
% 
%     % Calculate minimum bounding box for black dots
%     x_min_actual = min(rand_succ(idx1, :));
%     x_max_actual = max(rand_succ(idx1, :));
%     y_min_actual = min(rand_succ(idx2, :));
%     y_max_actual = max(rand_succ(idx2, :));
%     area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));
% 
%     % Plot over-approximation rectangles
%     handles = [];
%     proxy_handles = [];
% 
%     for m = 1:n_methods
%         % Proxy handle for initial approach (Dark Blue, Red)
%         h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_init];
% 
%         % Initial approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_all_methods(idx1, k, m);
%             y_min = succ_low_all_methods(idx2, k, m);
%             width = succ_up_all_methods(idx1, k, m) - x_min;
%             height = succ_up_all_methods(idx2, k, m) - y_min;
%             fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_overapprox = width * height;
%                 tightness_metrics(m, 1) = area_overapprox / area_actual;
%             else
%                 warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Proxy handle for incremental approach (Cyan, Yellow)
%         h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_inc];
% 
%         % Incremental approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_inc_methods(idx1, k, m);
%             y_min = succ_low_inc_methods(idx2, k, m);
%             width = succ_up_inc_methods(idx1, k, m) - x_min;
%             height = succ_up_inc_methods(idx2, k, m) - y_min;
%             fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_inc = width * height;
%                 tightness_metrics_inc(m, 1) = area_inc / area_actual;
%             else
%                 warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Boundary approach
%         if plot_last_only
%             k = n_t;
%             x_min_bound = succ_low_boundary{1}(idx1, k, m);
%             y_min_bound = succ_low_boundary{1}(idx2, k, m);
%             x_max_bound = succ_up_boundary{1}(idx1, k, m);
%             y_max_bound = succ_up_boundary{1}(idx2, k, m);
%             for j = 2:num_splits
%                 x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
%                 y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
%                 x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
%                 y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
%             end
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 if m == 1
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
%                 else
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
%                 end
%                 handles = [handles, h_bound];
%                 proxy_handles = [proxy_handles, h_proxy_bound];
%                 area_bound = width_bound * height_bound;
%                 tightness_metrics_bound(m, 1) = area_bound / area_actual;
%             else
%                 warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             end
%         end
%     end
% 
%     % Add labels and legend for Figure 1
%     xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%     all_proxy_handles = [proxy_handles(5), proxy_handles(6), proxy_handles(1), proxy_handles(2), proxy_handles(3), proxy_handles(4)];
%     all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
%                         'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
%                         'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
%     lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
%     set(lgd, 'FontSize', 9, 'FontWeight', 'bold');
% 
%     % Figure 2: x3 vs x4
%     figure
%     hold on
%     grid on
%     idx1 = plot_pairs(2, 1);
%     idx2 = plot_pairs(2, 2);
% 
%     % Plot the successors from random initial states
%     for j = 1:sample_succ_number
%         plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%     end
% 
%     % Calculate minimum bounding box for black dots
%     x_min_actual = min(rand_succ(idx1, :));
%     x_max_actual = max(rand_succ(idx1, :));
%     y_min_actual = min(rand_succ(idx2, :));
%     y_max_actual = max(rand_succ(idx2, :));
%     area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));
% 
%     % Plot over-approximation rectangles
%     handles = [];
%     proxy_handles = [];
% 
%     for m = 1:n_methods
%         % Proxy handle for initial approach (Dark Blue, Red)
%         h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_init];
% 
%         % Initial approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_all_methods(idx1, k, m);
%             y_min = succ_low_all_methods(idx2, k, m);
%             width = succ_up_all_methods(idx1, k, m) - x_min;
%             height = succ_up_all_methods(idx2, k, m) - y_min;
%             fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_overapprox = width * height;
%                 tightness_metrics(m, 2) = area_overapprox / area_actual;
%             else
%                 warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Proxy handle for incremental approach (Cyan, Yellow)
%         h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_inc];
% 
%         % Incremental approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_inc_methods(idx1, k, m);
%             y_min = succ_low_inc_methods(idx2, k, m);
%             width = succ_up_inc_methods(idx1, k, m) - x_min;
%             height = succ_up_inc_methods(idx2, k, m) - y_min;
%             fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_inc = width * height;
%                 tightness_metrics_inc(m, 2) = area_inc / area_actual;
%             else
%                 warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Boundary approach
%         if plot_last_only
%             k = n_t;
%             x_min_bound = succ_low_boundary{1}(idx1, k, m);
%             y_min_bound = succ_low_boundary{1}(idx2, k, m);
%             x_max_bound = succ_up_boundary{1}(idx1, k, m);
%             y_max_bound = succ_up_boundary{1}(idx2, k, m);
%             for j = 2:num_splits
%                 x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
%                 y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
%                 x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
%                 y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
%             end
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 if m == 1
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
%                 else
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
%                 end
%                 handles = [handles, h_bound];
%                 proxy_handles = [proxy_handles, h_proxy_bound];
%                 area_bound = width_bound * height_bound;
%                 tightness_metrics_bound(m, 2) = area_bound / area_actual;
%             else
%                 warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             end
%         end
%     end
% 
%     % Add labels and legend for Figure 2
%     xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%     all_proxy_handles = [proxy_handles(5), proxy_handles(6), proxy_handles(1), proxy_handles(2), proxy_handles(3), proxy_handles(4)];
%     all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
%                         'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
%                         'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
%     lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
%     set(lgd, 'FontSize', 9, 'FontWeight', 'bold');
% 
%     % Figure 3: x4 vs x5
%     figure
%     hold on
%     grid on
%     idx1 = plot_pairs(3, 1);
%     idx2 = plot_pairs(3, 2);
% 
%     % Plot the successors from random initial states
%     for j = 1:sample_succ_number
%         plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%     end
% 
%     % Calculate minimum bounding box for black dots
%     x_min_actual = min(rand_succ(idx1, :));
%     x_max_actual = max(rand_succ(idx1, :));
%     y_min_actual = min(rand_succ(idx2, :));
%     y_max_actual = max(rand_succ(idx2, :));
%     area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));
% 
%     % Plot over-approximation rectangles
%     handles = [];
%     proxy_handles = [];
% 
%     for m = 1:n_methods
%         % Proxy handle for initial approach (Dark Blue, Red)
%         h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_init];
% 
%         % Initial approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_all_methods(idx1, k, m);
%             y_min = succ_low_all_methods(idx2, k, m);
%             width = succ_up_all_methods(idx1, k, m) - x_min;
%             height = succ_up_all_methods(idx2, k, m) - y_min;
%             fprintf('Initial full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_overapprox = width * height;
%                 tightness_metrics(m, 3) = area_overapprox / area_actual;
%             else
%                 warning('Skipping initial full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Proxy handle for incremental approach (Cyan, Yellow)
%         h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_inc];
% 
%         % Incremental approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_inc_methods(idx1, k, m);
%             y_min = succ_low_inc_methods(idx2, k, m);
%             width = succ_up_inc_methods(idx1, k, m) - x_min;
%             height = succ_up_inc_methods(idx2, k, m) - y_min;
%             fprintf('Incremental full reachable set, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%                 area_inc = width * height;
%                 tightness_metrics_inc(m, 3) = area_inc / area_actual;
%             else
%                 warning('Skipping incremental full reachable set, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min, y_min, width, height);
%             end
%         end
% 
%         % Boundary approach
%         if plot_last_only
%             k = n_t;
%             x_min_bound = succ_low_boundary{1}(idx1, k, m);
%             y_min_bound = succ_low_boundary{1}(idx2, k, m);
%             x_max_bound = succ_up_boundary{1}(idx1, k, m);
%             y_max_bound = succ_up_boundary{1}(idx2, k, m);
%             for j = 2:num_splits
%                 x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
%                 y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
%                 x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
%                 y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
%             end
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Aggregated boundary split, method %d, pair [%d %d]: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 if m == 1
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
%                 else
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
%                 end
%                 handles = [handles, h_bound];
%                 proxy_handles = [proxy_handles, h_proxy_bound];
%                 area_bound = width_bound * height_bound;
%                 tightness_metrics_bound(m, 3) = area_bound / area_actual;
%             else
%                 warning('Skipping aggregated boundary split, method %d, pair [%d %d] due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, idx1, idx2, x_min_bound, y_min_bound, width_bound, height_bound);
%             end
%         end
%     end
% 
%     % Add labels and legend for Figure 3
%     xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 14);
%     title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
%     all_proxy_handles = [proxy_handles(5), proxy_handles(6), proxy_handles(1), proxy_handles(2), proxy_handles(3), proxy_handles(4)];
%     all_legend_labels = {'CT Mixed-Monotonicity (Boundary)', 'CT Sampled-Data Mixed-Monotonicity (Boundary)', ...
%                         'CT Mixed-Monotonicity (Initial)', 'CT Sampled-Data Mixed-Monotonicity (Initial)', ...
%                         'CT Mixed-Monotonicity (Incremental)', 'CT Sampled-Data Mixed-Monotonicity (Incremental)'};
%     lgd = legend(all_proxy_handles, all_legend_labels, 'Location', 'best');
%     set(lgd, 'FontSize', 9, 'FontWeight', 'bold');
% 
%     % Save the figures
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     save_dir = fullfile(pwd, 'figures');
%     fprintf('Attempting to save figures to: %s\n', save_dir);
%     if ~exist(save_dir, 'dir')
%         [status, msg] = mkdir(save_dir);
%         if status ~= 1
%             warning('Failed to create directory %s: %s', save_dir, msg);
%         else
%             fprintf('Created directory: %s\n', save_dir);
%         end
%     end
%     if exist(save_dir, 'dir')
%         try
%             saveas(figure(1), fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.eps', timestamp)), 'epsc');
%             fprintf('Figure 1 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.eps', timestamp)));
%             saveas(figure(2), fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.eps', timestamp)), 'epsc');
%             fprintf('Figure 2 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.eps', timestamp)));
%             saveas(figure(3), fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.eps', timestamp)), 'epsc');
%             fprintf('Figure 3 saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.eps', timestamp)));
%         catch e
%             warning('Failed to save figures as EPS: %s. Trying PNG format...', e.message);
%             saveas(figure(1), fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.png', timestamp)), 'png');
%             fprintf('Figure 1 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x1_x2_%s.png', timestamp)));
%             saveas(figure(2), fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.png', timestamp)), 'png');
%             fprintf('Figure 2 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x3_x4_%s.png', timestamp)));
%             saveas(figure(3), fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.png', timestamp)), 'png');
%             fprintf('Figure 3 saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_FPA_x4_x5_%s.png', timestamp)));
%         end
%     end
% 
%     % Display tightness metrics
%     fprintf('Tightness Metric (Area Ratio) for FPA 5D plane projections:\n');
%     for p = 1:size(plot_pairs, 1)
%         idx1 = plot_pairs(p, 1);
%         idx2 = plot_pairs(p, 2);
%         fprintf('Pair [%d %d]:\n', idx1, idx2);
%         for m = 1:n_methods
%             fprintf('Method %s (Initial): %.4f\n', method_names{m}, tightness_metrics(m, p));
%             fprintf('Method %s (Incremental): %.4f\n', method_names{m}, tightness_metrics_inc(m, p));
%             fprintf('Method %s (Boundary): %.4f\n', method_names{m}, tightness_metrics_bound(m, p));
%         end
%     end
% 
% catch e
%     fprintf('Error in plotting: %s\n', e.message);
%     rethrow(e);
% end


%% Plotting used for the Spiral 2D
% try
%     plot_pairs = [1 2]; % For 2D Spiral_non system
%     colors = [0 0 1; 1 0 0; 0 1 1; 1 1 0]; % Dark Blue, Red, Cyan (Light Blue), Yellow
%     method_names = {'CT Mixed-Monotonicity', 'CT Sampled-Data Mixed-Monotonicity'};
% 
%     figure
%     hold on
%     grid on
%     idx1 = plot_pairs(1, 1);
%     idx2 = plot_pairs(1, 2);
% 
%     % Plot the successors from random initial states
%     for j = 1:sample_succ_number
%         plot(rand_succ(idx1, j), rand_succ(idx2, j), 'k.', 'MarkerSize', 5);
%     end
% 
%     % Calculate minimum bounding box for black dots
%     x_min_actual = min(rand_succ(idx1, :));
%     x_max_actual = max(rand_succ(idx1, :));
%     y_min_actual = min(rand_succ(idx2, :));
%     y_max_actual = max(rand_succ(idx2, :));
%     area_actual = abs((x_max_actual - x_min_actual) * (y_max_actual - y_min_actual));
% 
%     % Plot over-approximation rectangles
%     handles = [];
%     proxy_handles = [];
% 
%     for m = 1:n_methods
%         % Proxy handle for initial approach (Dark Blue, Red)
%         h_proxy_init = plot(nan, nan, 'Color', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_init];
% 
%         % Initial approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_all_methods(idx1, k, m);
%             y_min = succ_low_all_methods(idx2, k, m);
%             width = succ_up_all_methods(idx1, k, m) - x_min;
%             height = succ_up_all_methods(idx2, k, m) - y_min;
%             fprintf('Initial full reachable set, method %d: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%             else
%                 warning('Skipping initial full reachable set, method %d due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min, y_min, width, height);
%             end
%         end
% 
%         % Proxy handle for incremental approach (Cyan, Yellow)
%         h_proxy_inc = plot(nan, nan, 'Color', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-', 'Visible', 'off');
%         proxy_handles = [proxy_handles, h_proxy_inc];
% 
%         % Incremental approach
%         if plot_last_only
%             k = n_t;
%             x_min = succ_low_inc_methods(idx1, k, m);
%             y_min = succ_low_inc_methods(idx2, k, m);
%             width = succ_up_inc_methods(idx1, k, m) - x_min;
%             height = succ_up_inc_methods(idx2, k, m) - y_min;
%             fprintf('Incremental full reachable set, method %d: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, x_min, y_min, width, height);
%             if all(isfinite([x_min, y_min, width, height])) && width >= 0 && height >= 0
%                 h = rectangle('Position', [x_min, y_min, width, height], ...
%                               'EdgeColor', colors(m+2, :), 'LineWidth', 1.5, 'LineStyle', '-');
%                 handles = [handles, h];
%             else
%                 warning('Skipping incremental full reachable set, method %d due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min, y_min, width, height);
%             end
%         end
% 
%         % Boundary approach
%         if plot_last_only
%             k = n_t;
%             x_min_bound = succ_low_boundary{1}(idx1, k, m);
%             y_min_bound = succ_low_boundary{1}(idx2, k, m);
%             x_max_bound = succ_up_boundary{1}(idx1, k, m);
%             y_max_bound = succ_up_boundary{1}(idx2, k, m);
%             for j = 2:num_splits
%                 x_min_bound = min(x_min_bound, succ_low_boundary{j}(idx1, k, m));
%                 y_min_bound = min(y_min_bound, succ_low_boundary{j}(idx2, k, m));
%                 x_max_bound = max(x_max_bound, succ_up_boundary{j}(idx1, k, m));
%                 y_max_bound = max(y_max_bound, succ_up_boundary{j}(idx2, k, m));
%             end
%             width_bound = x_max_bound - x_min_bound;
%             height_bound = y_max_bound - y_min_bound;
%             fprintf('Aggregated boundary split, method %d: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, x_min_bound, y_min_bound, width_bound, height_bound);
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 if m == 1
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'm', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'm--', 'LineWidth', 1.5, 'Visible', 'off');
%                 else
%                     h_bound = rectangle('Position', [x_min_bound, y_min_bound, width_bound, height_bound], ...
%                                       'EdgeColor', 'g', 'LineWidth', 1.5, 'LineStyle', '--');
%                     h_proxy_bound = plot(nan, nan, 'g--', 'LineWidth', 1.5, 'Visible', 'off');
%                 end
%                 handles = [handles, h_bound];
%                 proxy_handles = [proxy_handles, h_proxy_bound];
%             else
%                 warning('Skipping aggregated boundary split, method %d due to invalid parameters: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min_bound, y_min_bound, width_bound, height_bound);
%             end
%         end
% 
%         % Calculate tightness metrics
%         if plot_last_only
%             k = n_t;
%             % Initial approach
%             x_min_final = succ_low_all_methods(idx1, k, m);
%             y_min_final = succ_low_all_methods(idx2, k, m);
%             width_final = succ_up_all_methods(idx1, k, m) - x_min_final;
%             height_final = succ_up_all_methods(idx2, k, m) - y_min_final;
%             fprintf('Tightness metric (Initial), method %d: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, x_min_final, y_min_final, width_final, height_final);
%             if all(isfinite([x_min_final, y_min_final, width_final, height_final])) && width_final >= 0 && height_final >= 0
%                 area_overapprox = width_final * height_final;
%                 tightness_metrics(m) = area_overapprox / area_actual;
%             else
%                 warning('Invalid area calculation for initial approach, method %d: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min_final, y_min_final, width_final, height_final);
%                 tightness_metrics(m) = NaN;
%             end
% 
%             % Incremental approach
%             x_min_inc = succ_low_inc_methods(idx1, k, m);
%             y_min_inc = succ_low_inc_methods(idx2, k, m);
%             width_inc = succ_up_inc_methods(idx1, k, m) - x_min_inc;
%             height_inc = succ_up_inc_methods(idx2, k, m) - y_min_inc;
%             fprintf('Tightness metric (Incremental), method %d: x_min=%f, y_min=%f, width=%f, height=%f\n', ...
%                     m, x_min_inc, y_min_inc, width_inc, height_inc);
%             if all(isfinite([x_min_inc, y_min_inc, width_inc, height_inc])) && width_inc >= 0 && height_inc >= 0
%                 area_inc = width_inc * height_inc;
%                 tightness_metrics_inc(m) = area_inc / area_actual;
%             else
%                 warning('Invalid area calculation for incremental approach, method %d: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min_inc, y_min_inc, width_inc, height_inc);
%                 tightness_metrics_inc(m) = NaN;
%             end
% 
%             % Boundary approach
%             area_bound = width_bound * height_bound;
%             if all(isfinite([x_min_bound, y_min_bound, width_bound, height_bound])) && width_bound >= 0 && height_bound >= 0
%                 tightness_metrics_bound(m) = area_bound / area_actual;
%             else
%                 warning('Invalid area calculation for boundary split, method %d: x_min=%f, y_min=%f, width=%f, height=%f', ...
%                         m, x_min_bound, y_min_bound, width_bound, height_bound);
%                 tightness_metrics_bound(m) = NaN;
%             end
%         end
%     end
% 
%     % Add labels
%     xlabel(['$x_', num2str(idx1), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%     ylabel(['$x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 20);
%     title(['$x_', num2str(idx1), '$ vs. $x_', num2str(idx2), '$'], 'Interpreter', 'Latex', 'FontSize', 16);
% 
%     % Add single legend with correct method assignments in the specified order
% 
%     % Dashed magneta box equivalent to Mixed-Monotonicity (Boundary) 
%     % Dashed green box equivalent to sampled-Data Mixed-Monotonicity (Boundary)  
%     % Dark Blue solid box equivalent to Mixed-Monotonicity (single-step) 
%     % Red solid box equivalent to Sampled-Data Mixed-Monotonicity (single-step) 
%     % Light blue solid box equivalent to Mixed-Monotonicity (incremental) 
%     % Yellow solid box equivalent to Sampled-Data Mixed-Monotonicity (incremental)
% 
%     % all_proxy_handles = [proxy_handles(5), proxy_handles(6), proxy_handles(1), proxy_handles(2), proxy_handles(3), proxy_handles(4)];
%     all_proxy_handles = [proxy_handles(1), proxy_handles(4), proxy_handles(2), proxy_handles(5), proxy_handles(3), proxy_handles(6)];
%     all_legend_labels = {'Mixed-Monotonicity (Single-step)', 'Sampled-Data Mixed-Monotonicity (Single-step)', ...
%                          'Mixed-Monotonicity (Incremental)', 'Sampled-Data Mixed-Monotonicity (Incremental)', ...
%                          'Mixed-Monotonicity (Boundary)',    'Sampled-Data Mixed-Monotonicity (Boundary)'};
% 
%                          % 'Sampled-Data Mixed-Monotonicity (Incremental)', 'Mixed-Monotonicity (Boundary)', ...
%                          % 'Mixed-Monotonicity (Single-step)', 'Mixed-Monotonicity (Incremental)', ...
%                          % 'Sampled-Data Mixed-Monotonicity (Boundary)', 'Sampled-Data Mixed-Monotonicity (Single-step)'};
%     % all_legend_labels = {'Mixed-Monotonicity (Boundary)', 'Sampled-Data Mixed-Monotonicity (Boundary)', ...
%     %                     'Mixed-Monotonicity (Single-step)', 'Sampled-Data Mixed-Monotonicity (Single-step)', ...
%     %                     'Mixed-Monotonicity (Incremental)', 'Sampled-Data Mixed-Monotonicity (Incremental)'};
%     lgd= legend(all_proxy_handles, all_legend_labels, 'Location', 'southeast');
%     set(lgd, 'FontSize', 9, 'FontWeight', 'bold');
% 
%     % Save the figure
%     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%     save_dir = fullfile(pwd, 'figures');
%     fprintf('Attempting to save figure to: %s\n', fullfile(save_dir, sprintf('reachability_plot_all_%s.eps', timestamp)));
%     if ~exist(save_dir, 'dir')
%         [status, msg] = mkdir(save_dir);
%         if status ~= 1
%             warning('Failed to create directory %s: %s', save_dir, msg);
%         else
%             fprintf('Created directory: %s\n', save_dir);
%         end
%     end
%     if exist(save_dir, 'dir')
%         try
%             saveas(gcf, fullfile(save_dir, sprintf('reachability_plot_all_%s.eps', timestamp)), 'epsc');
%             fprintf('Figure saved successfully as EPS: %s\n', fullfile(save_dir, sprintf('reachability_plot_all_%s.eps', timestamp)));
%         catch e
%             warning('Failed to save figure as EPS: %s. Trying PNG format...', e.message);
%             saveas(gcf, fullfile(save_dir, sprintf('reachability_plot_all_%s.png', timestamp)), 'png');
%             fprintf('Figure saved as PNG: %s\n', fullfile(save_dir, sprintf('reachability_plot_all_%s.png', timestamp)));
%         end
%     else
%         warning('Directory %s does not exist and could not be created. Skipping figure save.', save_dir);
%     end
% catch e
%     fprintf('Error in plotting: %s\n', e.message);
%     rethrow(e);
% end
% 
% % Display tightness metrics and computation times
% fprintf('Tightness Metric (Area Ratio) for %s vs. %s plane:\n', ...
%         ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
% for m = 1:n_methods
%     fprintf('Method %s (Initial): %.2f\n', method_names{m}, tightness_metrics(m));
%     fprintf('Method %s (Incremental): %.2f\n', method_names{m}, tightness_metrics_inc(m));
%     fprintf('Method %s (Boundary): %.2f\n', method_names{m}, tightness_metrics_bound(m));
% end
% fprintf('Computation Times (seconds) for %s vs. %s plane:\n', ...
%         ['x_' num2str(idx1)], ['x_' num2str(idx2)]);
% for m = 1:n_methods
%     fprintf('Method %s (Initial): %.6f\n', method_names{m}, time_initial(m));
%     fprintf('Method %s (Incremental): %.6f\n', method_names{m}, time_incremental(m));
%     fprintf('Method %s (Boundary): %.6f\n', method_names{m}, mean(time_boundary(:, m)));
% end
% 
% %% Save the reachable sets and random successors
% try
%     filename_base = sprintf('reachability_data_sys_TIRA%d_all_%s', system_choice, timestamp);
%     save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'succ_low_inc_methods', 'succ_up_inc_methods', ...
%          'succ_low_boundary', 'succ_up_boundary', 'rand_succ', 't_points', 'methods', 'n_x', 'n_methods', ...
%          'time_initial', 'time_incremental', 'time_boundary');
%     disp(['Data saved to ' filename_base '.mat']);
% catch e
%     fprintf('Error in saving data: %s\n', e.message);
%     rethrow(e);
% end