% =========================================================================
% File: get_solver_parameters.m
%
% Purpose:
%   Function-style version of Solver_parameters.m that returns the TIRA
%   configuration struct.
%
% Description:
%   Replicates the full behaviour of the original Solver_parameters.m
%   script, but as a function so it can be called via run() without
%   triggering "static workspace" errors. Use this entry point in any
%   context where Solver_parameters.m's script execution would interfere
%   with the caller's workspace (e.g. parfor workers).
%
% Source:
%   P.-J. Meyer, A. Devonport and M. Arcak, "Interval Reachability
%   Analysis: Bounding Trajectories of Uncertain Systems with Boxes
%   for Control and Verification". Springer, 2021.
%   DOI: 10.1007/978-3-030-65110-7
%
% Inputs:
%   n_x                         : state dimension
%   n_p                         : input / parameter dimension
%   g_sensitivity_bounds_method : global sensitivity-bounds method (from
%                                 the global of the same name)
%
% Output:
%   parameters : fully configured TIRA parameters struct
%
% Part of the TNODEV Toolbox
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function parameters = get_solver_parameters(n_x, n_p, g_sensitivity_bounds_method)

    addpath('../Utilities');


    %% Logger configuration
    % Logging levels:
    %   ALL = 0, RUNTIME = 1, INFO = 2, WARNING = 3, ERROR = 4, OFF = 5
    parameters.log_level    = 5;     % OFF
    parameters.log_location = NaN;   % NaN -> command window; else a file name (e.g. "tira.log")

    % The 3 commands below do not need to be modified by the user
    [log, delete_logger] = logger.get_logger(); %#ok<ASGLU>
    log.set_log_level(parameters.log_level);
    log.set_log_location(parameters.log_location);


    %% ODE configuration
    % Select or define ODE solver. Must have the same function signature
    % as built-in ODE solvers invoked as
    %   parameters.ode_solver(odefun, tspan, y0, parameters.ode_options)
    parameters.ode_solver  = @ode45;
    % parameters.ode_solver = @ode15s;

    % Options structure that will be passed to calls of ode_solver
    parameters.ode_options = {};     % default
    % parameters.ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', 1);
    % parameters.ode_options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'MaxStep', 0.1);


    %% Choice of the over-approximation method
    %   1   - Continuous-time monotonicity
    %   2   - Continuous-time contraction and growth bound
    %   3   - Continuous-time mixed-monotonicity
    %   4   - Continuous-time sampled-data mixed-monotonicity
    %   5   - Discrete-time mixed-monotonicity
    %   6   - Quasi-Monte Carlo
    %   7   - Monte Carlo
    %   NaN - let TIRA.m auto-select the best available method
    parameters.OA_method = NaN;


        %% (2): Contraction / growth-bound submethod choice
        %   2-1 : User file
        %   2-2 : Contraction matrix / scalar
        %   2-3 : Jacobian bounds
        %   NaN : Auto-select
        parameters.growth_bound_method = NaN;


        %% (4): Sampled-data mixed-monotonicity submethod choice
        %   4-1 : User-provided sensitivity bounds
        %   4-2 : Interval Arithmetics
        %   4-3 : 2nd order Sensitivity
        %   4-0 : Sampling and falsification
        %   NaN : Auto-select (default: 4-0 if nothing provided)
        parameters.sensitivity_bounds_method = g_sensitivity_bounds_method;


            %% (4-2) and (4-3): Parameters for Taylor series of the interval matrix exponential
            parameters.taylor_order      = 50;
            parameters.taylor_tolerance  = 1e-3;
            parameters.taylor_increment  = 10;
            parameters.scaling_factor    = NaN;


            %% (4-3): Sampling parameters for the 2nd Order Sensitivity method
            parameters.sensitivity_samples_per_state_dimension = 2 * ones(n_x, 1);
            parameters.sensitivity_samples_per_input_dimension = 2 * ones(n_p, 1);


            %% (4-3) and (4-0): Solvers of the sensitivity equations
            %   0   : Euler on sensitivity definition
            %   1   : Euler on Jacobian + ODE on sensitivity
            %   2   : ODE solver with explicit Jacobian
            %   NaN : Auto-select (default: 0)
            parameters.sensitivity_equation_solver = NaN;

            % Euler approximation parameters
            parameters.euler_epsilon   = 1e-4;
            parameters.euler_tolerance = 1e-10;
            parameters.euler_max_iter  = 4;


            %% (4-0): Sampling and falsification parameters
            parameters.sensitivity_sampling_pairs         = 50000;   % was NaN originally
            parameters.sensitivity_sampling_per_dimension = 2;
            parameters.falsification_max_iter             = 0;       % was 2


        %% (6): Quasi-Monte Carlo, sampling parameters and submethod choice
        parameters.QMC_samples_per_state_dimension = 10 * ones(n_x, 1);
        parameters.QMC_samples_per_input_dimension =  2 * ones(n_p, 1);

        % How is the error-bound function obtained?
        %   6-1 : User file
        %   6-2 : Contraction matrix
        %   6-3 : Jacobian bounds
        %   6-4 : Lipschitz constant
        %   NaN : Auto-select
        parameters.error_bound_method = NaN;


        %% (7): Monte Carlo
        parameters.epsilon = 0.005;
        parameters.delta   = 1e-9;


    % --- Alternative defaults (for reference) -------------------------------
    % These were at the end of the original script and are kept for reference:
    %
    %   parameters.log_level                            = logger.logger.ALL;
    %   parameters.log_location                         = NaN;
    %   parameters.ode_solver                           = @ode45;
    %   parameters.ode_options                          = {};
    %   parameters.OA_method                            = NaN;
    %   parameters.growth_bound_method                  = NaN;
    %   parameters.sensitivity_bounds_method            = NaN;
    %   parameters.taylor_order                         = 100;
    %   parameters.taylor_tolerance                     = 1e-3;
    %   parameters.taylor_increment                     = 100;
    %   parameters.scaling_factor                       = NaN;
    %   parameters.sensitivity_samples_per_state_dimension = 2 * ones(n_x, 1);
    %   parameters.sensitivity_samples_per_input_dimension = 2 * ones(n_p, 1);
    %   parameters.sensitivity_equation_solver          = NaN;
    %   parameters.euler_epsilon                        = 1e-4;
    %   parameters.euler_tolerance                      = 1e-10;
    %   parameters.euler_max_iter                       = 4;
    %   parameters.sensitivity_sampling_pairs           = NaN;
    %   parameters.sensitivity_sampling_per_dimension   = 2;
    %   parameters.falsification_max_iter               = Inf;
    %   parameters.QMC_samples_per_state_dimension      = 2 * ones(n_x, 1);
    %   parameters.QMC_samples_per_input_dimension      = 2 * ones(n_p, 1);
    %   parameters.error_bound_method                   = NaN;
    %   parameters.epsilon                              = 0.05;
    %   parameters.delta                                = 1e-9;

end
%------------- END OF CODE --------------
