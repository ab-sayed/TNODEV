% =========================================================================
% File: Solver_parameters.m
%
% Purpose:
%   configuration script (script-form counterpart of
%   get_solver_parameters.m). Choice of over-approximation methods,
%   sub-methods and definition of their internal parameters.
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
if exist('../Utilities', 'dir')
    addpath('../Utilities');
end


%% Global parameters
% Defined as global variables to more easily call the methods several
% times with different parameter values
global g_sensitivity_bounds_method;


%% Logger configuration
% The logger outputs information at various criticality levels.
% Setting parameters.log_level to a particular value allows messages of
% equal or higher level to be printed to the console. For example,
% parameters.log_level = logger.logger.WARNING allows warning and error
% messages to be printed.
%   ALL = 0, RUNTIME = 1, INFO = 2, WARNING = 3, ERROR = 4, OFF = 5
% parameters.log_level = logger.logger.ALL;
parameters.log_level = 5;

% Log to command window or to file:
%   NaN       : to command window
parameters.log_location = NaN;

% *The 3 commands below do not need to be modified by the user*
[log, delete_logger] = logger.get_logger();
log.set_log_level(parameters.log_level);
log.set_log_location(parameters.log_location);


%% ODE configuration
% Allows the user to select (amongst built-in solvers) or define their
% own ODE solver to be used in the methods that need one.
%
% The chosen ODE solver must have the same function signature as
% built-in MATLAB ODE solvers, invoked as:
%   parameters.ode_solver(odefun, tspan, y0, parameters.ode_options)
% This format excludes the fully implicit solvers ode15i and decic.
%
% Inputs:
%   odefun  : function which takes time and state and returns derivatives
%             dydt = odefun(t, y)
%   tspan   : time interval of the integration, e.g. [t_init, t_final]
%   y0      : initial conditions
%   options : option structure
% Outputs:
%   t : evaluation points
%   y : solutions at the corresponding time points in t
parameters.ode_solver  = @ode45;
% parameters.ode_solver = @ode15s;

% Options structure that will be passed to calls of ode_solver. If using
% a MATLAB ODE solver, see
%   https://www.mathworks.com/help/matlab/math/summary-of-ode-options.html
% for available options and how to construct the options variable.
parameters.ode_options = {};     % default
% parameters.ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', 1);
% parameters.ode_options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'MaxStep', 0.1);


%% Choice of the over-approximation method

% 3 - Continuous-time mixed-monotonicity
%     Extension of method 1 to any system with bounded Jacobian matrices.
% Requirements:
%   - Continuous-time system
%   - One of:
%       Signs  of the Jacobian matrices in ./Input_files/UP_Jacobian_Signs.m
%       Bounds of the Jacobian matrices in ./Input_files/UP_Jacobian_Bounds.m

% NaN - (no specific method requested)
%     Lets TIRA.m automatically select the best available method based on
%     system properties and the additional system description files in
%     the './Input_files' folder.

parameters.OA_method = NaN;


    %% (2): Contraction / growth bound submethod choice
    % How is the growth-bound function obtained?
    %   2-1 : Read from user-provided file
    %         Requires Input_files/UP_Growth_Bound_Function.m
    %   2-2 : Create from a user-provided contraction matrix or scalar
    %         Requires Input_files/UP_Contraction_Matrix.m
    %   2-3 : Create from user-provided Jacobian bounds
    %         Requires Input_files/UP_Jacobian_Bounds.m
    %   NaN : Let Utilities/Growth_bound_choice.m select the best
    %         available method (in the above order) based on the provided
    %         files in './Input_files'
    parameters.growth_bound_method = NaN;


    %% (4): Sampled-data mixed-monotonicity: submethod choice
    % How are bounds on the sensitivity matrices obtained?
    %   4-1 : Read sensitivity bounds from user-provided files
    %         Requires Input_files/UP_Sensitivity_Bounds.m
    %         or      Input_files/UP_Sensitivity_Signs.m
    %   4-2 : Interval Arithmetics
    %         Requires Input_files/UP_Jacobian_Function.m
    %   4-3 : 2nd order Sensitivity
    %         Requires Input_files/UP_Jacobian_Bounds.m and
    %                  Input_files/UP_Jacobian_2ndOrder_Bounds.m
    %   4-0 : Sampling and falsification
    %         No additional requirement.
    %   NaN : Let Utilities/Sensitivity_Bounds.m select the best
    %         available method based on provided files in './Input_files'
    %         (default if no file is provided: 4-0)
    parameters.sensitivity_bounds_method = g_sensitivity_bounds_method;


        %% (4-2) and (4-3): Parameters for Taylor series of the interval matrix exponential

        % Initial order for the truncation of the Taylor series (may be
        % iteratively increased if results are not satisfactory)
        parameters.taylor_order = 50;

        % Stopping condition: max relative error allowed between two
        % iterations (set to NaN to do a single iteration)
        parameters.taylor_tolerance = 1e-3;

        % Taylor order increase at each iteration
        parameters.taylor_increment = 10;

        % Scaling factor used in the Scaling-and-Squaring method
        % (set to NaN to let the code pick it automatically)
        parameters.scaling_factor = NaN;


        %% (4-3): Sampling parameters for the 2nd Order Sensitivity method

        % Number of sample points for each state dimension
        % (must be a vector of the same length as the state variable)
        parameters.sensitivity_samples_per_state_dimension = 2 * ones(n_x, 1);

        % Number of sample points for each input dimension
        % (must be a vector of the same length as the input variable)
        parameters.sensitivity_samples_per_input_dimension = 2 * ones(n_p, 1);


        %% (4-3) and (4-0): Solvers of the sensitivity equations
        %   0   : Euler method on the sensitivity definition
        %   1   : Euler method on Jacobian + ODE solver on sensitivity ODE
        %   2   : ODE solver on sensitivity ODE using explicit Jacobian function
        %         Requires Input_files/UP_Jacobian_Function.m
        %   NaN : Let Utilities/Sensitivity_Bounds.m select the best
        %         available solver based on provided files in
        %         './Input_files' (default if no file is provided: 0)
        parameters.sensitivity_equation_solver = NaN;

        % Parameters for Euler approximations in Sensitivity_solver.m
        % (used when sensitivity_equation_solver is 0 or 1)
        parameters.euler_epsilon   = 1e-4;     % Initial step size
        parameters.euler_tolerance = 1e-10;    % Error tolerance between iterations (loop stop)
        parameters.euler_max_iter  = 4;        % Max iterations (each halves the step)


        %% (4-0): Sampling and falsification parameters

        % Sample pairs in [x_low, x_up] x [p_low, p_up] for which
        % sensitivities are evaluated in Utilities/Sensitivity_sampling.m
        %   NaN : let the sampling function pick a default value
        %         (sensitivity_sampling_per_dimension^(n_x + n_p))
        %   1   : skip and only do falsification
        parameters.sensitivity_sampling_pairs = 50000;     % originally NaN

        % Number of sample pairs per state and input dimension to be used
        % in Utilities/Sensitivity_sampling.m when
        % 'sensitivity_sampling_pairs = NaN'
        parameters.sensitivity_sampling_per_dimension = 2;

        % Max number of falsification iterations in
        % Utilities/Sensitivity_falsification.m
        %   Inf : infinite
        %   0   : skip falsification
        parameters.falsification_max_iter = 0;             % originally 2


    %% (6): Quasi-Monte Carlo, sampling parameters and submethod choice

    % Number of sample points for each state dimension
    % (must be a vector of the same length as the state variable)
    parameters.QMC_samples_per_state_dimension = 10 * ones(n_x, 1);

    % Number of sample points for each input dimension
    % (must be a vector of the same length as the input variable)
    parameters.QMC_samples_per_input_dimension =  2 * ones(n_p, 1);

    % How is the error-bound function obtained?
    %   6-1 : Read from user-provided file
    %         Requires Input_files/UP_Error_Bound_Function.m
    %   6-2 : Create from a user-provided contraction matrix or scalar
    %         Requires Input_files/UP_Contraction_Matrix.m
    %   6-3 : Create from user-provided Jacobian bounds
    %         Requires Input_files/UP_Jacobian_Bounds.m
    %   6-4 : Create from a user-provided Lipschitz constant
    %         Requires Input_files/UP_Lipschitz_Constant.m
    %   NaN : Let Utilities/Error_bound_choice.m select the best
    %         available method (in the above order) based on the provided
    %         files in './Input_files'
    parameters.error_bound_method = NaN;


    %% (7): Monte Carlo

    % Accuracy of the resulting interval approximation
    % (define epsilon in (0, 1) to obtain a (1 - epsilon)-accuracy)
    parameters.epsilon = 0.005;

    % Confidence in obtaining a (1 - epsilon)-accuracy
    % (define delta in (0, 1) to obtain a (1 - delta)-confidence)
    parameters.delta = 1e-9;


% --- Alternative defaults (for reference) -----------------------------------
% Original default values (kept for reference; uncomment to revert):
%
%   parameters.log_level                            = logger.logger.ALL;
%   parameters.log_location                         = NaN;
%   parameters.ode_solver                           = @ode45;
%   parameters.ode_options                          = {};
%
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
%   parameters.epsilon                              = 0.05;     % 95% accuracy
%   parameters.delta                                = 1e-9;     % (1 - 1e-9) confidence

%------------- END OF CODE --------------
