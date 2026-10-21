% =========================================================================
% File: NODE_RA_New.m
%
% Purpose:
%   Reachability analysis driver for Neural ODE systems using the TIRA
%   framework. Returns interval over-approximations (OAs) of the
%   reachable set at the final time, as a pure computational function.
%
% Description:
%   For each system_choice the routine selects an appropriate set of
%   TIRA OA methods and (optionally) augments them with boundary-based
%   sub-analyses. The intersection of all OAs is then formed as a tight
%   final bound and returned together with the per-method bounds for
%   diagnostics / plotting.
%
%   System-specific configuration (methods + boundary analysis):
%     14 (FPA)            : methods = [3, 4],   boundary = on
%     15 (Spiral_non)     : methods = [3, 4],   boundary = on
%     16 (MNIST)          : methods = [3],      boundary = off (676 dims)
%     17 (ACC nonlinear)  : methods = [3],      boundary = off
%     18 (Single Pendulum): methods = [3],      boundary = off
%     21 (Spiral_linear)  : methods = [3],      boundary = off
%     22 (Cartpole 12D)   : methods = [3],      boundary = off (2^12 vertices)
%     24 (ACC linear)     : methods = [3],      boundary = off
%
%   TIRA method codes used here:
%     3  = Continuous-Time Mixed-Monotonicity (CT-MM)
%     4  = Sampled-Data Mixed-Monotonicity    (SD-MM)
%
% Input:
%   params - struct with fields
%       .system_choice  : integer (see list above)
%       .x0             : initial-set centre
%       .Initial_radius : initial-set radius (vector or scalar)
%       .t_final        : final time for the analysis
%       .timeStep       : discretization step (kept for consistency;
%                         not used directly by TIRA)
%       .A, .B          : (system 16 only) MNIST linear-ODE matrices
%
% Output:
%   reachability_results - struct with fields
%       .succ_low_all_methods, .succ_up_all_methods
%       .succ_low_boundary,    .succ_up_boundary
%       .num_splits
%       .intersect_low,        .intersect_up
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
% =========================================================================

%------------- BEGIN CODE --------------
function reachability_results = NODE_RA_New(params)

    %% 1. Initialization (path setup + globals)
    if exist('../OA_methods', 'dir')
        addpath('../OA_methods');
    end
    if exist('../Utilities', 'dir')
        addpath('../Utilities');
    end

    % Initialize the global sensitivity-method choice to a scalar NaN.
    % Ensures TIRA's sub-functions always find a valid scalar value when
    % they check this global, preventing the '||' operator error.
    global g_sensitivity_bounds_method
    g_sensitivity_bounds_method = nan;


    %% 2. Definition of the system and reachability problem
    global u system_choice %#ok<NUSED>
    system_choice = get_from_params(params, 'system_choice');

    % Unified parameter loading (all systems)
    Initial_radius = get_from_params(params, 'Initial_radius');
    x0             = get_from_params(params, 'x0');
    t_final        = get_from_params(params, 't_final');
    steps          = get_from_params(params, 'timeStep'); %#ok<NASGU>

    x_low  = x0 - Initial_radius;
    x_up   = x0 + Initial_radius;
    t_init = 0;

    p_low = zeros(length(x_low), 1);
    p_up  = zeros(length(x_low), 1);

    n_x = length(x_low);


    %% 3. Default OA configuration (single-step methods + boundary on)
    run_boundary_analysis = true;
    methods = [3, 4];     % TIRA codes: 3 = CT-MM, 4 = SD-MM


    %% 4. System-specific overrides
    if system_choice == 16
        % --- MNIST ---
        % 1. Load A / B globals used by the linear MNIST dynamics
        global Aout_global Bout_global
        Aout_global = get_from_params(params, 'A');
        Bout_global = get_from_params(params, 'B');

        % 2. Restrict to CT-MM only
        methods = [3];

        % 3. Disable boundary analysis (too expensive for 676 dims)
        run_boundary_analysis = false;

    elseif system_choice == 17
        % --- Nonlinear ACC (8D) ---
        % Methods 3 and 4 are the default. Boundary analysis is kept off
        % for speed; flip both lines to enable tighter bounds at higher cost.
        % methods = [3, 4];
        methods = [3];
        run_boundary_analysis = false;

    elseif system_choice == 24
        % --- Linear ACC (8D) ---
        methods = [3];
        run_boundary_analysis = false;

    elseif system_choice == 18
        % --- Single Pendulum (3D) ---
        % Restrict to CT-MM only and disable boundary analysis for faster
        % iterative refinement. Use methods = [3, 4] and true for tighter bounds.
        methods = [3];
        run_boundary_analysis = false;

    elseif system_choice == 21
        % --- Spiral_linear (2D) ---
        methods = [3];
        run_boundary_analysis = false;

    elseif system_choice == 22
        % --- Cartpole (12D) ---
        % Boundary analysis would be prohibitively expensive in 12D
        % (2^12 = 4096 vertices).
        methods = [3];
        run_boundary_analysis = false;
    end

    n_methods_single_step = length(methods);

    succ_low_all_methods = NaN(n_x, 1, n_methods_single_step);
    succ_up_all_methods  = NaN(n_x, 1, n_methods_single_step);


    %% 5. Boundary decompositions (only if enabled)
    num_splits             = 0;
    Initial_set_boundaries = {};
    succ_low_boundary      = {};
    succ_up_boundary       = {};

    if run_boundary_analysis
        num_splits             = 2 * n_x;
        Initial_set_boundaries = cell(num_splits, 1);
        succ_low_boundary      = cell(num_splits, 1);
        succ_up_boundary       = cell(num_splits, 1);

        % Define the 2*n_x boundary sets (one face fixed at x_low / x_up
        % per dimension), used as TIRA inputs.
        for i = 1:n_x
            % Face at x_low(i) -> replace lower bound by upper bound on dim i
            temp_low = x_low; temp_up = x_up; temp_low(i) = x_up(i);
            Initial_set_boundaries{2*i-1} = interval(temp_low(:), temp_up(:));
            % Face at x_up(i)  -> replace upper bound by lower bound on dim i
            temp_low = x_low; temp_up = x_up; temp_up(i)  = x_low(i);
            Initial_set_boundaries{2*i}   = interval(temp_low(:), temp_up(:));
        end
    end


    %% 6. Main reachability computation
    try
        % Single-step methods (always run)
        for m = 1:n_methods_single_step
            [succ_low, succ_up] = TIRA([t_init, t_final], x_low, x_up, p_low, p_up, methods(m));
            succ_low_all_methods(:, 1, m) = succ_low;
            succ_up_all_methods(:, 1, m)  = succ_up;
        end

        % Boundary methods (conditional)
        if run_boundary_analysis
            for i = 1:num_splits
                succ_low_boundary{i} = NaN(n_x, 1, n_methods_single_step);
                succ_up_boundary{i}  = NaN(n_x, 1, n_methods_single_step);
                for m = 1:n_methods_single_step
                    [succ_low, succ_up] = TIRA([t_init, t_final], ...
                        Initial_set_boundaries{i}.inf, ...
                        Initial_set_boundaries{i}.sup, ...
                        p_low, p_up, methods(m));
                    succ_low_boundary{i}(:, 1, m) = succ_low;
                    succ_up_boundary{i}(:, 1, m)  = succ_up;
                end
            end
        end
    catch e
        fprintf('Error during TIRA computation in NODE_RA_New.m\n');
        rethrow(e);
    end


    %% 7. Package per-method results
    % --> Some optional fields (timing breakdowns, system_choice, n_x) are
    %     kept commented in case they are needed for later diagnostics.
    reachability_results = struct();
    reachability_results.succ_low_all_methods = succ_low_all_methods;
    reachability_results.succ_up_all_methods  = succ_up_all_methods;
    reachability_results.succ_low_boundary    = succ_low_boundary;
    reachability_results.succ_up_boundary     = succ_up_boundary;
    reachability_results.num_splits           = num_splits;

    % --- Optional diagnostic fields (uncomment to expose) ---------------
    % reachability_results.time_initial   = time_initial;
    % reachability_results.time_boundary  = time_boundary;
    % reachability_results.system_choice  = system_choice;
    % reachability_results.n_x            = n_x;


    %% 8. Compute intersection of all over-approximations
    k          = 1;        % index for the final time step
    check_dims = n_x;

    intersect_low = -Inf(check_dims, 1);
    intersect_up  =  Inf(check_dims, 1);

    if system_choice == 16
        % MNIST: only one method (CT-MM), no boundaries -> intersection
        % is simply the result of that single method.
        intersect_low = succ_low_all_methods(:, 1, 1);
        intersect_up  = succ_up_all_methods(:, 1, 1);

    else
        % All other systems: intersect single-step bounds, then refine
        % with boundary bounds when boundary analysis is enabled.

        % Single-step methods
        for m = 1:n_methods_single_step
            intersect_low = max(intersect_low, succ_low_all_methods(1:check_dims, k, m));
            intersect_up  = min(intersect_up,  succ_up_all_methods(1:check_dims, k, m));
        end

        % Boundary methods
        if run_boundary_analysis
            for m = 1:n_methods_single_step
                Ls = Inf(check_dims, num_splits);
                Us = -Inf(check_dims, num_splits);
                for i = 1:num_splits
                    Ls(:, i) = succ_low_boundary{i}(1:check_dims, k, m);
                    Us(:, i) = succ_up_boundary{i}(1:check_dims, k, m);
                end
                intersect_low = max(intersect_low, min(Ls, [], 2));
                intersect_up  = min(intersect_up,  max(Us, [], 2));
            end
        end
    end

    % Store the intersection in the output structure (the main "Package Results")
    reachability_results.intersect_low = intersect_low;
    reachability_results.intersect_up  = intersect_up;

end


% =========================================================================
% Helper Function
% =========================================================================
function val = get_from_params(params, field)
    % Safely extract a field from the params struct.
    if isfield(params, field) && ~isempty(params.(field))
        val = params.(field);
    else
        error('NODE_RA_New: Missing required parameter "%s".', field);
    end
end
%------------- END OF CODE --------------
