% =========================================================================
% File: TIRA.m
%
% Purpose:
%   Hub function serving as interface between the reachability problem
%   definition and the library of over-approximation methods available
%   in folder ./OA_methods.
%
% Part of the TNODEV Toolbox
% Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function [succ_low, succ_up] = TIRA(time_vector, x_low, x_up, p_low, p_up, OA_method, force_method)

    if nargin < 7
        force_method = false;
    end


    %% Initialization

    % State and input dimensions
    n_x = length(x_low);
    n_p = length(p_low); 

    % Extract method choices and parameters in the 'parameters' structure
    run('Solver_parameters.m');

    % --- Alternative: load via the function-style interface ---
    % global g_sensitivity_bounds_method
    % if isnan(g_sensitivity_bounds_method)
    %     g_sensitivity_bounds_method = NaN;
    % end
    % parameters = get_solver_parameters(n_x, n_p, g_sensitivity_bounds_method);

    % Use the over-approximation method choice from Solver_parameters.m
    % if it was not provided as input argument of this function
    if nargin < 6
        OA_method = parameters.OA_method;
    end

    % Create logger
    [log, delete_logger] = logger.get_logger();
    log.info('Start of TIRA call\n');


    %% Extract time information
    if length(time_vector) == 2
        % 2D vector: continuous-time system (time_vector = [t_init, t_final])
        t_init  = time_vector(1);
        t_final = time_vector(2);
        if t_final == t_init
            % Test if the time range is a single time instant
            log.info(['The initial and final times are equals: the reachable ' ...
                      'set is the interval of initial states\n']);
            succ_low = x_low;
            succ_up  = x_up;
            delete_logger();
            return
        elseif t_final < t_init
            % Test if the time range is not properly defined
            log.error('Inconsistent time range provided (t_final < t_init)')
        end
    else
        log.error(['time_vector can only contain 1 (for discrete-time system) ' ...
                   'or 2 elements (for continuous-time system)']);
    end


    %% Check consistency of user-provided intervals
    assert(all(x_low <= x_up), 'Inconsistent interval of initial states provided: need x_low <= x_up')
    assert(all(p_low <= p_up), 'Inconsistent interval of inputs provided: need p_low <= p_up')


    %% Check if over-approximation of the reachable set can be skipped
    if ~force_method && all(isequal(x_low, x_up)) && all(isequal(p_low, p_up))
        % Both intervals contain a single point: simply compute the successor
        log.info('Provided intervals are singletons: exact reachable set is a singleton too\n');

        ode_solver  = parameters.ode_solver;
        ode_options = parameters.ode_options;
        [~, x_traj] = ode_solver(@(t,x) System_description(t, x, p_low), ...
                                 [t_init, t_final], x_low, ode_options);
        succ_low = x_traj(end, :)';
        succ_up  = succ_low;
        delete_logger();
        return
    end


    % =====================================================================
    % Continuous-time methods
    % =====================================================================

    %% 3 - Continuous-time mixed-monotonicity
    % This method is an extension of method 1 to any system with bounded
    % Jacobian matrices.
    %
    % Requirements:
    %   - Continuous-time system
    %   - One of the following two user-provided items:
    %       Signs  of the Jacobian matrices in ./Input_files/UP_Jacobian_Signs.m
    %       Bounds of the Jacobian matrices in ./Input_files/UP_Jacobian_Bounds.m

    bool_skip_CTMM = false;
    if isnan(OA_method) || OA_method == 3
        tic_CTMM = tic;
        log.info('(3) Checking conditions for continuous-time mixed-monotonicity method...\n');

        % Check Input_files/UP_Jacobian_Bounds.m
        [J_x_low, J_x_up, J_p_low, J_p_up] = UP_Jacobian_Bounds(t_init, t_final, x_low, x_up, p_low, p_up);

        if any(isnan(J_x_low(~logical(eye(n_x))))) || any(isnan(J_x_up(~logical(eye(n_x))))) ...
                || any(isnan(J_p_low(:))) || any(isnan(J_p_up(:)))
            % Jacobian bounds undefined: try Input_files/UP_Jacobian_Signs.m
            [J_x_signs, J_p_signs] = UP_Jacobian_Signs(t_init, t_final, x_low, x_up, p_low, p_up);

            if any(isnan(J_x_signs(~logical(eye(n_x))))) || any(isnan(J_p_signs(:)))
                % Neither bounds nor signs are defined
                if OA_method == 3
                    % Return an error if this method was specifically requested
                    log.error('%s (OA_method=%d), but:\n%s\n%s', ...
                        'Continuous-time mixed-monotonicity approach selected', OA_method, ...
                        ' - Jacobian bounds are not defined in Input_files/UP_Jacobian_Bounds.m', ...
                        ' - Jacobian signs are not defined in Input_files/UP_Jacobian_Signs.m');
                else
                    log.info('Neither Jacobian bounds nor signs are provided.\n');
                    % Skip flag: move on to the next method when OA_method was undefined
                    bool_skip_CTMM = true;
                end
            else
                % Sign matrices are defined: convert them into simple bounds
                %   [+1, +1] if positive, [-1, -1] if negative, [0, 0] otherwise
                J_x_low = (J_x_signs > 0) - (J_x_signs < 0);
                J_x_up  = J_x_low;
                J_p_low = (J_p_signs > 0) - (J_p_signs < 0);
                J_p_up  = J_p_low;
                log.info('Read Jacobian signs from Input_files/UP_Jacobian_Signs.m\n');
            end
        else
            log.info('Read Jacobian bounds from Input_files/UP_Jacobian_Bounds.m\n');
        end

        % Apply the OA method only if Jacobian bounds are available
        if ~bool_skip_CTMM
            log.info('Call of the continuous-time mixed-monotonicity over-approximation method ...\n');
            [succ_low, succ_up] = OA_3_CT_Mixed_Monotonicity(t_init, t_final, ...
                                                              x_low, x_up, p_low, p_up, ...
                                                              J_x_low, J_x_up, J_p_low, J_p_up);
            toc_CTMM = toc(tic_CTMM);
            log.runtime('Checking conditions and calling continuous-time mixed-monotonicity method took %f seconds\n', toc_CTMM);
            delete_logger();
            return
        else
            % Otherwise, move on to checking the next method
            toc_CTMM = toc(tic_CTMM);
            log.runtime('Checking conditions for continuous-time mixed-monotonicity method took %f seconds\n', toc_CTMM);
        end
    end

    %% Error message if we reach the end of this file (this should not be possible)
    log.error('None of the over-approximation methods could be applied.')

end
%------------- END OF CODE --------------
