% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 21 October 2025
% Last update: 21 October 2025
% Last revision: October 2025



%% Hub function serving as interface between the reachability problem definition and the library of over-approximation methods available in folder ./OA_methods
% This function gathers the call of all over-approximation methods, each
% preceded by the tests of their requirements.
% If a specific method is requested by the user (either as the last
% argument of this function call, or defined in Solver_parameters.m), the
% function only checks the requirements for this method and then calls the
% over-approximation function (or returns an error if the requirements are
% not satisfied).
% Otherwise, the function applies the first over-approximation method whose
% requirements are met (in the order detailed below).

% Source:
% P.-J. Meyer, A. Devonport and M. Arcak, "Interval Reachability Analysis: 
% Bounding Trajectories of Uncertain Systems with Boxes for Control and 
% Verification". Springer, 2021. DOI: 10.1007/978-3-030-65110-7

% List of inputs
%   time_vector: time information for the reachability analysis
%       1D vector for discrete-time system: time_vector=t_init
%       2D vector for continuous-time system: time_vector=[t_init,t_final]
%   [x_low,x_up]: interval of initial states (at time t_init)
%   [p_low,p_up]: interval of allowed input values

% Optional input
%   OA_method: integer to request a specific over-approximation method
%       if not provided, checks if it is defined in Solver_parameters.m
%       if not defined in Solver_parameters.m either, the file picks the
%       first method whose requirements are met in the following order:
%       3 - Continuous-time mixed-monotonicity
%       4 - Continuous-time sampled-data mixed-monotonicity

% List of outputs
%   [succ_low,succ_up]: interval over-approximation of the reachable set
%       after one step for discrete-time system
%       or at time t_final for continuous-time system

% Authors:
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Alex Devonport, <alex_devonport -AT- berkeley.edu>, EECS, UC Berkeley
%   Neelay Junnarkar, <neelay.junnarkar -AT- berkeley.edu>, EECS, UC Berkeley
%
% Date: 21th of October 2025
% Last update: 21th of October 2025
% Last revision: th of October 2025


function [succ_low,succ_up] = TIRA(time_vector,x_low,x_up,p_low,p_up,OA_method,force_method)
if nargin < 7
    force_method = false;
end

%% Initialization

% State and input dimensions
n_x = length(x_low);
n_p = length(p_low);

% % Extract method choices and parameters in the 'parameters' structure
run('Solver_parameters.m');


% Extract method choices and parameters from `get_solver_parameters.m`
% global g_sensitivity_bounds_method
% if isnan(g_sensitivity_bounds_method)
%     g_sensitivity_bounds_method = NaN;
% end
% parameters = get_solver_parameters(n_x, n_p, g_sensitivity_bounds_method);

% Use the over-approximation method choice from Solver_parameters.m
%   if it was not provided as input argument of this function
if nargin < 6
    OA_method = parameters.OA_method;
end

% Create logger
[log, delete_logger] = logger.get_logger();
log.info('Start of TIRA call\n');

%% Extract time information

if length(time_vector) == 2
    % 2D vector: continuous-time system (time_vector = [t_init,t_final])
    t_init = time_vector(1);
    t_final = time_vector(2);
    if t_final == t_init
        % Test if the time range is a single time instant
        log.info('The initial and final times are equals: the reachable set is the interval of initial states\n')
        succ_low = x_low;
        succ_up = x_up;
        delete_logger();
        return
    elseif t_final < t_init
        % Test if the time range is not properly defined
        log.error('Inconsistent time range provided (t_final < t_init)')
    end
else
    log.error('time_vector can only contain 1 (for discrete-time system) or 2 elements (for continuous-time system)')
end

%% Check if the system is defined
% system_eval = System_description(t_init,x_low,p_low);
% assert(~any(isnan(system_eval)),'The considered system needs to be defined in System_description.m')

%% Check consistency of user-provided intervals
assert(all(x_low<=x_up),'Inconsistent interval of initial states provided: need x_low <= x_up')
assert(all(p_low<=p_up),'Inconsistent interval of inputs provided: need p_low <= p_up')

%% Check if over-approximation of the reachable set can be skipped
if ~force_method && all(isequal(x_low,x_up)) && all(isequal(p_low,p_up))
    % Both interval contain a single point: simply compute the successor
    log.info('Provided intervals are singletons: exact reachable set is a singleton too\n')
    % Continuous-time successor
        ode_solver = parameters.ode_solver;
        ode_options = parameters.ode_options;
        [~,x_traj] = ode_solver(@(t,x) System_description(t,x,p_low), [t_init, t_final], x_low, ode_options);
        succ_low = x_traj(end,:)';
        succ_up = succ_low;
        delete_logger();
    return
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time methods %
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 3 - Continuous-time mixed-monotonicity
% This method is an extension of method 1 to any system with bounded
% Jacobian matrices.

% *Requirements:
% - Continuous-time system
% - One of the following 2 user-provided items:
%     Signs of the Jacobian matrices in ./Input_files/UP_Jacobian_Signs.m
%     Bounds of the Jacobian matrices in ./Input_files/UP_Jacobian_Bounds.m

bool_skip_CTMM = false;
if isnan(OA_method) || OA_method == 3
    tic_CTMM = tic;
    log.info('(3) Checking conditions for continuous-time mixed-monotonicity method...\n')
    
        
    % Check Input_files/UP_Jacobian_Bounds.m
    [J_x_low,J_x_up,J_p_low,J_p_up] = UP_Jacobian_Bounds(t_init,t_final,x_low,x_up,p_low,p_up);
    if any(isnan(J_x_low(~logical(eye(n_x))))) || any(isnan(J_x_up(~logical(eye(n_x))))) || any(isnan(J_p_low(:))) || any(isnan(J_p_up(:)))
        % If the Jacobian bounds are not defined,
        % Check Input_files/UP_Jacobian_Signs.m
        [J_x_signs,J_p_signs] = UP_Jacobian_Signs(t_init,t_final,x_low,x_up,p_low,p_up);
        if any(isnan(J_x_signs(~logical(eye(n_x))))) || any(isnan(J_p_signs(:)))
            % If the Jacobian signs are not defined either
            if OA_method == 3
                % Return an error if this method was specifically requested
                log.error('%s (OA_method=%d), but:\n%s\n%s', ...
                    'Continuous-time mixed-monotonicitiy approach selected', OA_method, ...
                    ' - Jacobian bounds are not defined in Input_files/UP_Jacobian_Bounds.m', ...
                    ' - Jacobian signs are not defined in Input_files/UP_Jacobian_Signs.m')
            else
                log.info('Neither Jacobian bounds nor signs are provided.\n')
                % Flag to go to the next method if OA_method was undefined
                bool_skip_CTMM = true;
            end
        else
            % If sign matrices are defined, convert them into simple bounds
            % [1,1] if positive, [-1,-1] if negative, [0,0] otherwise
            J_x_low = (J_x_signs>0) - (J_x_signs<0);
            J_x_up = J_x_low;
            J_p_low = (J_p_signs>0) - (J_p_signs<0);
            J_p_up = J_p_low;
            log.info('Read Jacobian signs from Input_files/UP_Jacobian_Signs.m\n')
        end
    else
        log.info('Read Jacobian bounds from Input_files/UP_Jacobian_Bounds.m\n')
    end
    
    % Only apply the over-approximation method if we have Jacobian bounds
    if ~bool_skip_CTMM
        log.info('Call of the continuous-time mixed-monotonicity over-approximation method ...\n')
        % Call over-approximation method
        [succ_low,succ_up] = OA_3_CT_Mixed_Monotonicity(t_init,t_final,x_low,x_up,p_low,p_up,J_x_low,J_x_up,J_p_low,J_p_up);
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

%% 4 - Continuous-time sampled-data mixed-monotonicity
% This method applies the discrete-time mixed-monotonicity method (5)
% to the sampled-data version of a continuous-time system.
% The requirements on the Jacobian of the discrete-time system thus
% translate into requirements on the sensitivity matrices of the 
% continuous-time system.

% *Requirements:
% - Continuous-time system
% - Input p constant over the considered time range [t_init,t_final].
% - Optional user-provided inputs amongst the following items:
%     Signs of the sensitivity matrices in ./Input_files/UP_Sensitivity_Signs.m
%     Bounds of the sensitivity matrices in ./Input_files/UP_Sensitivity_Bounds.m
%     Bounds of the Jacobian matrices in ./Input_files/UP_Jacobian_Bounds.m
%     Bounds of the 2nd order Jacobian matrices in ./Input_files/UP_Jacobian_2ndOrder_Bounds.m
%     Jacobian functions in ./Input_files/UP_Jacobian_Function.m
% *If none of these optional input is provided, the method can still
% be called using default submethod 4-0 "Sampling and falsification".

if isnan(OA_method) || OA_method == 4
    tic_SDMM = tic;
    log.info('(4) Checking conditions for sampled-data mixed-monotonicity method based...\n')
    log.warning('This method over-approximates the reachable set only for constant input functions.\n')
    
    % Extract or compute sensitivity bounds
    % (submethod used depends on the available user-provided inputs)
    [S_x_low,S_x_up,S_p_low,S_p_up] = Sensitivity_bounds_choice(...
        t_init, t_final, x_low, x_up, p_low, p_up,...
        parameters.sensitivity_bounds_method,...
        parameters.sensitivity_equation_solver);
    
    % Call over-approximation method
    log.info('Call of the sampled-data mixed-monotonicity over-approximation method ...\n')
    [succ_low,succ_up] = OA_4_CT_Sampled_data_MM(t_init,t_final,x_low,x_up,p_low,p_up,S_x_low,S_x_up,S_p_low,S_p_up);
    
    toc_SDMM = toc(tic_SDMM);
    log.runtime('Checking conditions and calling sampled-data mixed-monotonicity method took %f seconds\n', toc_SDMM);
    delete_logger();
    return;
end


%% Error message if we reach the end of this file (this should not be possible)
log.error('None of the over-approximation methods could be applied.')
