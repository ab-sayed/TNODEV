% =========================================================================
% File: OA_3_CT_Mixed_Monotonicity.m
%
% Purpose:
%   Reachable-set over-approximation based on Jacobian bounds of a
%   continuous-time system.
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
function [succ_low, succ_up] = OA_3_CT_Mixed_Monotonicity(t_init, t_final, ...
                                                          x_low, x_up, p_low, p_up, ...
                                                          J_x_low, J_x_up, J_p_low, J_p_up)

    % State and input dimensions
    n_x = length(x_low);
    n_p = length(p_low); %#ok<NASGU>

    % Extract ODE solver to use
    run('Solver_parameters.m');
    ode_solver  = parameters.ode_solver;
    ode_options = parameters.ode_options;

    % Check consistency of user-provided intervals
    assert(all(J_x_low(~logical(eye(n_x))) <= J_x_up(~logical(eye(n_x)))), ...
           'Inconsistent bounds of the state Jacobian provided: need J_x_low <= J_x_up')
    assert(all(J_p_low(:) <= J_p_up(:)), ...
           'Inconsistent bounds of the input Jacobian provided: need J_p_low <= J_p_up')

    % Simulate the dynamical system duplicating the decomposition function
    % between t_init and t_final, from stacked initial state [x_low; x_up]
    % and with stacked constant input [p_low; p_up].
    [~, xstack_traj] = ode_solver(...
        @(t, x_stack) Duplicated_dynamics(t, x_stack, [p_low; p_up], ...
                                          J_x_low, J_x_up, J_p_low, J_p_up), ...
        [t_init, t_final], [x_low; x_up], ode_options);

    % Extract from the obtained successor (in R^(2*n_x)) an interval
    % over-approximation of the reachable set:
    succ_low = xstack_traj(end, 1:n_x)';        % first  n_x elements -> lower bound
    succ_up  = xstack_traj(end, n_x+1:end)';    % last   n_x elements -> upper bound
end


%% Dynamical system duplicating the decomposition function
% Inputs:
%   t                : current time
%   x_stack          : stack in R^(2*n_x) of 2 state vectors
%   p_stack          : stack in R^(2*n_p) of 2 input vectors
%   [J_x_low,J_x_up] : bounds of the state Jacobian
%   [J_p_low,J_p_up] : bounds of the input Jacobian
% Output:
%   dx_stack         : vector field of the duplicated dynamics in R^(2*n_x)
function dx_stack = Duplicated_dynamics(t, x_stack, p_stack, J_x_low, J_x_up, J_p_low, J_p_up)

    % State and input dimensions
    n_x = length(x_stack) / 2;
    n_p = length(p_stack) / 2;

    % Break state and input stacks into 2 states and 2 inputs
    x1 = x_stack(1:n_x);
    x2 = x_stack(n_x+1:end);
    p1 = p_stack(1:n_p);
    p2 = p_stack(n_p+1:end);

    % Dynamics of the duplicated system  h: R x R^(2*n_x) x R^(2*n_p) -> R^(2*n_x)
    %   [dx1; dx2] = h(t, x1, x2, p1, p2)
    %              = [g(t, x1, x2, p1, p2); g(t, x2, x1, p2, p1)]
    dx_stack = [MM_decomposition_function(t, x1, x2, p1, p2, J_x_low, J_x_up, J_p_low, J_p_up); ...
                MM_decomposition_function(t, x2, x1, p2, p1, J_x_low, J_x_up, J_p_low, J_p_up)];
end
%------------- END OF CODE --------------
