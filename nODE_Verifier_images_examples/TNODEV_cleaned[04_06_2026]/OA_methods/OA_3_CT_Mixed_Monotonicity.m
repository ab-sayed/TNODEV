% =========================================================================
% File: OA_3_CT_Mixed_Monotonicity.m
%
% Purpose:
%   Reachable-set over-approximation based on Jacobian bounds of a
%   continuous-time system.
%
% Description:
%   For each state dimension, state and input vectors are obtained based
%   on the Jacobian bounds, and are then used to define a decomposition
%   function in Utilities/MM_decomposition_function.m. The
%   over-approximation is obtained by computing a single successor of a
%   dynamical system that duplicates the decomposition function.
%
%   This method (source paper 2) relies on a generalization (to any
%   continuous-time system with bounded Jacobian matrices) of the
%   continuous-time mixed-monotonicity definition (source paper 1).
%   The state Jacobian does not need to be bounded on its diagonal.
%
%   This generalization covers as particular cases:
%     - the mixed-monotonicity approach (system with sign-stable Jacobians)
%     - the monotonicity approach (OA_methods/OA_1_CT_Monotonicity.m)
%   For a monotone system, this method yields the same result as method 1
%   without needing to check whether the system is indeed monotone, but
%   at the cost of slightly worse scalability (computes 2*n_x successors
%   instead of 2).
%
% Source paper 1 (generalization of continuous-time mixed-monotonicity):
%   L. Yang, O. Mickelin and N. Ozay, "On sufficient conditions for mixed
%   monotonicity". ArXiv preprint, https://arxiv.org/abs/1803.04528
%
% Source paper 2 (reachability analysis based on this generalization):
%   P.-J. Meyer and D. V. Dimarogonas, "Hierarchical decomposition of LTL
%   synthesis problem for mixed-monotone control systems". ArXiv preprint,
%   https://arxiv.org/abs/1712.06014
%
% Inputs:
%   t_init             : initial time
%   t_final            : time at which the reachable set is approximated
%   [x_low,x_up]       : interval of initial states (at time t_init)
%   [p_low,p_up]       : interval of allowed input values
%   [J_x_low,J_x_up]   : bounds of the Jacobian with respect to the state
%   [J_p_low,J_p_up]   : bounds of the Jacobian with respect to the input
%
% Outputs:
%   [succ_low,succ_up] : interval over-approximation of the reachable set
%                        (at time t_final)
%
% Part of the TIRA framework (used by the TNODEV Toolbox)
%
% Authors:
%   Alex Devonport      <alex_devonport    -AT- berkeley.edu>,  EECS, UC Berkeley
%   Pierre-Jean Meyer   <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 2nd of July 2021
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
