% =========================================================================
% File: UP_Jacobian_Signs.m
%
% Purpose:
%   User-provided signs on the Jacobian matrices for continuous-time or
%   discrete-time systems.
%
% Description:
%   Used in:
%     - OA_methods/TIRA.m
%         to check the requirements for over-approximation methods 1, 3, 5 in
%         OA_methods/OA_1_CT_Monotonicity.m
%         OA_methods/OA_3_CT_Mixed_Monotonicity.m
%         OA_methods/OA_5_DT_Mixed_Monotonicity.m
%
%   The user can either provide global signs, or a function of the input
%   arguments (t_init, t_final, x_low, x_up, p_low, p_up) returning local signs.
%
%   Jacobian definitions:
%     to states:  J_x(t) = d(System_description(t,x,p))/dx
%     to inputs:  J_p(t) = d(System_description(t,x,p))/dp
%
% Inputs:
%   t_init       : initial time
%   t_final      : time at which the reachable set is approximated
%                  (continuous-time systems only; for discrete-time, a
%                  dummy value can be provided)
%   [x_low,x_up] : interval of initial states (at time t_init)
%   [p_low,p_up] : interval of allowed input values
%
% Outputs:
%   J_x_signs : sign matrix of the Jacobian with respect to the state
%   J_p_signs : sign matrix of the Jacobian with respect to the input
%
% Part of the TIRA framework (used by the TNODEV Toolbox)
%
% Authors:
%   Pierre-Jean Meyer   <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Alex Devonport      <alex_devonport    -AT- berkeley.edu>,  EECS, UC Berkeley
%
% Date: 2nd of December 2021
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function [J_x_signs, J_p_signs] = UP_Jacobian_Signs(t_init, t_final, x_low, x_up, p_low, p_up)

    n_x = length(x_low);
    n_p = length(p_low);

    %% Default values as NaN (not a number)
    J_x_signs = NaN(n_x);
    J_p_signs = NaN(n_x, n_p);

    %% User-provided Jacobian signs
    % Either global signs, or local signs depending on inputs:
    % t_init, t_final, x_low, x_up, p_low, p_up

    % If System_description.m has no input variable 'p', uncomment below:
    % J_p_signs = zeros(n_x, n_p);

end
%------------- END OF CODE --------------
