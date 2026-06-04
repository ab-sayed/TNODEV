% =========================================================================
% File: UP_Sensitivity_Signs.m
%
% Purpose:
%   User-provided signs on the sensitivity matrices for a continuous-time
%   system.
%
% Description:
%   Used in:
%     - Utilities/Sensitivity_bounds_choice.m
%         for the over-approximation method 4 in
%         OA_methods/OA_4_CT_Sampled_data_MM.m
%
%   The user can either provide global signs, or a function of the input
%   arguments (t_init, t_final, x_low, x_up, p_low, p_up) returning local signs.
%
%   Sensitivity definitions:
%     to initial states:  S_x(t_final) = d x(t_final ; t_init, x0, p) / d x0
%     to inputs:          S_p(t_final) = d x(t_final ; t_init, x0, p) / d p
%
% Inputs:
%   t_init       : initial time
%   t_final      : time at which the reachable set is approximated
%   [x_low,x_up] : interval of initial states (at time t_init)
%   [p_low,p_up] : interval of allowed input values
%
% Outputs:
%   S_x_signs : sign of the sensitivity of successors (at t_final) to
%               variations of the initial states (at t_init)
%   S_p_signs : sign of the sensitivity of successors (at t_final) to
%               variations of the inputs
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
function [S_x_signs, S_p_signs] = UP_Sensitivity_Signs(t_init, t_final, x_low, x_up, p_low, p_up)

    n_x = length(x_low);
    n_p = length(p_low);

    %% Default values as NaN (not a number)
    S_x_signs = NaN(n_x);
    S_p_signs = NaN(n_x, n_p);

    %% User-provided sensitivity signs
    % Either global signs, or local signs depending on inputs:
    % t_init, t_final, x_low, x_up, p_low, p_up

    % If System_description.m has no input variable 'p', uncomment below:
    % S_p_signs = zeros(n_x, n_p);

end
%------------- END OF CODE --------------
