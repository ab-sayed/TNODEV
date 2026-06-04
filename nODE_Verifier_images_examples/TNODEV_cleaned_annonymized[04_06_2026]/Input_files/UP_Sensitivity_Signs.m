% =========================================================================
% File: UP_Sensitivity_Signs.m
%
% Purpose:
%   User-provided signs on the sensitivity matrices for a continuous-time
%   system.
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
