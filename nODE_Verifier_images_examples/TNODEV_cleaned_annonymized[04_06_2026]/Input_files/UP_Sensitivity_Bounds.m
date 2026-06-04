% =========================================================================
% File: UP_Sensitivity_Bounds.m
%
% Purpose:
%   User-provided bounds on the sensitivity matrices for a continuous-time
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
function [S_x_low, S_x_up, S_p_low, S_p_up] = UP_Sensitivity_Bounds(t_init, t_final, x_low, x_up, p_low, p_up)

    n_x = length(x_low);
    n_p = length(p_low);

    %% Default values as NaN (not a number)
    S_x_low = NaN(n_x);
    S_x_up  = NaN(n_x);
    S_p_low = NaN(n_x, n_p);
    S_p_up  = NaN(n_x, n_p);

    %% User-provided sensitivity bounds
    % Either global bounds, or local bounds depending on inputs:
    % t_init, t_final, x_low, x_up, p_low, p_up

    % If System_description.m has no input variable 'p', uncomment below:
    % S_p_low = zeros(n_x, n_p);
    % S_p_up  = zeros(n_x, n_p);

end
%------------- END OF CODE --------------
