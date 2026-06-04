% =========================================================================
% File: UP_Jacobian_Function.m
%
% Purpose:
%   User-provided Jacobian functions of the system dynamics, evaluated at
%   a given state and input.
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
function [J_x, J_p] = UP_Jacobian_Function(t_init, t, x, p)

    n_x = length(x);
    n_p = length(p);

    %% Default values as NaN (not a number)
    J_x = NaN(n_x);
    J_p = NaN(n_x, n_p);

    %% User-provided Jacobian functions depending on inputs: t_init, t, x, p

    % If System_description.m has no input variable 'p', uncomment below:
    % J_p = zeros(n_x, n_p);

end
%------------- END OF CODE --------------
