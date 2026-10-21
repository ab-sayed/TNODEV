% =========================================================================
% File: UP_Jacobian_2ndOrder_Bounds.m
%
% Purpose:
%   User-provided bounds on the second-order Jacobian matrices for a
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
function [J_xx_low, J_xx_up, J_xp_low, J_xp_up, J_pp_low, J_pp_up, J_px_low, J_px_up] = ...
            UP_Jacobian_2ndOrder_Bounds(t_init, t_final, x_low, x_up, p_low, p_up)

    n_x = length(x_low);
    n_p = length(p_low);

    %% Default values as NaN (not a number)
    J_xx_low = NaN(n_x, n_x, n_x);
    J_xp_low = NaN(n_x, n_x, n_p);
    J_pp_low = NaN(n_x, n_p, n_p);
    J_px_low = NaN(n_x, n_p, n_x);
    J_xx_up  = NaN(n_x, n_x, n_x);
    J_xp_up  = NaN(n_x, n_x, n_p);
    J_pp_up  = NaN(n_x, n_p, n_p);
    J_px_up  = NaN(n_x, n_p, n_x);

    %% User-provided second-order Jacobian bounds as 3D matrices, with:
    %   size(J_xx_low) = [n_x, n_x, n_x]   and   J_xx_low(i,j,k) = d J_x(i,j) / d x(k)
    %   size(J_xp_low) = [n_x, n_x, n_p]   and   J_xp_low(i,j,k) = d J_x(i,j) / d p(k)
    %   size(J_pp_low) = [n_x, n_p, n_p]   and   J_pp_low(i,j,k) = d J_p(i,j) / d p(k)
    %   size(J_px_low) = [n_x, n_p, n_x]   and   J_px_low(i,j,k) = d J_p(i,j) / d x(k)
    % These 3D matrices are converted to the appropriate 2D form at the
    % end of this file.

    % If System_description.m has no input variable 'p', uncomment below:
    % J_xp_low = zeros(n_x, n_x, n_p);
    % J_pp_low = zeros(n_x, n_p, n_p);
    % J_px_low = zeros(n_x, n_p, n_x);
    % J_xp_up  = zeros(n_x, n_x, n_p);
    % J_pp_up  = zeros(n_x, n_p, n_p);
    % J_px_up  = zeros(n_x, n_p, n_x);

    %% Convert these 3D matrices to their corresponding 2D form:
    %   size(J_xx_low) = [n_x, n_x^2]
    %   size(J_xp_low) = [n_x, n_x*n_p]
    %   size(J_pp_low) = [n_x, n_p^2]
    %   size(J_px_low) = [n_x, n_x*n_p]
    % e.g.: J_xx_low = [J_xx_low(:,1,:) , ... , J_xx_low(:,n_x,:)]
    J_xx_low = reshape(permute(J_xx_low, [1,3,2]), [n_x, n_x^2]);
    J_xx_up  = reshape(permute(J_xx_up,  [1,3,2]), [n_x, n_x^2]);
    J_xp_low = reshape(permute(J_xp_low, [1,3,2]), [n_x, n_x*n_p]);
    J_xp_up  = reshape(permute(J_xp_up,  [1,3,2]), [n_x, n_x*n_p]);
    J_pp_low = reshape(permute(J_pp_low, [1,3,2]), [n_x, n_p^2]);
    J_pp_up  = reshape(permute(J_pp_up,  [1,3,2]), [n_x, n_p^2]);
    J_px_low = reshape(permute(J_px_low, [1,3,2]), [n_x, n_x*n_p]);
    J_px_up  = reshape(permute(J_px_up,  [1,3,2]), [n_x, n_x*n_p]);

end
%------------- END OF CODE --------------
