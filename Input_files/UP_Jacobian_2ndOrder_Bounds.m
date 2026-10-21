% =========================================================================
% File: UP_Jacobian_2ndOrder_Bounds.m
%
% Purpose:
%   User-provided bounds on the second-order Jacobian matrices for a
%   continuous-time system.
%
% Description:
%   Used in:
%     - Utilities/Sensitivity_bounds_choice.m
%         to compute bounds on the sensitivity matrices of a
%         continuous-time system, used in over-approximation method 4
%         (sub-method 4-3) in OA_methods/OA_4_CT_Sampled_data_MM.m
%
%   The user can either provide global bounds, or a function of the input
%   arguments (t_init, t_final, x_low, x_up, p_low, p_up) returning local
%   bounds.
%
%   Given the first-order Jacobian definitions:
%     J_x(t,x,p) = d(System_description(t,x,p))/dx
%     J_p(t,x,p) = d(System_description(t,x,p))/dp
%   the second-order Jacobians are:
%     J_xx(t,x,p) = d J_x(t,x,p) / dx
%     J_xp(t,x,p) = d J_x(t,x,p) / dp
%     J_pp(t,x,p) = d J_p(t,x,p) / dp
%     J_px(t,x,p) = d J_p(t,x,p) / dx
%
% Inputs:
%   t_init       : initial time
%   t_final      : time at which the reachable set is approximated
%   [x_low,x_up] : interval of initial states (at time t_init)
%   [p_low,p_up] : interval of allowed input values
%
% Outputs:
%   Bounds on the four second-order Jacobians, returned as 2D matrices:
%     [J_xx_low, J_xx_up] : (n_x, n_x^2)
%     [J_xp_low, J_xp_up] : (n_x, n_x*n_p)
%     [J_pp_low, J_pp_up] : (n_x, n_p^2)
%     [J_px_low, J_px_up] : (n_x, n_x*n_p)
%
% Part of the TIRA framework (used by the TNODEV Toolbox)
%
% Author:
%   Pierre-Jean Meyer   <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 2nd of December 2021
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
