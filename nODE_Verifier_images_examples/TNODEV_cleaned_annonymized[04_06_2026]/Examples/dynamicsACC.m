% =========================================================================
% File: dynamicsACC.m
%
% Purpose:
%   Continuous-time dynamics of the 6-state ACC benchmark plant
%   (lead car + ego car).
%
% Part of the TNODEV Toolbox
% % Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function dx = dynamicsACC(x, a_ego)

    mu     = 0.0001;   % friction parameter
    a_lead = -2;       % constant lead-car acceleration command

    % --- Lead car dynamics ---
    dx(1,1) = x(2);
    dx(2,1) = x(3);
    dx(3,1) = -2 * x(3) + 2 * a_lead - mu * x(2)^2;

    % --- Ego car dynamics ---
    dx(4,1) = x(5);
    dx(5,1) = x(6);
    dx(6,1) = -2 * x(6) + 2 * a_ego  - mu * x(5)^2;

end
%------------- END OF CODE --------------
