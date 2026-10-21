% =========================================================================
% File: dynamicsACC.m
%
% Purpose:
%   Continuous-time dynamics of the 6-state ACC benchmark plant
%   (lead car + ego car).
%
% Description:
%   Returns the time derivative dx of the 6D state vector for the
%   Adaptive Cruise Control benchmark, used together with a separately
%   defined controller that supplies the ego-car acceleration command.
%
%   State layout:
%       x1, x2, x3 : lead car position, velocity, internal state
%       x4, x5, x6 : ego  car position, velocity, internal state
%
% Inputs:
%   x     : 6x1 state vector
%   a_ego : commanded ego-car acceleration (control input)
%
% Outputs:
%   dx : 6x1 state derivative
%
% Part of the TNODEV Toolbox
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
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
