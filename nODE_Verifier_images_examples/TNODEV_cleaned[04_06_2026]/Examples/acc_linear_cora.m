% =========================================================================
% File: acc_linear_cora.m
%
% Purpose:
%   Linear 3rd-order Neural ODE plant of the Adaptive Cruise Control
%   (ACC) example, formatted for CORA reachability analysis.
%
% Description:
%   Defines the continuous-time dynamics of the 8-state ACC plant in
%   which the embedded Neural ODE has no activation function (purely
%   linear). The network weights and biases are passed via global
%   variables so that the function signature stays compatible with
%   CORA's nonlinearSys class.
%
% Inputs:
%   x : 8x1 state vector
%   u : input (unused; kept to match CORA's expected signature)
%
% Outputs:
%   dx : 8x1 state derivative
%
% Globals:
%   w1_acc_lin, b1_acc_lin : input-layer weights and biases
%   w2_acc_lin, b2_acc_lin : output-layer weights and biases
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
function dx = acc_linear_cora(x, u) %#ok<INUSD>

    % Network weights / biases shared via globals (set by the caller)
    global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin

    % Extract the four states that feed into the network. We use
    % explicit concatenation rather than x([3,6,7,8]) because CORA's
    % overloaded zonotope objects can fail with multi-index arrays.
    nn_in = [x(3); x(6); x(7); x(8)];

    % Forward pass through the linear network (no activation function)
    y      = w1_acc_lin * nn_in + b1_acc_lin;
    nn_out = w2_acc_lin * y     + b2_acc_lin;

    % Assemble the 8D state derivative
    dx = [
        x(2);         % dx1 = x2     (lead car velocity)
        x(3);         % dx2 = x3     (lead car acceleration)
        nn_out(1);    % dx3 = NN output 1
        x(5);         % dx4 = x5     (ego car velocity)
        x(6);         % dx5 = x6     (ego car acceleration)
        nn_out(2);    % dx6 = NN output 2
        nn_out(3);    % dx7 = NN output 3
        nn_out(4)     % dx8 = NN output 4
    ];
end
%------------- END OF CODE --------------
