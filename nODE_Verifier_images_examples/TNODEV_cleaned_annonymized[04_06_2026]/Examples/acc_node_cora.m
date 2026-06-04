% =========================================================================
% File: acc_node_cora.m
%
% Purpose:
%   Nonlinear (tanh) 3rd-order Neural ODE plant of the Adaptive Cruise
%   Control (ACC) example, formatted for CORA reachability analysis.
%
% Part of the TNODEV Toolbox
% % Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
% =========================================================================

%------------- BEGIN CODE --------------
function dx = acc_node_cora(x, u)

    % Network weights / biases shared via globals (set by the caller)
    global w1_acc b1_acc w2_acc b2_acc

    % Extract the four states that feed into the network. We use
    % explicit concatenation rather than x([3,6,7,8]) because CORA's
    % overloaded zonotope objects can fail with multi-index arrays.
    nn_in = [x(3); x(6); x(7); x(8)];

    % Forward pass through the 1-hidden-layer tanh network
    z      = w1_acc * nn_in + b1_acc;
    h      = tanh(z);
    nn_out = w2_acc * h + b2_acc;

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
