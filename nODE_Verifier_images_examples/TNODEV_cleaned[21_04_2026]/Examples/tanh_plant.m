% =========================================================================
% File: tanh_plant.m
%
% Purpose:
%   Continuous dynamics of the 8-state Adaptive Cruise Control (ACC)
%   plant, where the embedded Neural ODE uses a tanh activation.
%
% Description:
%   Loads the trained Neural ODE parameters from disk and evaluates the
%   plant ODE  dx = f(x). The four states {x3, x6, x7, x8} are fed into
%   a 1-hidden-layer tanh network whose four outputs drive the velocity
%   / acceleration channels of the lead and ego cars. Vectorised across
%   N column-states for use with sampled trajectories.
%
% Inputs:
%   x : 8xN state matrix (column-vectorised states)
%   u : input (unused; kept for solver-signature compatibility)
%
% Outputs:
%   dx : 8xN state-derivative matrix
%
% Required data files:
%   plant_3rd_order_tanh.mat   (pretrained Neural ODE parameters)
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
function dx = tanh_plant(x, u) %#ok<INUSD>

    % Load pre-trained Neural ODE parameters from disk
    pl = load("plant_3rd_order_tanh.mat");
    w1 = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
    b1 = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
    w2 = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
    b2 = double(extractdata(pl.neuralOdeParameters.fc2.Bias));

    % Pass-through (kinematic) channels of the two cars
    x1 = x(2,:);    % lead car velocity      -> dx1
    x2 = x(3,:);    % lead car acceleration  -> dx2
    x4 = x(5,:);    % ego  car velocity      -> dx4
    x5 = x(6,:);    % ego  car acceleration  -> dx5

    % Forward pass through the 1-hidden-layer tanh network
    %   inputs  : {x3, x6, x7, x8}        (4xN)
    %   outputs : 4 derivative channels   (4xN)
    y  = tanh(w1 * x([3,6,7,8],:) + b1);
    dx = w2 * y + b2;

    % Assemble the 8D state derivative
    dx = [x1; x2; dx(1,:); x4; x5; dx(2:4,:)];

end
%------------- END OF CODE --------------
