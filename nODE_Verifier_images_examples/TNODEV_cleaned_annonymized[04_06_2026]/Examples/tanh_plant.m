% =========================================================================
% File: tanh_plant.m
%
% Purpose:
%   Continuous dynamics of the 8-state Adaptive Cruise Control (ACC)
%   plant, where the embedded Neural ODE uses a tanh activation.
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function dx = tanh_plant(x, u) 

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
