% =========================================================================
% File: Jacob_bounds_element_wise.m
%
% Purpose:
%   Computes element-wise Jacobian bounds  [J_lb, J_ub]  of the
%   continuous-time vector field of any system supported by the toolbox,
%   evaluated over an input interval [a, b] (typically the CORA
%   reachable-tube hull).
%
% Description:
%   Dispatches on system_choice to a per-system helper that performs
%   the element-wise interval-arithmetic Jacobian computation for the
%   neural-network / physics dynamics of that system:
%
%     14   FPA                 (5D)
%     15   Spiral_non          (2D)
%     16   MNIST Neural ODE    (676D, linear)   -> J = Aout_global (constant)
%     17   ACC nonlinear       (8D, tanh)
%     18   Single Pendulum     (3D, 1-hidden-layer NODE surrogate)
%     19   Copepod (ReLU)      (256D)
%     20   Copepod (tanh)      (32D / 256D)
%     21   Spiral_linear       (2D, constant Jacobian)
%     22   Cartpole CTRNN      (12D)
%     24   ACC linear          (8D, constant Jacobian)
%
%   Interval bounds for tanh-derivative (sech^2), cos and sin are
%   provided by the local helpers at the end of this file.
%
% Inputs:
%   a              : column vector of lower bounds for x (typically the
%                    RT lower hull from CORA)
%   b              : column vector of upper bounds for x (typically the
%                    RT upper hull from CORA)
%   system_choice  : integer system identifier (see list above)
%
% Outputs:
%   J_lb : element-wise lower-bound matrix of the Jacobian
%   J_ub : element-wise upper-bound matrix of the Jacobian
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
function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice)
% Jacob_bounds_element_wise.m: computes Jacobian bounds using element-wise
% interval arithmetic.
%
% Inputs:
%   a - vector of lower bounds for x (Now representing the Reachable Tube)
%   b - vector of upper bounds for x (Now representing the Reachable Tube)
%   system_choice - ID of the system (14=FPA, 15=Spiral, 16=MNIST, 17=ACC)
%
% Outputs:
%   J_lb - matrix of lower bounds for the Jacobian
%   J_ub - matrix of upper bounds for the Jacobian

% Validate inputs
if ~isequal(size(a), size(b)) || any(a > b)
    error('a and b must be same size and a <= b element-wise');
end

n = length(a);

switch system_choice
    case 15
        % === SPIRAL_NON (2D) ===
        if n ~= 2
            error('Spiral_non expects 2D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b);
        
    case 14
        % === FPA (5D) ===
        if n ~= 5
            error('FPA expects 5D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b);
        
    case 16
        % === MNIST Neural ODE (676D - Linear) ===
        % Dynamics: dx/dt = Aout * x + Bout
        % Jacobian: J(x) = Aout (Constant)
        
        % Linear System: J = Aout_global
        global Aout_global
        
        if isempty(Aout_global)
            error('Jacob_bounds_element_wise: Aout_global is empty. Ensure reach_cnn_tiny_NODE_RA sets it.');
        end
        
        % For a linear system, Jacobian is constant and independent of state x.
        J_lb = Aout_global;
        J_ub = Aout_global;

    case 17
        % === ACC Neural ODE Plant (8D) ===
        if n ~= 8
            error('ACC expects 8D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_acc(a, b); 
    
    case 24
        % === Linear 3rd-order NODE ACC Plant (8D) ===
        if n ~= 8
            error('Linear ACC NODE expects 8D state, got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_acc_linear(a, b);

    case 18
        % === Single Pendulum Neural ODE (3D) ===
        if n ~= 3
            error('Single Pendulum expects 3D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_pendulum(a, b);
    
    case 19
        % === Copepod Neural ODE (256D, ReLU bottleneck) ===
        % Dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
        % Jacobian: J(x) = W2 * diag(ReLU'(W1*x + b1)) * W1
        % ReLU'(z) = 0 if z<0, 1 if z>0, [0,1] if interval contains 0
        
        global W1_ode b1_ode W2_ode b2_ode
        
        if isempty(W1_ode)
            error('Jacob_bounds_element_wise: W1_ode not set for system 19.');
        end
        
        bottleneck = size(W1_ode, 1);  % 16
        state_dim = size(W1_ode, 2);   % 256
        
        % 1. Compute bounds for z = W1*x + b1
        z_lb = zeros(bottleneck, 1);
        z_ub = zeros(bottleneck, 1);
        for i = 1:bottleneck
            temp_lb = 0; temp_ub = 0;
            for j = 1:state_dim
                if W1_ode(i,j) >= 0
                    temp_lb = temp_lb + W1_ode(i,j) * a(j);
                    temp_ub = temp_ub + W1_ode(i,j) * b(j);
                else
                    temp_lb = temp_lb + W1_ode(i,j) * b(j);
                    temp_ub = temp_ub + W1_ode(i,j) * a(j);
                end
            end
            z_lb(i) = temp_lb + b1_ode(i);
            z_ub(i) = temp_ub + b1_ode(i);
        end
        
        % 2. Compute ReLU derivative bounds over each bottleneck neuron
        % ReLU'(z): 0 if z<0, 1 if z>0, [0,1] if interval spans zero
        relu_deriv_lb = zeros(bottleneck, 1);
        relu_deriv_ub = zeros(bottleneck, 1);
        for i = 1:bottleneck
            if z_ub(i) <= 0
                % Entire interval negative -> ReLU' = 0
                relu_deriv_lb(i) = 0;
                relu_deriv_ub(i) = 0;
            elseif z_lb(i) > 0
                % Entire interval positive -> ReLU' = 1
                relu_deriv_lb(i) = 1;
                relu_deriv_ub(i) = 1;
            else
                % Interval spans zero -> ReLU' in [0, 1]
                relu_deriv_lb(i) = 0;
                relu_deriv_ub(i) = 1;
            end
        end
        
        % 3. Compute Jacobian bounds: J = W2 * diag(relu') * W1
        J_lb = zeros(state_dim, state_dim);
        J_ub = zeros(state_dim, state_dim);
        for i = 1:state_dim
            for j = 1:state_dim
                sum_lb = 0; sum_ub = 0;
                for k = 1:bottleneck
                    prod_w = W2_ode(i,k) * W1_ode(k,j);
                    r_min = relu_deriv_lb(k);
                    r_max = relu_deriv_ub(k);
                    if prod_w >= 0
                        sum_lb = sum_lb + prod_w * r_min;
                        sum_ub = sum_ub + prod_w * r_max;
                    else
                        sum_lb = sum_lb + prod_w * r_max;
                        sum_ub = sum_ub + prod_w * r_min;
                    end
                end
                J_lb(i,j) = sum_lb;
                J_ub(i,j) = sum_ub;
            end
        end
    
    case 20
        % === Copepod Neural ODE (32D, TANH bottleneck) ===
        % Dynamics: dx/dt = W2 * tanh(W1*x + b1) + b2
        % Jacobian: J(x) = W2 * diag(tanh'(W1*x + b1)) * W1
        % tanh'(z) = sech^2(z) = 1 - tanh^2(z)
        %   sech^2(z) is always in (0, 1]  (never zero, unlike ReLU)
        %   sech^2(0) = 1  (maximum, at z = 0)
        %   sech^2 decreases monotonically as |z| increases
        
        global W1_ode b1_ode W2_ode b2_ode
        
        if isempty(W1_ode)
            error('Jacob_bounds_element_wise: W1_ode not set for system 20.');
        end
        
        bottleneck = size(W1_ode, 1);  % 8
        state_dim = size(W1_ode, 2);   % 32
        
        % 1. Compute bounds for z = W1*x + b1
        z_lb = zeros(bottleneck, 1);
        z_ub = zeros(bottleneck, 1);
        for i = 1:bottleneck
            temp_lb = 0; temp_ub = 0;
            for j = 1:state_dim
                if W1_ode(i,j) >= 0
                    temp_lb = temp_lb + W1_ode(i,j) * a(j);
                    temp_ub = temp_ub + W1_ode(i,j) * b(j);
                else
                    temp_lb = temp_lb + W1_ode(i,j) * b(j);
                    temp_ub = temp_ub + W1_ode(i,j) * a(j);
                end
            end
            z_lb(i) = temp_lb + b1_ode(i);
            z_ub(i) = temp_ub + b1_ode(i);
        end
        
        % 2. Compute tanh derivative (sech^2) bounds over each bottleneck neuron
        % sech^2(z) = 1 - tanh^2(z)
        % sech^2 is maximized where |z| is smallest (closest to 0)
        % sech^2 is minimized where |z| is largest (farthest from 0)
        tanh_deriv_lb = zeros(bottleneck, 1);
        tanh_deriv_ub = zeros(bottleneck, 1);
        for i = 1:bottleneck
            % Find |z| closest to 0 in [z_lb, z_ub] -> gives max sech^2
            if z_lb(i) <= 0 && z_ub(i) >= 0
                % Interval contains 0 -> sech^2 max = sech^2(0) = 1
                z_closest = 0;
            elseif z_lb(i) > 0
                z_closest = z_lb(i);
            else  % z_ub < 0
                z_closest = z_ub(i);
            end
            
            % Find |z| farthest from 0 in [z_lb, z_ub] -> gives min sech^2
            z_farthest = max(abs(z_lb(i)), abs(z_ub(i)));
            
            % sech^2 bounds
            tanh_deriv_ub(i) = 1 - tanh(z_closest)^2;    % max sech^2
            tanh_deriv_lb(i) = 1 - tanh(z_farthest)^2;    % min sech^2
        end
        
        % 3. Compute Jacobian bounds: J = W2 * diag(sech^2) * W1
        J_lb = zeros(state_dim, state_dim);
        J_ub = zeros(state_dim, state_dim);
        for i = 1:state_dim
            for j = 1:state_dim
                sum_lb = 0; sum_ub = 0;
                for k = 1:bottleneck
                    prod_w = W2_ode(i,k) * W1_ode(k,j);
                    d_min = tanh_deriv_lb(k);
                    d_max = tanh_deriv_ub(k);
                    if prod_w >= 0
                        sum_lb = sum_lb + prod_w * d_min;
                        sum_ub = sum_ub + prod_w * d_max;
                    else
                        sum_lb = sum_lb + prod_w * d_max;
                        sum_ub = sum_ub + prod_w * d_min;
                    end
                end
                J_lb(i,j) = sum_lb;
                J_ub(i,j) = sum_ub;
            end
        end

    case 21
        % === Spiral_linear (2D - Linear Effective Dynamics) ===
        % dx = (w2*w1)*x + (w2*b1 + b2), so J = w2*w1 (constant)
        if n ~= 2
            error('Spiral_linear expects 2D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_spiral_linear(a, b);

    case 22
        % === Cartpole 12D with CT-RNN controller ===
        if n ~= 12
            error('Cartpole expects 12D state, but got %dD', n);
        end
        [J_lb, J_ub] = compute_jacobian_bounds_cartpole(a, b);  
        
    otherwise
        error('Invalid system_choice: %d', system_choice);
end

end

%% ========================================================================
%  SPIRAL_NON: Neural ODE Jacobian Bounds (2D -> 10 hidden -> 2D)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b)
    % Parameters from spiral_non.m
    w1 = [0.2911133 ,  0.12008807;
           -0.24582624,  0.23181419;
           -0.25797904,  0.21687193;
           -0.19282854, -0.2602416 ;
            0.26780415, -0.20697702;
            0.23462369,  0.2294843 ;
           -0.2583547 ,  0.21444395;
           -0.04514714,  0.29514763;
           -0.15318371, -0.275755  ;
           0.24873598,  0.21018365];
    b1 = [0.0038677 , -0.00026365, -0.007168970,  0.02469357,  0.01338706,...
            0.00856025, -0.00888401,  0.00516089, -0.00634514, -0.01914518]';
    
    w2 = [-0.58693904, -0.814841  , -0.8175157 ,  0.97060364,  0.6908913 ,...
            -0.92446184, -0.79249185, -1.1507587 ,  1.2072723 , -0.7983982 ;
           1.1564877 , -0.8991244 , -1.0774536 , -0.6731967 ,  1.0154784 ,...
             0.8984464 , -1.0766245 , -0.238209  , -0.5233613 ,  0.8886671];
    b2 = [-0.04129209, -0.01508532]';

    num_neurons = size(w1, 1); % 10

    % z = w1*x + b1
    z_lb = zeros(num_neurons, 1);
    z_ub = zeros(num_neurons, 1);
    for i = 1:num_neurons
        temp_lb = 0; temp_ub = 0;
        for j = 1:2
            if w1(i,j) >= 0
                temp_lb = temp_lb + w1(i,j) * a(j);
                temp_ub = temp_ub + w1(i,j) * b(j);
            else
                temp_lb = temp_lb + w1(i,j) * b(j);
                temp_ub = temp_ub + w1(i,j) * a(j);
            end
        end
        z_lb(i) = temp_lb + b1(i);
        z_ub(i) = temp_ub + b1(i);
    end

    % sech^2(z) interval
    sech2_int = zeros(num_neurons, 2);
    for j = 1:num_neurons
        sech2_int(j, :) = sech2_interval(z_lb(j), z_ub(j));
    end

    % J = w2 * diag(sech^2(z)) * w1
    J_lb = zeros(2,2); J_ub = zeros(2,2);
    for i = 1:2
        for j = 1:2
            sum_lb = 0; sum_ub = 0;
            for k = 1:num_neurons
                prod_w = w2(i,k) * w1(k,j);
                s_min = sech2_int(k,1); s_max = sech2_int(k,2);
                if prod_w >= 0
                    sum_lb = sum_lb + prod_w * s_min;
                    sum_ub = sum_ub + prod_w * s_max;
                else
                    sum_lb = sum_lb + prod_w * s_max;
                    sum_ub = sum_ub + prod_w * s_min;
                end
            end
            J_lb(i,j) = sum_lb;
            J_ub(i,j) = sum_ub;
        end
    end
end

%% ========================================================================
%  FPA: Structured Jacobian Bounds (5D, linear + sech^2)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b)
    tau = -1e-6;
    
    A = [-1.20327, -0.07202, -0.93635;
          1.18810, -1.50015,  0.93519];
    B = [ 1.21464, -0.10502;
          0.12023,  0.19387;
         -1.36695,  0.12201];
    BA = B * A;
    W = [zeros(2,2), A;
         zeros(3,2), BA];

    % Evaluate sech^2 interval (Derivative of tanh(x)) for each state
    S = zeros(5, 2);
    for j = 1:5
        S(j, :) = sech2_interval(a(j), b(j));
    end

    J_lb = zeros(5,5); J_ub = zeros(5,5);
    for i = 1:5
        for j = 1:5
            if W(i,j) > 0
                temp_lb = W(i,j) * S(j,1);
                temp_ub = W(i,j) * S(j,2);
            elseif W(i,j) < 0
                temp_lb = W(i,j) * S(j,2);
                temp_ub = W(i,j) * S(j,1);
            else
                temp_lb = 0; temp_ub = 0;
            end
            if i == j
                J_lb(i,j) = tau + temp_lb;
                J_ub(i,j) = tau + temp_ub;
            else
                J_lb(i,j) = temp_lb;
                J_ub(i,j) = temp_ub;
            end
        end
    end
end

%% ========================================================================
%  ACC: Neural ODE Jacobian Bounds (8D)                                    |->`b2_acc` was removed as we do not need it to compute the Jacobian because the derivative of a constant bias term is zero
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_acc(a, b)
    global w1_acc b1_acc w2_acc 
    
    % Lazy load if missing (same as System_description)
    if isempty(w1_acc)
        pl = load("plant_3rd_order_tanh.mat");
        w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
        b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
        w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
        w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
        b1_acc = b1_acc(:); % FORCE column vectors here too!
    end

    num_neurons = size(w1_acc, 1);
    
    % The NN only takes 4 specific inputs: x(3), x(6), x(7), x(8)
    nn_idx = [3, 6, 7, 8];
    a_sub = a(nn_idx);
    b_sub = b(nn_idx);

    % 1. Compute bounds for z = w1 * x_sub + b1
    z_lb = zeros(num_neurons, 1);
    z_ub = zeros(num_neurons, 1);
    for i = 1:num_neurons
        temp_lb = 0; temp_ub = 0;
        for j = 1:4
            if w1_acc(i,j) >= 0
                temp_lb = temp_lb + w1_acc(i,j) * a_sub(j);
                temp_ub = temp_ub + w1_acc(i,j) * b_sub(j);
            else
                temp_lb = temp_lb + w1_acc(i,j) * b_sub(j);
                temp_ub = temp_ub + w1_acc(i,j) * a_sub(j);
            end
        end
        z_lb(i) = temp_lb + b1_acc(i);
        z_ub(i) = temp_ub + b1_acc(i);
    end

    % 2. Compute interval bounds for sech^2(z)
    sech2_int = zeros(num_neurons, 2);
    for j = 1:num_neurons
        sech2_int(j, :) = sech2_interval(z_lb(j), z_ub(j));
    end

    % 3. Compute Jacobian bounds for the NN part: J_nn = w2 * diag(sech^2) * w1
    % Output of NN affects dx(3) and dx(6). We only care about those 2 rows.
    % Wait, w2 is 4x20. 
    % The outputs map to dx3, dx6, dx7, dx8. 
    % But wait! In our System_description, we forced dx7 and dx8 to 0. 
    % Therefore, we only care about row 1 of w2 (maps to dx3) and row 2 of w2 (maps to dx6).
    
    J_nn_lb = zeros(4, 4); 
    J_nn_ub = zeros(4, 4);
    
    for i = 1:4 % Corresponds to the 4 outputs of the NN
        for j = 1:4 % Corresponds to the 4 inputs of the NN
            sum_lb = 0; sum_ub = 0;
            for k = 1:num_neurons
                prod_w = w2_acc(i,k) * w1_acc(k,j);
                s_min = sech2_int(k,1); s_max = sech2_int(k,2);
                if prod_w >= 0
                    sum_lb = sum_lb + prod_w * s_min;
                    sum_ub = sum_ub + prod_w * s_max;
                else
                    sum_lb = sum_lb + prod_w * s_max;
                    sum_ub = sum_ub + prod_w * s_min;
                end
            end
            J_nn_lb(i,j) = sum_lb;
            J_nn_ub(i,j) = sum_ub;
        end
    end

    % 4. Assemble the full 8x8 Jacobian matrix
    J_lb = zeros(8, 8);
    J_ub = zeros(8, 8);
    
    % Pure kinematic linear parts (Constant 1s)
    % x1_dot = x2 -> J(1,2) = 1
    J_lb(1,2) = 1; J_ub(1,2) = 1;
    % x2_dot = x3 -> J(2,3) = 1
    J_lb(2,3) = 1; J_ub(2,3) = 1;
    % x4_dot = x5 -> J(4,5) = 1
    J_lb(4,5) = 1; J_ub(4,5) = 1;
    % x5_dot = x6 -> J(5,6) = 1
    J_lb(5,6) = 1; J_ub(5,6) = 1;
    
    % Map the NN Jacobian back to the full 8x8 matrix
    % NN Outputs: [dx3, dx6, dx7, dx8] (But dx7 and dx8 are overridden to 0)
    % NN Inputs:  [x3, x6, x7, x8]
    
    % Row 3 (dx3) mappings (Lead Car Jerk):
    J_lb(3, nn_idx) = J_nn_lb(1, :);
    J_ub(3, nn_idx) = J_nn_ub(1, :);
    
    % Row 6 (dx6) mappings (Ego Car Jerk):
    J_lb(6, nn_idx) = J_nn_lb(2, :);
    J_ub(6, nn_idx) = J_nn_ub(2, :);

    % Row 7 (dx7) mappings (Ego Accel Derivative):
    J_lb(7, nn_idx) = J_nn_lb(3, :);
    J_ub(7, nn_idx) = J_nn_ub(3, :);

    % Row 8 (dx8) mappings (Lead Accel Derivative):
    J_lb(8, nn_idx) = J_nn_lb(4, :);
    J_ub(8, nn_idx) = J_nn_ub(4, :);

end

%% ========================================================================
%  LINEAR ACC NODE: constant Jacobian (8D)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_acc_linear(a, b) %#ok<INUSD>
    % dx_nn = W2 * W1 * x_sub + (W2*b1 + b2), linear in x_sub.
    % Jacobian is constant: d(dx_nn)/d(x_sub) = W2 * W1  (4x4).
    global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin

    if isempty(w1_acc_lin)
        pl = load("plant_3rd_order_node.mat");
        w1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
        b1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
        w2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
        b2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
        b1_acc_lin = b1_acc_lin(:);
        b2_acc_lin = b2_acc_lin(:);
    end

    J_nn = w2_acc_lin * w1_acc_lin;      % 4x4 constant

    J_lb = zeros(8,8);  J_ub = zeros(8,8);

    % Kinematic parts: constant 1s
    J_lb(1,2) = 1; J_ub(1,2) = 1;
    J_lb(2,3) = 1; J_ub(2,3) = 1;
    J_lb(4,5) = 1; J_ub(4,5) = 1;
    J_lb(5,6) = 1; J_ub(5,6) = 1;

    % Map NN Jacobian into rows 3, 6, 7, 8 and columns 3, 6, 7, 8
    nn_idx = [3, 6, 7, 8];
    for i = 1:4
        for j = 1:4
            J_lb(nn_idx(i), nn_idx(j)) = J_nn(i, j);
            J_ub(nn_idx(i), nn_idx(j)) = J_nn(i, j);
        end
    end
end

%% ========================================================================
%  SINGLE PENDULUM: Neural ODE Jacobian Bounds (3D, 1 hidden layer)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_pendulum(a, b)
    % 1-hidden-layer architecture (2-layer variant no longer used).
    global w1_sp b1_sp w2_sp b2_sp 
    
    if isempty(w1_sp)
        pl = load('odenet_pendulum_surrogate.mat');
        w1_sp = double(pl.Wb{1}); b1_sp = double(pl.Wb{2}(:));
        w2_sp = double(pl.Wb{3}); b2_sp = double(pl.Wb{4}(:));
    end

    n_neurons = size(w1_sp, 1); % Will be 20
    
    % Dynamically read the input/output dimensions
    n_in = size(w1_sp, 2);  % Will be 3
    n_out = size(w2_sp, 1); % Will be 2 (because of removeconstantrows)

    % 1. Bounds for z1 = w1*x + b1
    z1_lb = zeros(n_neurons, 1); 
    z1_ub = zeros(n_neurons, 1);
    for i = 1:n_neurons
        t_lb = 0; t_ub = 0;
        for j = 1:n_in
            if w1_sp(i,j) >= 0
                t_lb = t_lb + w1_sp(i,j)*a(j); 
                t_ub = t_ub + w1_sp(i,j)*b(j);
            else
                t_lb = t_lb + w1_sp(i,j)*b(j); 
                t_ub = t_ub + w1_sp(i,j)*a(j);
            end
        end
        z1_lb(i) = t_lb + b1_sp(i); 
        z1_ub(i) = t_ub + b1_sp(i);
    end

    % 2. Compute interval bounds for sech^2(z1)
    sech2_int = zeros(n_neurons, 2);
    for j = 1:n_neurons
        sech2_int(j, :) = sech2_interval(z1_lb(j), z1_ub(j));
    end

    % 3. Compute Jacobian bounds for the NN part: J_nn = w2 * diag(sech^2) * w1
    J_nn_lb = zeros(n_out, n_in); 
    J_nn_ub = zeros(n_out, n_in);
    
    for i = 1:n_out 
        for j = 1:n_in 
            sum_lb = 0; sum_ub = 0;
            for k = 1:n_neurons
                prod_w = w2_sp(i,k) * w1_sp(k,j);
                s_min = sech2_int(k,1); 
                s_max = sech2_int(k,2);
                if prod_w >= 0
                    sum_lb = sum_lb + prod_w * s_min;
                    sum_ub = sum_ub + prod_w * s_max;
                else
                    sum_lb = sum_lb + prod_w * s_max;
                    sum_ub = sum_ub + prod_w * s_min;
                end
            end
            J_nn_lb(i,j) = sum_lb;
            J_nn_ub(i,j) = sum_ub;
        end
    end

    % 4. Reconstruct the full Square Jacobian
    J_lb = zeros(n_in, n_in);
    J_ub = zeros(n_in, n_in);
    
    % Pad the neural network Jacobian with zeros for the constant time state
    J_lb(1:n_out, 1:n_in) = J_nn_lb;
    J_ub(1:n_out, 1:n_in) = J_nn_ub;
end

%% ========================================================================
%  SPIRAL_LINEAR: Constant Jacobian (no activation, pure linear)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_spiral_linear(a, b) %#ok<INUSD>
    % Effective dynamics: dx = (w2*w1)*x + (w2*b1 + b2)
    % Jacobian is CONSTANT, independent of the state interval.
    w1 = [-0.32294768,  0.59955627;
           0.47014388, -0.39748120;
          -0.56326932,  0.33752987;
           0.45147443,  0.31528524;
           0.41403031, -0.47271276;
          -0.12952870, -0.62095606;
          -0.41343114, -0.45678866;
          -0.33266136,  0.29245856;
           0.50114638,  0.39612201;
           0.47665390,  0.55137879];
    w2 = [-0.50525320,  0.34800902, -0.34015974, -0.40054744,  0.39193857, ...
           0.59363592,  0.56743664, -0.33811751, -0.36945280, -0.46805024;
          -0.41715327,  0.56257814, -0.56921810,  0.60423535,  0.53992182, ...
          -0.14412111, -0.45906776, -0.35295558,  0.49238238,  0.43526673];
    J_const = w2 * w1;  % 2x2
    J_lb = J_const;
    J_ub = J_const;
end

%% ========================================================================
%  CARTPOLE: Neural ODE Jacobian Bounds (12D: 4 physical + 8 RNN hidden)
% =========================================================================
% Important caveat on Cartpole Jacobian: the physics terms xacc and thetaacc involve division, 
% sin/cos, and products — deriving tight element-wise interval bounds on their partials is non-trivial. 
% The Lipschitz-style enclosures I used above are sound but loose; they'll work correctly with TNODEV 
% (the refinement loop will compensate), but you may see more refinement iterations than for FPA/Spiral. 
% If you find convergence too slow, consider computing xacc and thetaacc partials symbolically in MATLAB
% once (using syms and jacobian) and then evaluating them with interval arithmetic — that will give tighter bounds.

function [J_lb, J_ub] = compute_jacobian_bounds_cartpole(a, b)
    % State order: [x_00 (cart pos), x_01 (cart vel), x_02 (pole angle),
    %               x_03 (pole rate), h_00...h_07]
    %
    % Dynamics (from CartpoleCTRNN):
    %   x_00_dot = x_01
    %   x_01_dot = xacc(x_02, x_03, action(h))
    %   x_02_dot = x_03
    %   x_03_dot = thetaacc(x_02, x_03, action(h))
    %   h_i_dot  = -h_i + tanh(W_h(i,:) * h + W_x(i,:) * x_phys + bias(i))
    %
    % We compute interval bounds using element-wise interval arithmetic
    % over the pre-activation and the post-activation derivatives.

    % -------- CT-RNN weights (8 hidden units, 12 inputs: 8 h + 4 x_phys) --------
    % Each row: [h0..h7, x0..x3, bias]
    W_rnn = [
        -1.49394, -0.61947,  0.37393, -0.63451, -1.08420,  2.57981, -1.53850, -1.64354, -0.07445,  0.08736,  0.47684,  0.56397,  1.81867;
         1.38295, -1.45811,  1.01473, -0.04578, -0.13416, -0.21970,  0.41791, -0.10833, -1.70409,  0.51560, -0.71273, -0.61720,  0.86009;
         0.57055,  0.05941, -0.16993, -0.69688, -0.30939, -1.31558,  0.03316,  2.35873,  0.21849,  1.42990,  1.60666, -0.66847, -1.56112;
        -1.53087, -0.42779, -0.02195,  0.18007,  1.54262, -0.19275, -0.64598, -0.85840,  0.10095,  0.55012,  1.51416,  1.95952,  0.00020;
        -0.54493,  0.41139, -0.18790, -0.14312, -0.10144,  2.38792, -0.44134, -2.06462, -0.01260,  0.84681,  0.51889,  0.35089, -0.06552;
         1.08793, -0.24559, -0.97041, -0.16874, -0.27133,  0.73513,  0.73096, -2.26244,  0.80312, -1.35635, -0.95283, -0.68460,  0.11646;
         0.08745,  0.10104,  0.82251, -0.59823,  1.16949,  1.73511, -0.89387,  0.77149,  0.22731, -0.73872,  2.83509, -2.49535,  0.70343;
        -0.89045, -1.30985,  0.48740, -0.45750, -0.70727, -0.43216, -0.29159,  3.70331, -0.13294, -0.23242,  0.33244, -1.45838, -0.02392];
    W_hh = W_rnn(:, 1:8);    % hidden->hidden 8x8
    W_xh = W_rnn(:, 9:12);   % physical->hidden 8x4
    b_h  = W_rnn(:, 13);     % 8x1

    % Readout weights for the action (tanh) from the 8 hidden units
    W_a = [-0.32011,  0.32958,  0.07718,  2.39431, -1.37732,  0.92722, -1.07137, -1.47286];

    % Physics constants
    gravity = 9.8;
    masscart = 1;
    masspole = 0.1;
    total_mass = masscart + masspole;
    length_pole = 0.5;
    polemass_length = masspole * length_pole;
    force_mag = 10;

    % -------- Extract state intervals --------
    xp_lb = a(1:4);   xp_ub = b(1:4);     % physical states
    h_lb  = a(5:12);  h_ub  = b(5:12);    % RNN hidden states

    % =======================================================================
    % STEP A: Pre-activation z_i = W_hh(i,:) * h + W_xh(i,:) * x_phys + b_h(i)
    % =======================================================================
    z_lb = zeros(8,1); z_ub = zeros(8,1);
    for i = 1:8
        t_lb = 0; t_ub = 0;
        % hidden contribution
        for k = 1:8
            if W_hh(i,k) >= 0
                t_lb = t_lb + W_hh(i,k)*h_lb(k);
                t_ub = t_ub + W_hh(i,k)*h_ub(k);
            else
                t_lb = t_lb + W_hh(i,k)*h_ub(k);
                t_ub = t_ub + W_hh(i,k)*h_lb(k);
            end
        end
        % physical contribution
        for k = 1:4
            if W_xh(i,k) >= 0
                t_lb = t_lb + W_xh(i,k)*xp_lb(k);
                t_ub = t_ub + W_xh(i,k)*xp_ub(k);
            else
                t_lb = t_lb + W_xh(i,k)*xp_ub(k);
                t_ub = t_ub + W_xh(i,k)*xp_lb(k);
            end
        end
        z_lb(i) = t_lb + b_h(i);
        z_ub(i) = t_ub + b_h(i);
    end

    % sech^2(z_i) intervals (derivative of tanh)
    S = zeros(8,2);
    for i = 1:8
        S(i,:) = sech2_interval(z_lb(i), z_ub(i));
    end

    % =======================================================================
    % STEP B: Action = tanh(W_a * h), derivative bounds w.r.t. h
    % =======================================================================
    % Pre-activation za = W_a * h
    za_lb = 0; za_ub = 0;
    for k = 1:8
        if W_a(k) >= 0
            za_lb = za_lb + W_a(k)*h_lb(k);
            za_ub = za_ub + W_a(k)*h_ub(k);
        else
            za_lb = za_lb + W_a(k)*h_ub(k);
            za_ub = za_ub + W_a(k)*h_lb(k);
        end
    end
    Sa = sech2_interval(za_lb, za_ub);    % [s_min, s_max]  >=0, <=1
    % Action bounds (tanh is monotone)
    action_lb = tanh(za_lb);
    action_ub = tanh(za_ub);

    % =======================================================================
    % STEP C: Build 12x12 Jacobian
    %   Rows 1..4  -> physical states
    %   Rows 5..12 -> hidden states
    % =======================================================================
    J_lb = zeros(12,12);
    J_ub = zeros(12,12);

    % ----- Row 1: x_00_dot = x_01 -----
    J_lb(1,2) = 1; J_ub(1,2) = 1;

    % ----- Row 3: x_02_dot = x_03 -----
    J_lb(3,4) = 1; J_ub(3,4) = 1;

    % ----- Rows 2 and 4: x_01_dot = xacc, x_03_dot = thetaacc -----
    % These depend on x_02, x_03, and (through 'force') on action(h),
    % hence on h_00..h_07. The dependency is nonlinear (sin/cos, division).
    % We compute interval bounds via interval evaluation of the partial derivatives.
    theta_lb = xp_lb(3); theta_ub = xp_ub(3);
    thd_lb   = xp_lb(4); thd_ub   = xp_ub(4);

    % cos(theta) interval
    [cos_lb, cos_ub] = cos_interval(theta_lb, theta_ub);
    % sin(theta) interval
    [sin_lb, sin_ub] = sin_interval(theta_lb, theta_ub);
    % theta_dot^2 interval (always >= 0)
    thd2_lb = 0;
    if thd_lb >= 0
        thd2_lb = thd_lb^2;
    elseif thd_ub <= 0
        thd2_lb = thd_ub^2;
    end
    thd2_ub = max(thd_lb^2, thd_ub^2);

    % force interval = force_mag * action
    force_lb = force_mag * action_lb;
    force_ub = force_mag * action_ub;

    % temp = (force + polemass_length * theta_dot^2 * sintheta) / total_mass
    % Interval version:
    pml_t2_sin_lb = interval_mul_scalar_interval_interval(polemass_length, ...
                        [thd2_lb thd2_ub], [sin_lb sin_ub]);
    pml_t2_sin_ub = pml_t2_sin_lb(2);
    pml_t2_sin_lb = pml_t2_sin_lb(1);
    temp_lb = (force_lb + pml_t2_sin_lb) / total_mass;
    temp_ub = (force_ub + pml_t2_sin_ub) / total_mass;

    % denom = length * (4/3 - masspole*cos^2/total_mass)
    cos2 = interval_mul_scalar_interval_interval(1, [cos_lb cos_ub], [cos_lb cos_ub]);
    cos2_lb = cos2(1); cos2_ub = cos2(2);
    denom_lb = length_pole * (4/3 - masspole * cos2_ub / total_mass);
    denom_ub = length_pole * (4/3 - masspole * cos2_lb / total_mass);
    % Physical regime guarantees denom > 0.

    % NOTE: We bound d(xacc)/d{x_02,x_03,h_k} and d(thetaacc)/d{x_02,x_03,h_k}
    % CONSERVATIVELY by a Lipschitz-style worst case. For TNODEV's refinement
    % loop to remain sound, these interval bounds only need to contain the true
    % Jacobian; looseness will trigger more refinement, which is acceptable.
    %
    % We approximate by a large enclosing interval derived from the magnitudes
    % of the physics terms evaluated over the current state box:
    M_sin = max(abs(sin_lb), abs(sin_ub));
    M_cos = max(abs(cos_lb), abs(cos_ub));
    M_thd = max(abs(thd_lb), abs(thd_ub));
    M_temp = max(abs(temp_lb), abs(temp_ub));
    M_denom_inv = 1/min(abs(denom_lb), abs(denom_ub));

    % Row 2 (x_01_dot = xacc):
    %   depends on x_02, x_03, and h (via action)
    % Row 4 (x_03_dot = thetaacc): same dependency structure
    % Worst-case Lipschitz bounds:
    L_xacc_theta   = gravity*M_cos*M_denom_inv + M_temp*M_cos + 2*polemass_length*M_thd*M_sin*M_denom_inv;
    L_xacc_thd     = 2*polemass_length*M_thd*M_sin/total_mass;
    L_thacc_theta  = gravity*M_cos*M_denom_inv + M_sin*M_denom_inv + M_denom_inv*M_temp;
    L_thacc_thd    = 2*polemass_length*M_thd*M_sin*M_denom_inv/total_mass;
    L_force_h      = force_mag * max(abs(Sa));   % |d(action)/d h_k|  <= max sech^2

    % Row 2: d(x_01_dot)/d(x_02), d(x_03), d(h_k)
    J_lb(2,3) = -L_xacc_theta; J_ub(2,3) =  L_xacc_theta;
    J_lb(2,4) = -L_xacc_thd;   J_ub(2,4) =  L_xacc_thd;
    for k = 1:8
        % action->xacc via temp: d(xacc)/d h_k bounded by M_denom_inv * L_force_h * |W_a(k)|
        bound_h = M_denom_inv * L_force_h * abs(W_a(k)) / total_mass;
        J_lb(2,4+k) = -bound_h; J_ub(2,4+k) = bound_h;
    end

    % Row 4: d(x_03_dot)/d(x_02), d(x_03), d(h_k)
    J_lb(4,3) = -L_thacc_theta; J_ub(4,3) = L_thacc_theta;
    J_lb(4,4) = -L_thacc_thd;   J_ub(4,4) = L_thacc_thd;
    for k = 1:8
        bound_h = M_denom_inv * L_force_h * abs(W_a(k));
        J_lb(4,4+k) = -bound_h; J_ub(4,4+k) = bound_h;
    end

    % ----- Rows 5..12: hidden state derivatives -----
    % h_i_dot = -h_i + tanh(z_i(h,x))
    % d(h_i_dot)/d h_j = -delta_{ij} + sech^2(z_i) * W_hh(i,j)
    % d(h_i_dot)/d x_k =                sech^2(z_i) * W_xh(i,k)
    for i = 1:8
        s_min = S(i,1); s_max = S(i,2);  % sech^2 bounds for neuron i

        % Dependence on physical states (columns 1..4)
        for k = 1:4
            w = W_xh(i,k);
            if w >= 0
                J_lb(4+i, k) = w * s_min;
                J_ub(4+i, k) = w * s_max;
            else
                J_lb(4+i, k) = w * s_max;
                J_ub(4+i, k) = w * s_min;
            end
        end

        % Dependence on hidden states (columns 5..12)
        for j = 1:8
            w = W_hh(i,j);
            if w >= 0
                lb_term = w * s_min;
                ub_term = w * s_max;
            else
                lb_term = w * s_max;
                ub_term = w * s_min;
            end
            if i == j
                % diagonal: -1 from the leak term
                J_lb(4+i, 4+j) = -1 + lb_term;
                J_ub(4+i, 4+j) = -1 + ub_term;
            else
                J_lb(4+i, 4+j) = lb_term;
                J_ub(4+i, 4+j) = ub_term;
            end
        end
    end
end

%% ========================================================================
%  HELPER: cos(x) over [a,b]
% =========================================================================
function [c_lb, c_ub] = cos_interval(a, b)
    % Sample endpoints + check for interior extrema (cos has max at 2*pi*k,
    % min at pi + 2*pi*k).
    vals = [cos(a), cos(b)];
    % Find any k such that 2*pi*k or pi + 2*pi*k is inside [a,b]
    k_max_lo = ceil((a - 0) / (2*pi));
    k_max_hi = floor((b - 0) / (2*pi));
    if k_max_lo <= k_max_hi
        vals(end+1) = 1;     % cos attains 1
    end
    k_min_lo = ceil((a - pi) / (2*pi));
    k_min_hi = floor((b - pi) / (2*pi));
    if k_min_lo <= k_min_hi
        vals(end+1) = -1;    % cos attains -1
    end
    c_lb = min(vals);
    c_ub = max(vals);
end

%% ========================================================================
%  HELPER: sin(x) over [a,b]
% =========================================================================
function [s_lb, s_ub] = sin_interval(a, b)
    vals = [sin(a), sin(b)];
    k_max_lo = ceil((a - pi/2) / (2*pi));
    k_max_hi = floor((b - pi/2) / (2*pi));
    if k_max_lo <= k_max_hi
        vals(end+1) = 1;
    end
    k_min_lo = ceil((a + pi/2) / (2*pi));
    k_min_hi = floor((b + pi/2) / (2*pi));
    if k_min_lo <= k_min_hi
        vals(end+1) = -1;
    end
    s_lb = min(vals);
    s_ub = max(vals);
end

%% ========================================================================
%  HELPER: Interval multiplication  c * [a1,a2] * [b1,b2]
% =========================================================================
function out = interval_mul_scalar_interval_interval(c, I1, I2)
    % out = c * I1 * I2, returned as [lb, ub]
    prods = c * [I1(1)*I2(1), I1(1)*I2(2), I1(2)*I2(1), I1(2)*I2(2)];
    out = [min(prods), max(prods)];
end

%% ========================================================================
%  HELPER: sech^2(x) over [a,b]
% =========================================================================
function S = sech2_interval(a, b)
    if a > b, error('a must <= b'); end
    
    val_a = 1 / cosh(a)^2;
    val_b = 1 / cosh(b)^2;
    
    if a <= 0 && b >= 0
        s_min = min(val_a, val_b);
        s_max = 1;
    elseif b < 0
        s_min = val_a;  % increasing
        s_max = val_b;
    elseif a > 0
        s_min = val_b;  % decreasing
        s_max = val_a;
    else
        s_min = min(val_a, val_b);
        s_max = max(val_a, val_b);
    end
    S = [s_min, s_max];
end
%------------- END OF CODE --------------
