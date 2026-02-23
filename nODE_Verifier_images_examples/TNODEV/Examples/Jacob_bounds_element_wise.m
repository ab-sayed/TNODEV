% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 20 February 2026
% Last revision: November 2025

% %------------- BEGIN CODE --------------
% % function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b)
% function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice)
% % Jacob_bounds_element_wise.m: computes Jacobian bounds using element-wise
% % interval arithmetic for these examples : FPA (14) or Spiral_non (15) 
% % FPA case:
%     % Inputs:
%     %   a - 5x1 vector of lower bounds for x
%     %   b - 5x1 vector of upper bounds for x
%     % Outputs:
%     %   J_lb - 5x5 matrix of lower bounds for the Jacobian
%     %   J_ub - 5x5 matrix of upper bounds for the Jacobian
% % Spiral case:
%     % Inputs:
%     %   a - 2x1 vector of lower bounds for x
%     %   b - 2x1 vector of upper bounds for x
%     % Outputs:
%     %   J_lb - 2x2 matrix of lower bounds for the Jacobian
%     %   J_ub - 2x2 matrix of upper bounds for the Jacobian
% 
% % Debug Prints    
% % fprintf('Computing Jacobian bounds based on reachable tube bounds:\n');
% % fprintf('a = [%s]\n', num2str(a', '%.6f '));
% % fprintf('b = [%s]\n', num2str(b', '%.6f '));
% % fprintf('System choice: %d\n', system_choice);
% 
% % Validate inputs
% if ~isequal(size(a), size(b)) || any(a > b)
%     error('a and b must be same size and a <= b element-wise');
% end
% 
% n = length(a);
% 
% if system_choice == 15
%     % === SPIRAL_NON (2D) ===
%     if n ~= 2
%         error('Spiral_non expects 2D state, but got %dD', n);
%     end
%     [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b);
% 
% elseif system_choice == 14
%     % === FPA (5D) ===
%     if n ~= 5
%         error('FPA expects 5D state, but got %dD', n);
%     end
%     [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b);
% 
% else
%     error('Invalid system_choice: %d. Use 14 (FPA) or 15 (Spiral_non)', system_choice);
% end
% 
% end
% 
% 
% %% ========================================================================
% %  SPIRAL_NON: Neural ODE Jacobian Bounds (2D → 10 hidden → 2D)
% % =========================================================================
% function [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b)
%     % Parameters from spiral_non.m
%     w1 = [0.2911133 ,  0.12008807;
%            -0.24582624,  0.23181419;
%            -0.25797904,  0.21687193;
%            -0.19282854, -0.2602416 ;
%             0.26780415, -0.20697702;
%             0.23462369,  0.2294843 ;
%            -0.2583547 ,  0.21444395;
%            -0.04514714,  0.29514763;
%            -0.15318371, -0.275755  ;
%            0.24873598,  0.21018365];
%     b1 = [0.0038677 , -0.00026365, -0.007168970,  0.02469357,  0.01338706,...
%             0.00856025, -0.00888401,  0.00516089, -0.00634514, -0.01914518]';
% 
%     w2 = [-0.58693904, -0.814841  , -0.8175157 ,  0.97060364,  0.6908913 ,...
%             -0.92446184, -0.79249185, -1.1507587 ,  1.2072723 , -0.7983982 ;
%            1.1564877 , -0.8991244 , -1.0774536 , -0.6731967 ,  1.0154784 ,...
%              0.8984464 , -1.0766245 , -0.238209  , -0.5233613 ,  0.8886671];
%     b2 = [-0.04129209, -0.01508532]';
% 
%     num_neurons = size(w1, 1); % 10
% 
%     % z = w1*x + b1
%     z_lb = zeros(num_neurons, 1);
%     z_ub = zeros(num_neurons, 1);
%     for i = 1:num_neurons
%         temp_lb = 0; temp_ub = 0;
%         for j = 1:2
%             if w1(i,j) >= 0
%                 temp_lb = temp_lb + w1(i,j) * a(j);
%                 temp_ub = temp_ub + w1(i,j) * b(j);
%             else
%                 temp_lb = temp_lb + w1(i,j) * b(j);
%                 temp_ub = temp_ub + w1(i,j) * a(j);
%             end
%         end
%         z_lb(i) = temp_lb + b1(i);
%         z_ub(i) = temp_ub + b1(i);
%     end
% 
%     % sech²(z) interval
%     sech2_int = zeros(num_neurons, 2);
%     for j = 1:num_neurons
%         sech2_int(j, :) = sech2_interval(z_lb(j), z_ub(j));
%     end
% 
%     % J = w2 * diag(sech²(z)) * w1
%     J_lb = zeros(2,2); J_ub = zeros(2,2);
%     for i = 1:2
%         for j = 1:2
%             sum_lb = 0; sum_ub = 0;
%             for k = 1:num_neurons
%                 prod_w = w2(i,k) * w1(k,j);
%                 s_min = sech2_int(k,1); s_max = sech2_int(k,2);
%                 if prod_w >= 0
%                     sum_lb = sum_lb + prod_w * s_min;
%                     sum_ub = sum_ub + prod_w * s_max;
%                 else
%                     sum_lb = sum_lb + prod_w * s_max;
%                     sum_ub = sum_ub + prod_w * s_min;
%                 end
%             end
%             J_lb(i,j) = sum_lb;
%             J_ub(i,j) = sum_ub;
%         end
%     end
% end
% 
% %% ========================================================================
% %  FPA: Structured Jacobian Bounds (5D, linear + sech²)
% % =========================================================================
% function [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b)
%     tau = -1e-6;
% 
%     A = [-1.20327, -0.07202, -0.93635;
%           1.18810, -1.50015,  0.93519];
%     B = [ 1.21464, -0.10502;
%           0.12023,  0.19387;
%          -1.36695,  0.12201];
%     BA = B * A;
%     W = [zeros(2,2), A;
%          zeros(3,2), BA];
% 
%     % sech² for each state
%     S = zeros(5, 2);
%     for j = 1:5
%         S(j, :) = sech2_interval(a(j), b(j));
%     end
% 
%     J_lb = zeros(5,5); J_ub = zeros(5,5);
%     for i = 1:5
%         for j = 1:5
%             if W(i,j) > 0
%                 temp_lb = W(i,j) * S(j,1);
%                 temp_ub = W(i,j) * S(j,2);
%             elseif W(i,j) < 0
%                 temp_lb = W(i,j) * S(j,2);
%                 temp_ub = W(i,j) * S(j,1);
%             else
%                 temp_lb = 0; temp_ub = 0;
%             end
%             if i == j
%                 J_lb(i,j) = tau + temp_lb;
%                 J_ub(i,j) = tau + temp_ub;
%             else
%                 J_lb(i,j) = temp_lb;
%                 J_ub(i,j) = temp_ub;
%             end
%         end
%     end
% end
% 
% 
% %% ========================================================================
% %  HELPER: sech²(x) over [a,b]
% % =========================================================================
% function S = sech2_interval(a, b)
%     if a > b, error('a must <= b'); end
% 
%     val_a = 1 / cosh(a)^2;
%     val_b = 1 / cosh(b)^2;
% 
%     if a <= 0 && b >= 0
%         s_min = min(val_a, val_b);
%         s_max = 1;
%     elseif b < 0
%         s_min = val_a;  % increasing
%         s_max = val_b;
%     elseif a > 0
%         s_min = val_b;  % decreasing
%         s_max = val_a;
%     else
%         s_min = min(val_a, val_b);
%         s_max = max(val_a, val_b);
%     end
%     S = [s_min, s_max];
% end
% 
% %------------- END OF CODE --------------

%%

%------------- BEGIN CODE V 1.4--------------

% function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice)
% % Jacob_bounds_element_wise.m: computes Jacobian bounds using element-wise
% % interval arithmetic.
% %
% % Inputs:
% %   a - vector of lower bounds for x
% %   b - vector of upper bounds for x
% %   system_choice - ID of the system (14=FPA, 15=Spiral, 16=MNIST)
% %
% % Outputs:
% %   J_lb - matrix of lower bounds for the Jacobian
% %   J_ub - matrix of upper bounds for the Jacobian
% 
% % Validate inputs
% if ~isequal(size(a), size(b)) || any(a > b)
%     error('a and b must be same size and a <= b element-wise');
% end
% 
% n = length(a);
% 
% switch system_choice
%     case 15
%         % === SPIRAL_NON (2D) ===
%         if n ~= 2
%             error('Spiral_non expects 2D state, but got %dD', n);
%         end
%         [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b);
% 
%     case 14
%         % === FPA (5D) ===
%         if n ~= 5
%             error('FPA expects 5D state, but got %dD', n);
%         end
%         [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b);
% 
%     case 16
%         % === MNIST Neural ODE (676D - Linear) ===
%         % Dynamics: dx/dt = Aout * x + Bout
%         % Jacobian: J(x) = Aout (Constant)
% 
%         % Linear System: J = Aout_global
%         global Aout_global
% 
%         if isempty(Aout_global)
%             error('Jacob_bounds_element_wise: Aout_global is empty. Ensure reach_cnn_tiny_NODE_RA sets it.');
%         end
% 
%         % For a linear system, Jacobian is constant and independent of state x.
%         J_lb = Aout_global;
%         J_ub = Aout_global;
% 
%     otherwise
%         error('Invalid system_choice: %d', system_choice);
% end
% 
% end
% 
% %% ========================================================================
% %  SPIRAL_NON: Neural ODE Jacobian Bounds (2D -> 10 hidden -> 2D)
% % =========================================================================
% function [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b)
%     % Parameters from spiral_non.m
%     w1 = [0.2911133 ,  0.12008807;
%            -0.24582624,  0.23181419;
%            -0.25797904,  0.21687193;
%            -0.19282854, -0.2602416 ;
%             0.26780415, -0.20697702;
%             0.23462369,  0.2294843 ;
%            -0.2583547 ,  0.21444395;
%            -0.04514714,  0.29514763;
%            -0.15318371, -0.275755  ;
%            0.24873598,  0.21018365];
%     b1 = [0.0038677 , -0.00026365, -0.007168970,  0.02469357,  0.01338706,...
%             0.00856025, -0.00888401,  0.00516089, -0.00634514, -0.01914518]';
% 
%     w2 = [-0.58693904, -0.814841  , -0.8175157 ,  0.97060364,  0.6908913 ,...
%             -0.92446184, -0.79249185, -1.1507587 ,  1.2072723 , -0.7983982 ;
%            1.1564877 , -0.8991244 , -1.0774536 , -0.6731967 ,  1.0154784 ,...
%              0.8984464 , -1.0766245 , -0.238209  , -0.5233613 ,  0.8886671];
%     b2 = [-0.04129209, -0.01508532]';
% 
%     num_neurons = size(w1, 1); % 10
% 
%     % z = w1*x + b1
%     z_lb = zeros(num_neurons, 1);
%     z_ub = zeros(num_neurons, 1);
%     for i = 1:num_neurons
%         temp_lb = 0; temp_ub = 0;
%         for j = 1:2
%             if w1(i,j) >= 0
%                 temp_lb = temp_lb + w1(i,j) * a(j);
%                 temp_ub = temp_ub + w1(i,j) * b(j);
%             else
%                 temp_lb = temp_lb + w1(i,j) * b(j);
%                 temp_ub = temp_ub + w1(i,j) * a(j);
%             end
%         end
%         z_lb(i) = temp_lb + b1(i);
%         z_ub(i) = temp_ub + b1(i);
%     end
% 
%     % sech^2(z) interval
%     sech2_int = zeros(num_neurons, 2);
%     for j = 1:num_neurons
%         sech2_int(j, :) = sech2_interval(z_lb(j), z_ub(j));
%     end
% 
%     % J = w2 * diag(sech^2(z)) * w1
%     J_lb = zeros(2,2); J_ub = zeros(2,2);
%     for i = 1:2
%         for j = 1:2
%             sum_lb = 0; sum_ub = 0;
%             for k = 1:num_neurons
%                 prod_w = w2(i,k) * w1(k,j);
%                 s_min = sech2_int(k,1); s_max = sech2_int(k,2);
%                 if prod_w >= 0
%                     sum_lb = sum_lb + prod_w * s_min;
%                     sum_ub = sum_ub + prod_w * s_max;
%                 else
%                     sum_lb = sum_lb + prod_w * s_max;
%                     sum_ub = sum_ub + prod_w * s_min;
%                 end
%             end
%             J_lb(i,j) = sum_lb;
%             J_ub(i,j) = sum_ub;
%         end
%     end
% end
% 
% %% ========================================================================
% %  FPA: Structured Jacobian Bounds (5D, linear + sech^2)
% % =========================================================================
% function [J_lb, J_ub] = compute_jacobian_bounds_fpa(a, b)
%     tau = -1e-6;
% 
%     A = [-1.20327, -0.07202, -0.93635;
%           1.18810, -1.50015,  0.93519];
%     B = [ 1.21464, -0.10502;
%           0.12023,  0.19387;
%          -1.36695,  0.12201];
%     BA = B * A;
%     W = [zeros(2,2), A;
%          zeros(3,2), BA];
% 
%     % sech^2 for each state
%     S = zeros(5, 2);
%     for j = 1:5
%         S(j, :) = sech2_interval(a(j), b(j));
%     end
% 
%     J_lb = zeros(5,5); J_ub = zeros(5,5);
%     for i = 1:5
%         for j = 1:5
%             if W(i,j) > 0
%                 temp_lb = W(i,j) * S(j,1);
%                 temp_ub = W(i,j) * S(j,2);
%             elseif W(i,j) < 0
%                 temp_lb = W(i,j) * S(j,2);
%                 temp_ub = W(i,j) * S(j,1);
%             else
%                 temp_lb = 0; temp_ub = 0;
%             end
%             if i == j
%                 J_lb(i,j) = tau + temp_lb;
%                 J_ub(i,j) = tau + temp_ub;
%             else
%                 J_lb(i,j) = temp_lb;
%                 J_ub(i,j) = temp_ub;
%             end
%         end
%     end
% end
% 
% 
% %% ========================================================================
% %  HELPER: sech^2(x) over [a,b]
% % =========================================================================
% function S = sech2_interval(a, b)
%     if a > b, error('a must <= b'); end
% 
%     val_a = 1 / cosh(a)^2;
%     val_b = 1 / cosh(b)^2;
% 
%     if a <= 0 && b >= 0
%         s_min = min(val_a, val_b);
%         s_max = 1;
%     elseif b < 0
%         s_min = val_a;  % increasing
%         s_max = val_b;
%     elseif a > 0
%         s_min = val_b;  % decreasing
%         s_max = val_a;
%     else
%         s_min = min(val_a, val_b);
%         s_max = max(val_a, val_b);
%     end
%     S = [s_min, s_max];
% end

%------------- END OF CODE V1.4 --------------

%%

%------------- END OF CODE V1.5 -------------- |--> Includes ACC example 17

function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b, system_choice)
% Jacob_bounds_element_wise.m: computes Jacobian bounds using element-wise
% interval arithmetic.
%
% Inputs:
%   a - vector of lower bounds for x
%   b - vector of upper bounds for x
%   system_choice - ID of the system (14=FPA, 15=Spiral, 16=MNIST)
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

    % sech^2 for each state
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
%  ACC: Neural ODE Jacobian Bounds (8D)
% =========================================================================
function [J_lb, J_ub] = compute_jacobian_bounds_acc(a, b)
    global w1_acc b1_acc w2_acc b2_acc
    
    % Lazy load if missing (same as System_description)
    if isempty(w1_acc)
        pl = load("plant_3rd_order_tanh.mat");
        w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
        b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
        w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
        w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
        b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));

        % FORCE column vectors here too!
        b1_acc = b1_acc(:);
        b2_acc = b2_acc(:);
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
    
    % Row 3 (dx3) mappings:
    J_lb(3, nn_idx) = J_nn_lb(1, :);
    J_ub(3, nn_idx) = J_nn_ub(1, :);
    
    % Row 6 (dx6) mappings:
    J_lb(6, nn_idx) = J_nn_lb(2, :);
    J_ub(6, nn_idx) = J_nn_ub(2, :);

    % Note: Rows 7 and 8 remain strictly 0 because we froze accelerations.
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

%------------- END OF CODE V1.5 --------------