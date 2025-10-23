% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 23 October 2025
% Last revision: October 2025

%------------- BEGIN CODE --------------

function [J_lb, J_ub] = Jacob_bounds_element_wise(a, b)
% Jacob_bounds_element_wise.m computes Jacobian bounds using element-wise interval arithmetic
% Inputs:
%   a - 2x1 vector of lower bounds for x
%   b - 2x1 vector of upper bounds for x
% Outputs:
%   J_lb - 2x2 matrix of lower bounds for the Jacobian
%   J_ub - 2x2 matrix of upper bounds for the Jacobian

fprintf('Computing Jacobian bounds based on reachable tube bounds:\n');
fprintf('a = [%s]\n', num2str(a', '%.4f '));
fprintf('b = [%s]\n', num2str(b', '%.4f '));

[J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b);

fprintf('\nLower bounds of Jacobian (J_lb):\n');
disp(J_lb);
fprintf('\nUpper bounds of Jacobian (J_ub):\n');
disp(J_ub);

% Display Jacobian bounds in a formatted way
fprintf('\nJacobian Matrix Bounds Summary:\n');
fprintf('J(1,1): [%.4f, %.4f]\n', J_lb(1,1), J_ub(1,1));
fprintf('J(1,2): [%.4f, %.4f]\n', J_lb(1,2), J_ub(1,2));
fprintf('J(2,1): [%.4f, %.4f]\n', J_lb(2,1), J_ub(2,1));
fprintf('J(2,2): [%.4f, %.4f]\n', J_lb(2,2), J_ub(2,2));

end

function [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b)
% COMPUTE_JACOBIAN_BOUNDS_SPIRAL Computes bounds on the Jacobian matrix of the spiral_non neural ODE

    % Check input validity
    if length(a) ~= 2 || length(b) ~= 2 || any(a > b)
        error("Inputs a and b must be 2x1 vectors with a <= b element-wise");
    end
    
    % Parameters of neural ode (from spiral_non.m)
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

    % Compute the interval bounds for each element of z = w1*x + b1
    num_neurons = size(w1, 1); % 10 neurons
    z_lb = zeros(num_neurons, 1);
    z_ub = zeros(num_neurons, 1);
    
    for i = 1:num_neurons
        temp_lb = 0;
        temp_ub = 0;
        for j = 1:size(w1, 2) % Iterate over input dimensions (2 for x)
            if w1(i,j) >= 0
                temp_lb = temp_lb + w1(i,j) * a(j);
                temp_ub = temp_ub + w1(i,j) * b(j);
            else
                temp_lb = temp_lb + w1(i,j) * b(j);
                temp_ub = temp_ub + w1(i,j) * a(j);
            end
        end
        % Add bias b1_i
        z_lb(i) = temp_lb + b1(i);
        z_ub(i) = temp_ub + b1(i);
    end
    
    % Compute sech^2 intervals for each element of z
    sech2_z_intervals = zeros(num_neurons, 2); % Each row: [s_min, s_max]
    for j = 1:num_neurons
        sech2_z_intervals(j, :) = sech2_interval(z_lb(j), z_ub(j));
    end
    
    % Initialize Jacobian bounds
    J_lb = zeros(2, 2);
    J_ub = zeros(2, 2);
    
    % The Jacobian J = w2 * diag(sech^2(z)) * w1
    for i = 1:2 % Output dimensions of dx (2 for spiral_non)
        for j = 1:2 % Input dimensions of x (2 for spiral_non)
            sum_lb = 0;
            sum_ub = 0;
            for k = 1:num_neurons % Iterate over hidden neurons (10)
                % Term: w2_ik * sech^2(z_k) * w1_kj
                sech2_min = sech2_z_intervals(k,1);
                sech2_max = sech2_z_intervals(k,2);
                
                % Product of w2_ik and w1_kj
                prod_w = w2(i,k) * w1(k,j);
                
                if prod_w >= 0
                    term_k_lb = prod_w * sech2_min;
                    term_k_ub = prod_w * sech2_max;
                else
                    term_k_lb = prod_w * sech2_max;
                    term_k_ub = prod_w * sech2_min;
                end
                
                sum_lb = sum_lb + term_k_lb;
                sum_ub = sum_ub + term_k_ub;
            end
            J_lb(i,j) = sum_lb;
            J_ub(i,j) = sum_ub;
        end
    end
end

function S = sech2_interval(a, b)
% SECH2_INTERVAL Computes the interval of sech^2(x) over [a, b]
    
    if a > b
        error("Lower bound a must be less than or equal to upper bound b");
    end
    
    % sech^2(x) = 1 / cosh^2(x), max at 0, decreases as |x| increases
    val_a = 1 / cosh(a)^2;
    val_b = 1 / cosh(b)^2;
    
    if (a <= 0 && b >= 0) % Interval crosses or includes 0
        s_max = 1; % sech^2(0) = 1 is the global maximum
        s_min = min(val_a, val_b);
    else % Interval does not include 0, monotonic over the interval
        s_min = min(val_a, val_b);
        s_max = max(val_a, val_b);
    end
    
    S = [s_min, s_max];
end


%------------- END OF CODE --------------