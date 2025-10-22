%% Tanh `Spiral system`

% % % For the 1st approach (single-step)
a = [-2.0122    -1.2886];
b = [2.2009  2.0821];

% % % For the 2nd approach (incremental)
% a = [-2.0124    -1.2894];
% b = [2.2009  2.0821];


% % For the 1st approach for final RS @t=0-2
% a = [1.7791        -0.2];
% b= [2.2     0.28589];

[J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b); % Call the function here

disp('Lower bounds of Jacobian (J_lb):');
disp(J_lb);
disp('Upper bounds of Jacobian (J_ub):');
disp(J_ub);

function [J_lb, J_ub] = compute_jacobian_bounds_spiral(a, b)
% COMPUTE_JACOBIAN_BOUNDS_SPIRAL Computes bounds on the Jacobian matrix of the spiral_non neural ODE
% Inputs:
%   a - 2x1 vector of lower bounds for x
%   b - 2x1 vector of upper bounds for x
% Outputs:
%   J_lb - 2x2 matrix of lower bounds for the Jacobian
%   J_ub - 2x2 matrix of upper bounds for the Jacobian

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

    % The dynamics are dx = w2 * tanh(w1*x + b1) + b2
    % The Jacobian J = d(dx)/dx = w2 * diag(sech^2(w1*x + b1)) * w1
    
    % Compute the interval for each element of (w1*x + b1)
    % This requires interval arithmetic for matrix multiplication and addition
    % For a simple interval [a,b] for x, and matrix M, vector v:
    % M*x + v will have lower bound M_lower*a + M_upper*b + v (if M_lower/upper are positive/negative parts)
    % For now, let\'s assume we can compute the interval for each element of (w1*x + b1)
    % For a general case, this is complex. Let\'s simplify for now based on the given structure.
    % w1 is 10x2, x is 2x1, b1 is 10x1. So w1*x is 10x1. (w1*x + b1) is 10x1.
    
    % Compute the interval bounds for each element of z = w1*x + b1
    % z_i = sum_j (w1_ij * x_j) + b1_i
    % For each z_i, we need its lower and upper bound.
    % Since x_j is in [a_j, b_j], the interval for w1_ij * x_j is:
    % [min(w1_ij*a_j, w1_ij*b_j), max(w1_ij*a_j, w1_ij*b_j)]
    % Then sum these intervals and add b1_i.
    
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
    % This is a product of three matrices. The middle one is a diagonal matrix
    % with interval entries. This is equivalent to:
    % J_ij = sum_k (w2_ik * sech^2(z_k) * w1_kj)
    
    for i = 1:2 % Output dimensions of dx (2 for spiral_non)
        for j = 1:2 % Input dimensions of x (2 for spiral_non)
            sum_lb = 0;
            sum_ub = 0;
            for k = 1:num_neurons % Iterate over hidden neurons (10)
                % Term: w2_ik * sech^2(z_k) * w1_kj
                % Let term_k = w2_ik * sech^2(z_k) * w1_kj
                % We need the interval for term_k
                
                % Interval for sech^2(z_k) is [sech2_z_intervals(k,1), sech2_z_intervals(k,2)]
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

% Helper function to compute sech^2 interval
function S = sech2_interval(a, b)
    % SECH2_INTERVAL Computes the interval of sech^2(x) over [a, b]
    % Inputs:
    %   a - Lower bound
    %   b - Upper bound
    % Output:
    %   S - [s_min, s_max] interval for sech^2(x)
    
    if a > b
        error("Lower bound a must be less than or equal to upper bound b");
    end
    
    % sech^2(x) = 1 / cosh^2(x), max at 0, decreases as |x| increases
    % The function is symmetric around 0.
    
    % Critical points for min/max are at the interval boundaries or at x=0 if 0 is included.
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