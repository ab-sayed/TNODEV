%% Tanh AF  `FPA System`

% Main execution block

% For the 1st approach
% a = [ -1.2891   -0.6803   -0.7335    0.3538    0.6882]'    %-ones(5,1) % Lower bounds: [-1, -1, -1, -1, -1] / [-1.7436   -2.3294   -1.0296    0.0066    0.6882]
% b = [0.0112   -0.3005    0.8111    0.5333    2.4518]'  % Upper bounds: [1, 1, 1, 1, 1] / [0.0112   -0.3005    0.8111    0.5333    2.8406]
% [J_lb, J_ub] = bound_jacobian(a, b); % Compute Jacobian bounds

% % For the 2nd approach
% 
% a = [ -1.2908   -0.6819   -0.7345    0.3535    1.9703]'    %-ones(5,1) % Lower bounds: [-1, -1, -1, -1, -1] / [-1.7436   -2.3294   -1.0296    0.0066    0.6882]
% b = [ -0.9056   -0.3133   -0.3461    0.4671    2.4543]'  % Upper bounds: [1, 1, 1, 1, 1] / [0.0112   -0.3005    0.8111    0.5333    2.8406]
% [J_lb, J_ub] = bound_jacobian(a, b); % Compute Jacobian bounds

% From CORA 

a = [ -1.2854    -0.67664    -0.73291     0.35404        0.69]'    %-ones(5,1) % Lower bounds: [-1, -1, -1, -1, -1] / [-1.7436   -2.3294   -1.0296    0.0066    0.6882]
b = [ 0.01    -0.30409        0.81     0.53323      2.4463]'  % Upper bounds: [1, 1, 1, 1, 1] / [0.0112   -0.3005    0.8111    0.5333    2.8406]
[J_lb, J_ub] = bound_jacobian(a, b); % Compute Jacobian bounds

% Display results
disp('Lower bounds of Jacobian (J_lb):');
disp(J_lb);
disp('Upper bounds of Jacobian (J_ub):');
disp(J_ub);

% Main function to compute Jacobian bounds
function [J_lb, J_ub] = bound_jacobian(a, b)
    % BOUND_JACOBIAN Computes bounds on the Jacobian matrix of the FPA neural ODE
    % Inputs:
    %   a - 5x1 vector of lower bounds for x
    %   b - 5x1 vector of upper bounds for x
    % Outputs:
    %   J_lb - 5x5 matrix of lower bounds for the Jacobian
    %   J_ub - 5x5 matrix of upper bounds for the Jacobian

    % Check input validity
    if length(a) ~= 5 || length(b) ~= 5 || any(a > b)
        error('Inputs a and b must be 5x1 vectors with a <= b element-wise');
    end

    % Define constants
    tau = -1e-6; 
    % matrix A (2x3)
    A = [-1.20327, -0.07202, -0.93635;
         1.18810, -1.50015, 0.93519];
    % matrix B (3x2)
    B = [1.21464, -0.10502;
         0.12023, 0.19387;
         -1.36695, 0.12201];
    % Compute BA (3x3)
    BA = B * A;
    % Construct W (5x5)
    W = [zeros(2,2), A;
         zeros(3,2), BA];

    % Compute sech^2 intervals for each dimension
    S = zeros(5, 2);  % Each row: [s_min, s_max]
    for j = 1:5
        S(j, :) = sech2_interval(a(j), b(j));
    end

    % Initialize bound matrices
    J_lb = zeros(5, 5);
    J_ub = zeros(5, 5);

    % Compute bounds for each Jacobian element
    for i = 1:5
        for j = 1:5
            % Compute W_{ij} * sech^2([a_j, b_j])
            if W(i,j) > 0
                temp_lb = W(i,j) * S(j, 1);  % W_{ij} * s_min
                temp_ub = W(i,j) * S(j, 2);  % W_{ij} * s_max
            elseif W(i,j) < 0
                temp_lb = W(i,j) * S(j, 2);  % W_{ij} * s_max
                temp_ub = W(i,j) * S(j, 1);  % W_{ij} * s_min
            else
                temp_lb = 0;
                temp_ub = 0;
            end
            % Add tau for diagonal elements
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

% Helper function to compute sech^2 interval
function S = sech2_interval(a, b)
    % SECH2_INTERVAL Computes the interval of sech^2(x) over [a, b]
    % Inputs:
    %   a - Lower bound
    %   b - Upper bound
    % Output:
    %   S - [s_min, s_max] interval for sech^2(x)

    if a > b
        error('Lower bound a must be less than or equal to upper bound b');
    end

    % sech^2(x) = 1 / cosh^2(x), max at 0, decreases as |x| increases
    if b < 0
        % Interval in (-∞, 0), sech^2 increasing
        s_min = 1 / cosh(a)^2;
        s_max = 1 / cosh(b)^2;
    elseif a > 0
        % Interval in (0, ∞), sech^2 decreasing
        s_min = 1 / cosh(b)^2;
        s_max = 1 / cosh(a)^2;
    else
        % Interval crosses or includes 0, max at sech^2(0) = 1
        s_min = min(1 / cosh(a)^2, 1 / cosh(b)^2);
        s_max = 1;
    end

    S = [s_min, s_max];
end


