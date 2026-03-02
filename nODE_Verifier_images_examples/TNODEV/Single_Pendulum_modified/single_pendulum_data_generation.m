%% Generate Training Data for Single Pendulum Neural ODE Surrogate
clear; clc;

% 1. Setup Simulation Parameters
num_sims = 100;           % Number of random initial states to simulate
% t_final = 5.0;            % Simulate each for 5 seconds
t_final = 1.0;            % Simulate each for 1 second similar tot he specification in ARCH-COMP ∀t ∈ [0.5, 1]
Ts = 0.05;                % Control sampling time (standard for this benchmark)
time_steps = 0:Ts:t_final;

% Initialize arrays to hold our training data
X_train = [];
dX_train = [];

% 2. Load the discrete controller
% This loads W (weights), b (biases), and act_fcns
load('controller_single_pendulum.mat'); 

disp('Simulating hybrid trajectories...');

for i = 1:num_sims
    % Random initial conditions: theta in [0.5, 1.2], dtheta in [0, 0.2]
    % Adjust these bounds based on your specific reachability target
    % x_current = [0.5 + 0.7*rand(); 0.2*rand(); 0]; 
    x_current = [1.0 + 0.3*rand(); 0.0 + 0.3*rand(); 0]; % Instead of generating random initial states across the whole space, generate dense data strictly inside and slightly around the verification target

    for t = 1:(length(time_steps)-1)
        % --- A. DISCRETE CONTROLLER WAKES UP ---
        % Pass the first two states (theta, dtheta) through the NN
        % Note: You may need to transpose W/b depending on how they were saved
        h1 = max(0, W{1} * x_current(1:2) + b{1}); % ReLU
        h2 = max(0, W{2} * h1 + b{2});             % ReLU
        a_control = W{3} * h2 + b{3};              % Linear
        
        % --- B. RECORD GROUND TRUTH DERIVATIVE ---
        % We calculate the exact continuous derivative at this exact state and control input
        dx_current = dynamics_sp(x_current, a_control);
        
        % Save to our dataset
        X_train = [X_train, x_current];
        dX_train = [dX_train, dx_current];
        
        % --- C. CONTINUOUS PLANT EVOLUTION ---
        % Simulate the physics forward for 0.05s holding 'a' constant
        [~, x_traj] = ode45(@(t,x) dynamics_sp(x, a_control), [0 Ts], x_current);
        
        % Update the state for the next discrete step
        x_current = x_traj(end, :)';
    end
end

disp(['Data generation complete! Collected ', num2str(size(X_train, 2)), ' data points.']);
save('pendulum_surrogate_data.mat', 'X_train', 'dX_train');

%% The Plant Dynamics Function
function [dx] = dynamics_sp(x,a)
    l=0.5; m=0.5; g= 1; c= 0;
    dx = zeros(3,1);
    dx(1,1) = x(2);
    dx(2,1) = g/l * sin(x(1)) + (a - c*x(2))/(m*l^2);
    dx(3,1) = 1; % Time state
end