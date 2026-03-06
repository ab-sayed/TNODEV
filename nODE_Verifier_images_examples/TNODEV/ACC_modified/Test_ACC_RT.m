% Test_ACC_Reachable_Tube.m
clear; clc; close all;

%% 1. Load the NN weights
global w1_acc b1_acc w2_acc b2_acc
pl = load("plant_3rd_order_tanh.mat");
w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));

% Force column vectors
b1_acc = b1_acc(:); 
b2_acc = b2_acc(:);

%% 2. Define Initial Set (X0) and Time Step
t_init = 0;
t_final = 0.1; % 0.1 second simulation step
delta_t = t_final - t_init;

% Arbitrary initial state for the 8D ACC system
x_nom = [10; 20; 0; 50; 15; 0; 0; 0]; 
epsilon = 0.5; % Define a bounding box around the nominal state

x_low = x_nom - epsilon;
x_up  = x_nom + epsilon;

%% 3. Compute the Reachable Tube (Euler Bloat + Picard Refinement)
% Step A: Initial velocity bounds
[dx_lb_init, dx_ub_init] = compute_dynamics_interval_acc(x_low, x_up);

% Step B: Euler Bloat (Initial guess of the tube)
tube_low_1 = x_low + min(0, dx_lb_init * delta_t);
tube_up_1  = x_up  + max(0, dx_ub_init * delta_t);

% Step C: Picard Refinement (Evaluate velocity over the bloated tube)
[dx_lb_tube, dx_ub_tube] = compute_dynamics_interval_acc(tube_low_1, tube_up_1);

% Step D: Final Safe Reachable Tube Bounds
tube_low = x_low + min(0, dx_lb_tube * delta_t);
tube_up  = x_up  + max(0, dx_ub_tube * delta_t);

%% 4. Simulate Random Trajectories and Plot
num_samples = 1000;

figure('Color', 'w'); hold on; grid on;
% We will project the 8D system onto States 3 and 6 (the NN-driven states)
xlabel('State x_3 (Lead Car Jerk)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('State x_6 (Ego Car Jerk)', 'FontSize', 12, 'FontWeight', 'bold');
title('ACC Reachable Tube vs Simulated Trajectories', 'FontSize', 14);

% Draw Initial Set X0 (Red dashed box)
plot_box(x_low([3,6]), x_up([3,6]), 'r', '--', 2);

% Draw Final Reachable Tube (Blue solid box)
plot_box(tube_low([3,6]), tube_up([3,6]), 'b', '-', 2);

% Run Simulations
disp('Simulating trajectories...');
for i = 1:num_samples
    % Sample a random initial state uniformly from X0
    x0_sample = x_low + rand(8,1) .* (x_up - x_low);
    
    % Simulate the true ODE
    [t_sim, x_sim] = ode45(@acc_ode, [t_init, t_final], x0_sample);
    
    % Plot the trajectory (black lines)
    plot(x_sim(:,3), x_sim(:,6), 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    
    % Mark the final point of the trajectory
    plot(x_sim(end,3), x_sim(end,6), 'k.', 'MarkerSize', 8, 'HandleVisibility', 'off');
end

% Add a dummy black line just for the legend
plot(NaN, NaN, 'k-', 'LineWidth', 1);
legend('Initial Set X_0', 'Reachable Tube (Picard)', 'Simulated Trajectories', 'Location', 'best');
axis tight;
disp('Done!');

%% ========================================================================
%  HELPER FUNCTIONS FOR THE SCRIPT
% =========================================================================

function plot_box(low, up, color, linestyle, linewidth)
    % Helper to draw a 2D bounding box
    x = [low(1), up(1), up(1), low(1), low(1)];
    y = [low(2), low(2), up(2), up(2), low(2)];
    plot(x, y, 'Color', color, 'LineStyle', linestyle, 'LineWidth', linewidth);
end

function dx = acc_ode(~, x)
    % The true continuous ODE function for ode45
    global w1_acc b1_acc w2_acc b2_acc
    nn_in = x([3, 6, 7, 8]);
    
    z = w1_acc * nn_in + b1_acc;
    h = tanh(z);
    nn_out = w2_acc * h + b2_acc;
    
    dx = zeros(8,1);
    dx(1) = x(2);
    dx(2) = x(3);
    dx(3) = nn_out(1);
    dx(4) = x(5);
    dx(5) = x(6);
    dx(6) = nn_out(2);
    dx(7) = nn_out(3); % Corrected mappings
    dx(8) = nn_out(4);
end

function [dx_lb, dx_ub] = compute_dynamics_interval_acc(a, b)
    % The interval arithmetic dynamics (same as your main code)
    global w1_acc b1_acc w2_acc b2_acc
    nn_idx = [3, 6, 7, 8];
    a_sub = a(nn_idx); b_sub = b(nn_idx);
    num_neurons = size(w1_acc, 1); out_dim = size(w2_acc, 1);

    z_lb = zeros(num_neurons, 1); z_ub = zeros(num_neurons, 1);
    for i = 1:num_neurons
        t_lb = 0; t_ub = 0;
        for j = 1:4
            if w1_acc(i,j) >= 0
                t_lb = t_lb + w1_acc(i,j) * a_sub(j); t_ub = t_ub + w1_acc(i,j) * b_sub(j);
            else
                t_lb = t_lb + w1_acc(i,j) * b_sub(j); t_ub = t_ub + w1_acc(i,j) * a_sub(j);
            end
        end
        z_lb(i) = t_lb + b1_acc(i); z_ub(i) = t_ub + b1_acc(i);
    end

    h_lb = tanh(z_lb); h_ub = tanh(z_ub);

    nn_lb = zeros(out_dim, 1); nn_ub = zeros(out_dim, 1);
    for i = 1:out_dim
        t_lb = 0; t_ub = 0;
        for j = 1:num_neurons
            if w2_acc(i,j) >= 0
                t_lb = t_lb + w2_acc(i,j) * h_lb(j); t_ub = t_ub + w2_acc(i,j) * h_ub(j);
            else
                t_lb = t_lb + w2_acc(i,j) * h_ub(j); t_ub = t_ub + w2_acc(i,j) * h_lb(j);
            end
        end
        nn_lb(i) = t_lb + b2_acc(i); nn_ub(i) = t_ub + b2_acc(i);
    end

    dx_lb = zeros(8,1); dx_ub = zeros(8,1);
    dx_lb(1) = a(2); dx_ub(1) = b(2);          
    dx_lb(2) = a(3); dx_ub(2) = b(3);          
    dx_lb(3) = nn_lb(1); dx_ub(3) = nn_ub(1);  
    dx_lb(4) = a(5); dx_ub(4) = b(5);          
    dx_lb(5) = a(6); dx_ub(5) = b(6);          
    dx_lb(6) = nn_lb(2); dx_ub(6) = nn_ub(2);  
    dx_lb(7) = nn_lb(3); dx_ub(7) = nn_ub(3);  
    dx_lb(8) = nn_lb(4); dx_ub(8) = nn_ub(4);  
end