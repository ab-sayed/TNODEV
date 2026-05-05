% Compare_ACC_Tubes.m
% Compares Euler-Picard Interval RT against CORA Zonotope RT for ACC
clear; clc; close all;

%% 1. Load the NN weights
global w1_acc b1_acc w2_acc b2_acc
pl = load("plant_3rd_order_tanh.mat");
w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
b1_acc = b1_acc(:); b2_acc = b2_acc(:);

%% 2. Define Initial Conditions (NNV Benchmark)
t_init = 0;
t_final = 0.1;
delta_t = t_final - t_init;

x_low = [90; 20; -5; 30; 30; 0; 0; 0];
x_up  = [92; 30; -5; 31; 30.5; 0; 0; 0];

% For CORA, we need center (x0) and radius
x0 = (x_up + x_low) / 2;
Initial_radius = (x_up - x_low) / 2;

%% 3. Compute Tube 1: Euler-Picard (Interval Arithmetic)
disp('--- Running Euler-Picard RT ---');
tic;
[dx_lb_init, dx_ub_init] = compute_dynamics_interval_acc(x_low, x_up);
tube_low_1 = x_low + min(0, dx_lb_init * delta_t);
tube_up_1  = x_up  + max(0, dx_ub_init * delta_t);
[dx_lb_tube, dx_ub_tube] = compute_dynamics_interval_acc(tube_low_1, tube_up_1);

ep_lb = x_low + min(0, dx_lb_tube * delta_t);
ep_ub = x_up  + max(0, dx_ub_tube * delta_t);
ep_time = toc;
fprintf('Euler-Picard completed in %.4f seconds.\n\n', ep_time);

%% 4. Compute Tube 2: CORA (Zonotopes)
disp('--- Running CORA RT ---');
cora_params.timeStep      = 0.05; % Use 2 steps of 0.05s to reach 0.1s
cora_params.tFinal        = t_final;
cora_params.taylorTerms   = 3;
cora_params.zonotopeOrder = 10;
cora_params.alg           = 'lin';
cora_params.tensorOrder   = 2;
cora_params.x0            = x0;
cora_params.Initial_radius= Initial_radius;

cora_bounds = acc_CORA_RT(cora_params);
cora_lb = cora_bounds.lb;
cora_ub = cora_bounds.ub;

%% 5. Numerical Comparison of Box Sizes (Widths)
disp('--- Bounding Box Width Comparison ---');
ep_widths = ep_ub - ep_lb;
cora_widths = cora_ub - cora_lb;

fprintf('State | Euler-Picard Width | CORA Width | Difference\n');
fprintf('------------------------------------------------------\n');
for i = 1:8
    fprintf('  x%d  | %18.6f | %10.6f | %10.6f\n', ...
        i, ep_widths(i), cora_widths(i), ep_widths(i) - cora_widths(i));
end
fprintf('\n');

%% 6. Plotting the Comparison (States 3 and 6)
num_samples = 500;
figure('Color', 'w', 'Position', [100, 100, 900, 600]); 
hold on; grid on;

xlabel('State x_3 (Lead Car Acceleration/Jerk)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('State x_6 (Ego Car Acceleration/Jerk)', 'FontSize', 12, 'FontWeight', 'bold');
title('Reachable Tube Comparison: Euler-Picard vs. CORA', 'FontSize', 14);

% 1. Plot Euler-Picard Box (Blue)
plot_box(ep_lb([3,6]), ep_ub([3,6]), 'b', '-', 2.5);

% 2. Plot CORA Box (Green)
plot_box(cora_lb([3,6]), cora_ub([3,6]), 'g', '-', 2.5);

% 3. Plot Initial Set (Red Dashed Point/Line)
plot_box(x_low([3,6]), x_up([3,6]), 'r', '--', 2);
plot(-5, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'HandleVisibility', 'off'); % Mark the dot explicitly

% 4. Simulate Random Trajectories (Black lines)
for i = 1:num_samples
    x0_sample = x_low + rand(8,1) .* (x_up - x_low);
    [~, x_sim] = ode45(@acc_ode_sim, [t_init, t_final], x0_sample);
    plot(x_sim(:,3), x_sim(:,6), 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(x_sim(end,3), x_sim(end,6), 'k.', 'MarkerSize', 6, 'HandleVisibility', 'off');
end

% Dummy plots for the legend
plot(NaN, NaN, 'k-', 'LineWidth', 1);
legend('Euler-Picard RT (Intervals)', 'CORA RT (Zonotopes)', 'Initial Set X_0', 'Simulated Trajectories', 'Location', 'best');
axis tight;

%% ========================================================================
%  HELPER FUNCTIONS FOR THE SCRIPT
% =========================================================================

function plot_box(low, up, color, linestyle, linewidth)
    x = [low(1), up(1), up(1), low(1), low(1)];
    y = [low(2), low(2), up(2), up(2), low(2)];
    plot(x, y, 'Color', color, 'LineStyle', linestyle, 'LineWidth', linewidth);
end

function dx = acc_ode_sim(~, x)
    global w1_acc b1_acc w2_acc b2_acc
    nn_in = x([3, 6, 7, 8]);
    z = w1_acc * nn_in + b1_acc;
    h = tanh(z);
    nn_out = w2_acc * h + b2_acc;
    dx = [x(2); x(3); nn_out(1); x(5); x(6); nn_out(2); nn_out(3); nn_out(4)];
end

function [dx_lb, dx_ub] = compute_dynamics_interval_acc(a, b)
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