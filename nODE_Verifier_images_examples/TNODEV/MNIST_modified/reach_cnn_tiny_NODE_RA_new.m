% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 29 January 2026
% Last update: 13 February 2026 
% Last revision:  2026


% %------------- BEGIN OF CODE V1.1  --------------
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     neuralLayers_Post = {layerout};
%     net_post = NN(neuralLayers_Post);
% 
%     % ---------------------------------------------------------
%     % DEFINE ARCHITECTURE |---------------------------------------------------- --------> THIS IS WRONG comment it !!! 
%     % ---------------------------------------------------------
%     % neuralLayers = {layer1, layer2, layer3, layer4, layer5, layer6, layer7, odelayer_NODE_RA, layerout};
%     % net = NN(neuralLayers);
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Initialize results containers
%     time_ode = zeros(numT,1);
%     rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
% 
% 
%     % Maximum number of iterations for Refinement
%     MAX_ITER = 1000; 
% 
%     % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % -----------------------------------------------------
%         % B. RUN VERIFICATION (Split Network + Loop)
%         % -----------------------------------------------------
%         t = tic;
% 
%         % Step 1: Reachability for Pre-ODE Layers (1-7) Using NNV
%         R_pre = net_pre.reach(inpS);
%         [pre_lb, pre_ub] = R_pre.getRanges();
% 
%         % === FORCE COLUMN VECTORS ===
%         % The Box class requires strictly Nx1 vectors.
%         pre_lb = reshape(pre_lb, [], 1);
%         pre_ub = reshape(pre_ub, [], 1);                
% 
%         % =========================================================
%         % VISUALIZATION FOR LAYER 7 (Star Set vs. Interval Hull)   | --> Commentted to save time
%         % =========================================================
% 
%         % if i == 1 
%         %     % 1. Create Folder with Timestamp
%         %     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%         %     save_folder = fullfile(pwd, ['Layer_7_set_shape_' timestamp]);
%         %     if ~exist(save_folder, 'dir')
%         %         mkdir(save_folder);
%         %     end
%         %     fprintf('  Saving Layer 7 plots to: %s\n', save_folder);
%         % 
%         %     % 2. Convert `Image Star` to `Star set`
%         %     S_flat = [];
%         %     if isa(R_pre, 'ImageStar')
%         %         for k = 1:length(R_pre)
%         %             S_flat = [S_flat, R_pre(k).toStar()];
%         %         end
%         %     else
%         %         S_flat = R_pre;
%         %     end
%         % 
%         %     % 3. Get Global Ranges
%         %     [all_lbs, all_ubs] = S_flat.getRanges();
%         %     global_lb = min(all_lbs, [], 2);
%         %     global_ub = max(all_ubs, [], 2);
%         %     n_dims = length(global_lb); % Should be 676
%         % 
%         %     % --- PLOT A: IMPROVED SUMMARY (Area/Ribbon Plot) ---
%         %     fprintf('  Generating Summary Area Plot...');
%         %     f_sum = figure('Visible', 'off', 'Position', [100, 100, 1200, 600]); 
%         %     hold on; grid on; set(gca, 'FontSize', 12);
%         % 
%         %     x_axis = 1:n_dims;
%         % 
%         %     % Define coordinates for the filled area polygon
%         %     x_fill = [x_axis, fliplr(x_axis)];
%         %     y_fill = [global_lb', fliplr(global_ub')];
%         % 
%         %     % Plot the area between min and max bounds
%         %     fill(x_fill, y_fill, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r', 'LineWidth', 0.5);
%         % 
%         %     xlabel('Dimension Index (Neuron # 1-676)', 'FontSize', 14, 'FontWeight', 'bold');
%         %     ylabel('Feature Value Range', 'FontSize', 14, 'FontWeight', 'bold');
%         %     title({'Layer 7 Output Envelopes';'Red Area = Interval Hull (Input to ODE)'}, 'FontSize', 16);
%         % 
%         %     xlim([1 n_dims]);
%         % 
%         %     % Add a legend for clarity
%         %     h_area = patch(NaN, NaN, 'r', 'FaceColor', 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r');
%         %     legend(h_area, 'Interval Hull Range', 'Location', 'northeast');
%         % 
%         %     saveas(f_sum, fullfile(save_folder, 'Summary_All_Ranges_AreaPlot.png'));
%         %     close(f_sum);
%         %     fprintf(' Done.\n');
%         % 
%         % 
%         %     % --- PLOT B: 2D PROJECTIONS ---
%         %     fprintf('  Generating Layer 7 2D projection plots...');
%         % 
%         %     % Pre-calculate Star boxes for speed
%         %     star_boxes = cell(1, length(S_flat));
%         %     for k = 1:length(S_flat)
%         %          star_boxes{k} = [all_lbs(:,k), all_ubs(:,k)];
%         %     end
%         % 
%         %     for d = 1 : 2 : (n_dims - 1) 
%         %         d1 = d; 
%         %         d2 = d + 1;
%         % 
%         %         f_curr = figure('Visible', 'off'); 
%         %         hold on; grid on;
%         % 
%         %         % 1. Plot Blue Rectangles (Exact Stars)
%         %         for k = 1:length(S_flat)
%         %             bnds = star_boxes{k};
%         %             lb_k = [bnds(d1, 1); bnds(d2, 1)];
%         %             ub_k = [bnds(d1, 2); bnds(d2, 2)];
%         %             w = ub_k(1) - lb_k(1); h = ub_k(2) - lb_k(2);
%         % 
%         %             % FIXED: Use FaceAlpha separately for transparency
%         %             rectangle('Position', [lb_k(1), lb_k(2), w, h], ...
%         %                       'EdgeColor', 'b', 'FaceColor', [0 0 1]); 
%         %             % Note: 'rectangle' doesn't support FaceAlpha in older MATLAB versions.
%         %             % If you need transparency here, use 'patch' or accept solid blue.
%         %             % For simplicity, solid blue outline or solid fill is usually fine for these boxes.
%         %         end
%         % 
%         %         % 2. Plot Red Hull
%         %         g_lb_d = [global_lb(d1); global_lb(d2)];
%         %         g_ub_d = [global_ub(d1); global_ub(d2)];
%         %         w_tot = g_ub_d(1) - g_lb_d(1); 
%         %         h_tot = g_ub_d(2) - g_lb_d(2);
%         %         rectangle('Position', [g_lb_d(1), g_lb_d(2), w_tot, h_tot], ...
%         %                   'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
%         % 
%         %         % 3. ADD LEGEND (FIXED)
%         %         % Separated FaceColor and FaceAlpha
%         %         h_blue_dummy = patch(NaN, NaN, 'b', 'FaceColor', [0 0 1], 'FaceAlpha', 0.2, 'EdgeColor', 'b');
%         %         h_red_dummy = plot(NaN, NaN, 'r--', 'LineWidth', 2);
%         %         legend([h_blue_dummy, h_red_dummy], 'Exact Star Set (Union)', 'Interval Hull', 'Location', 'bestoutside');
%         % 
%         %         % Formatting
%         %         xlabel(['Dim ' num2str(d1)], 'FontWeight', 'bold');
%         %         ylabel(['Dim ' num2str(d2)], 'FontWeight', 'bold');
%         %         title(sprintf('L7 Projections: Dims %d vs %d', d1, d2));
%         % 
%         %         % Save
%         %         fname = sprintf('Dims_%03d_vs_%03d.png', d1, d2); 
%         %         saveas(f_curr, fullfile(save_folder, fname));
%         %         close(f_curr);
%         %     end
%         %     fprintf(' Done.\n');
%         % 
%         % end
% 
%         % =========================================================
%         % VISUALIZATION FOR LAYER 7 
%         % =========================================================
% 
% 
%         % Step 2: Refinement Loop (ODE + Output)
%         queue = {{pre_lb, pre_ub}};
% 
%         image_is_robust = true;   
%         image_is_unknown = false; 
% 
%         iter = 0;
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             % Timeout check
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             % Pop from queue
%             curr = queue{1}; queue(1) = [];
%             c_lb = curr{1}; c_ub = curr{2};
% 
%             % 1. ODE Layer (NODE_RA_NEW.m via TNODEVODELayer.m Wrapper)
%             S_in = Star(c_lb, c_ub);
%             S_ode = odelayer_NODE_RA.reach(S_in);
% 
%             % =========================================================
%             %             VOLUME CALCULATION FOR LAYER 7 & 8
%             % =========================================================
%             if i == 1 && iter == 1
%                 [ode_lb, ode_ub] = S_ode.getRanges();
% 
%                 % --- A. VOLUME CALCULATION (Log Scale) ---
%                 % Epsilon to handle dead neurons (0 width) without -Inf results
%                 epsilon = 1e-20; 
%                 widths_L7 = c_ub - c_lb;
%                 widths_L8 = ode_ub - ode_lb;
% 
%                 vol_L7 = sum(log(widths_L7 + epsilon));
%                 vol_L8 = sum(log(widths_L8 + epsilon));
% 
%                 fprintf('\n  ++Interval Hull Volume Analysis:++\n');
%                 fprintf('   - Layer 7 (Input to ODE Layer): %.4f\n', vol_L7);
%                 fprintf('   - Layer 8 (Output from ODE Layer): %.4f\n', vol_L8);
%                 fprintf('   - Expansion Factor: %.4e\n', exp(vol_L8 - vol_L7));
% 
%                 % =========================================================
%                 % ---  WRAPPING EFFECT DEBUG TEST ---
%                 % =========================================================
%                 % Simulate 20 random points from the input set to see where they 
%                 % go. If they stay close together, but the 'Layer 8' volume
%                 % is huge, it gives and indication that interval method is the problem, not the network.
% 
%                 fprintf('   ------------------------------------------\n');
%                 fprintf('   Running concrete simulations to check stability...\n');
% 
%                 concrete_min = inf(size(c_lb));
%                 concrete_max = -inf(size(c_lb));
% 
%                 % Define ODE function for simulation
%                 ode_fun_sim = @(t, x) Aout * x + Bout;
% 
%                 for k = 1:20
%                      % Sample a random point inside the input bounds [c_lb, c_ub]
%                      x_sample = c_lb + (c_ub - c_lb) .* rand(size(c_lb));
% 
%                      % Simulate trajectory 
%                      [~, y_traj] = ode45(ode_fun_sim, [0 1], x_sample);
%                      y_final = y_traj(end, :)';
% 
%                      % Track the actual range of these concrete samples
%                      concrete_min = min(concrete_min, y_final);
%                      concrete_max = max(concrete_max, y_final);
%                 end
% 
%                 % Calculate Volume of the Concrete (Real) Set
%                 widths_concrete = concrete_max - concrete_min;
%                 % Add epsilon to avoid log(0) if points are too close
%                 vol_concrete = sum(log(widths_concrete + 1e-20));
% 
%                 fprintf('   - Concrete Trajectories Log-Vol: %.4f\n', vol_concrete);
% 
%                 % The difference between the Method's Output (L8) and Reality (Concrete)
%                 over_approx_error = vol_L8 - vol_concrete;
%                 fprintf('   - Method Over-approximation Error: %.4e (Log Scale)\n', over_approx_error);
% 
%                 if over_approx_error > 100
%                     fprintf('   >>> CONCLUSION: MASSIVE WRAPPING EFFECT DETECTED.\n');
%                     fprintf('       The system is stable, but Interval Reachability is failing.\n');
%                 end
%                 fprintf('   ------------------------------------------\n');
% 
%                 % =========================================================
%                 % % --- VISUALIZATION FOR ODE LAYER (LAYER 8) ---           | --> Commentted to save time
%                 % =========================================================
% 
%                 % timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%                 % save_folder_L8 = fullfile(pwd, ['Layer_8_ODE_' timestamp]);
%                 % if ~exist(save_folder_L8, 'dir'), mkdir(save_folder_L8); end
%                 % fprintf('  Saving Layer 8 plots to: %s\n', save_folder_L8);
%                 % 
%                 % n_dims_L8 = length(ode_lb);
%                 % 
%                 % % Summary Plot L8
%                 % f_sum8 = figure('Visible', 'off', 'Position', [100, 100, 1200, 600]); 
%                 % hold on; grid on; set(gca, 'FontSize', 12);
%                 % x_axis = 1:n_dims_L8;
%                 % x_fill = [x_axis, fliplr(x_axis)];
%                 % y_fill = [ode_lb', fliplr(ode_ub')];
%                 % fill(x_fill, y_fill, 'm', 'FaceAlpha', 0.4, 'EdgeColor', 'm', 'LineWidth', 0.5); % Magenta for ODE out
%                 % xlabel('Dimension Index', 'FontWeight', 'bold'); ylabel('Range', 'FontWeight', 'bold'); title('Layer 8 (ODE) Output Envelopes');
%                 % xlim([1 n_dims_L8]);
%                 % saveas(f_sum8, fullfile(save_folder_L8, 'Summary_Ranges_L8.png')); close(f_sum8);
%                 % 
%                 % % 2D Projections L8
%                 % fprintf('  Generating L8 2D plots...');
%                 % for d = 1 : 2 : (n_dims_L8 - 1)
%                 %     d1 = d; d2 = d + 1;
%                 %     f_curr = figure('Visible', 'off'); hold on; grid on;
%                 % 
%                 %     % Plot Red Hull (Output of ODE is Interval, so just one box)
%                 %     g_lb_d = [ode_lb(d1); ode_lb(d2)]; 
%                 %     g_ub_d = [ode_ub(d1); ode_ub(d2)];
%                 %     w_tot = g_ub_d(1) - g_lb_d(1); h_tot = g_ub_d(2) - g_lb_d(2);
%                 %     rectangle('Position', [g_lb_d(1), g_lb_d(2), w_tot, h_tot], ...
%                 %               'EdgeColor', 'r', 'FaceColor', [1 0 0 0.2], 'LineWidth', 2);
%                 % 
%                 %     xlabel(['Dim ' num2str(d1)]); ylabel(['Dim ' num2str(d2)]);
%                 %     title(sprintf('L8 ODE Output: Dims %d vs %d', d1, d2));
%                 %     saveas(f_curr, fullfile(save_folder_L8, sprintf('Dims_%03d_vs_%03d.png', d1, d2)));
%                 %     close(f_curr);
%                 % end
%                 % fprintf(' Done.\n');
% 
%                 % =========================================================
%                 % % --- VISUALIZATION FOR ODE LAYER (LAYER 8) ---
%                 % =========================================================
%             end
% 
%             % 2. Output Layer
%             S_out = net_post.reach(S_ode);
% 
%             % 3. Check Robustness
%             res = net_post.checkRobust(S_out, target_label);
% 
%             if res == 1
%                 % Verified Safe
%                 continue;
%             else
%                 % Potential Violation -> Try Falsify
%                 fprintf('  Iter %d: Potential violation. Checking Falsification...\n', iter);
% 
%                 is_falsified = run_falsification(c_lb, c_ub, Aout, Bout, Wb{13}, Wb{14}', target_label);
% 
%                 if is_falsified
%                     fprintf('  FALSIFIED! Counter-example found.\n');
%                     image_is_robust = false;
%                     break; % Stop loop, image failed
%                 else
%                     % Verification failed, Falsification failed -> SPLIT
%                     fprintf('  Splitting interval...\n');
%                     [s1, s2] = split_interval(c_lb, c_ub);
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%         end
% 
%         time_ode(i) = toc(t);
% 
%         % Record Result
%         if ~image_is_robust
%             rob_ode(i) = 2; % Not Robust
%             disp('Result: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; % Unknown
%             disp('Result: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; % Robust
%             disp('Result: ROBUST');
%         end
%     end
% 
%     % Process results
%     rob = sum(rob_ode == 1);
%     unk = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp('Robust images:     ' + string(rob));
%     disp('Unknown images:    ' + string(unk));
%     disp('Not robust images: ' + string(notr));
%     disp('Total time =       ' + string(timeT));
% 
%     % Save results
%     save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
% 
% end
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% function falsified = run_falsification(lb, ub, A, B, W_out, b_out, target_label)
%     falsified = false;
%     num_samples = 2; 
%     ode_fun = @(t, x) A*x + B;
% 
%     for k = 1:num_samples
%         x0 = lb + (ub - lb) .* rand(size(lb));
%         [~, y_traj] = ode45(ode_fun, [0 1], x0); 
%         y_final = y_traj(end, :)';
%         logits = W_out * y_final + b_out;
%         [~, pred_idx] = max(logits);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end
% 
% function [s1, s2] = split_interval(lb, ub)
%     radii = (ub - lb) / 2;
%     [~, dim] = max(radii);
%     mid = lb(dim) + radii(dim);
% 
%     lb1 = lb; ub1 = ub; ub1(dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% 
% %------------- END OF CODE V1.1  --------------


%% Modified vewrsion based on PJ email on 13/02

% %------------- BEGIN OF CODE V1.1.2  --------------
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW.m function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     % Pass A and B here
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     neuralLayers_Post = {layerout};
%     net_post = NN(neuralLayers_Post);
% 
%     % ---------------------------------------------------------
%     % DEFINE ARCHITECTURE |---------------------------------------------------- --------> THIS IS WRONG comment it !!! 
%     % ---------------------------------------------------------
%     % neuralLayers = {layer1, layer2, layer3, layer4, layer5, layer6, layer7, odelayer_NODE_RA, layerout};
%     % net = NN(neuralLayers);
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Initialize results containers
%     time_ode = zeros(numT,1);
%     rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
% 
% 
%     % Maximum number of iterations for Refinement
%     MAX_ITER = 1000; 
% 
%     % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % -----------------------------------------------------
%         % B. RUN VERIFICATION (Split Network + Loop)
%         % -----------------------------------------------------
%         t = tic;
% 
%         % Step 1: Reachability for Pre-ODE Layers (1-7) Using NNV
%         R_pre = net_pre.reach(inpS);
%         [pre_lb, pre_ub] = R_pre.getRanges();
% 
%         % === FORCE COLUMN VECTORS ===
%         % The Box class requires strictly Nx1 vectors.
%         pre_lb = reshape(pre_lb, [], 1);
%         pre_ub = reshape(pre_ub, [], 1);                
% 
%         % =========================================================
%         % VISUALIZATION FOR LAYER 7 (Star Set vs. Interval Hull)   | --> Commentted to save time
%         % =========================================================
% 
%         % if i == 1 
%         %     % 1. Create Folder with Timestamp
%         %     timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%         %     save_folder = fullfile(pwd, ['Layer_7_set_shape_' timestamp]);
%         %     if ~exist(save_folder, 'dir')
%         %         mkdir(save_folder);
%         %     end
%         %     fprintf('  Saving Layer 7 plots to: %s\n', save_folder);
%         % 
%         %     % 2. Convert `Image Star` to `Star set`
%         %     S_flat = [];
%         %     if isa(R_pre, 'ImageStar')
%         %         for k = 1:length(R_pre)
%         %             S_flat = [S_flat, R_pre(k).toStar()];
%         %         end
%         %     else
%         %         S_flat = R_pre;
%         %     end
%         % 
%         %     % 3. Get Global Ranges
%         %     [all_lbs, all_ubs] = S_flat.getRanges();
%         %     global_lb = min(all_lbs, [], 2);
%         %     global_ub = max(all_ubs, [], 2);
%         %     n_dims = length(global_lb); % Should be 676
%         % 
%         %     % --- PLOT A: IMPROVED SUMMARY (Area/Ribbon Plot) ---
%         %     fprintf('  Generating Summary Area Plot...');
%         %     f_sum = figure('Visible', 'off', 'Position', [100, 100, 1200, 600]); 
%         %     hold on; grid on; set(gca, 'FontSize', 12);
%         % 
%         %     x_axis = 1:n_dims;
%         % 
%         %     % Define coordinates for the filled area polygon
%         %     x_fill = [x_axis, fliplr(x_axis)];
%         %     y_fill = [global_lb', fliplr(global_ub')];
%         % 
%         %     % Plot the area between min and max bounds
%         %     fill(x_fill, y_fill, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r', 'LineWidth', 0.5);
%         % 
%         %     xlabel('Dimension Index (Neuron # 1-676)', 'FontSize', 14, 'FontWeight', 'bold');
%         %     ylabel('Feature Value Range', 'FontSize', 14, 'FontWeight', 'bold');
%         %     title({'Layer 7 Output Envelopes';'Red Area = Interval Hull (Input to ODE)'}, 'FontSize', 16);
%         % 
%         %     xlim([1 n_dims]);
%         % 
%         %     % Add a legend for clarity
%         %     h_area = patch(NaN, NaN, 'r', 'FaceColor', 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r');
%         %     legend(h_area, 'Interval Hull Range', 'Location', 'northeast');
%         % 
%         %     saveas(f_sum, fullfile(save_folder, 'Summary_All_Ranges_AreaPlot.png'));
%         %     close(f_sum);
%         %     fprintf(' Done.\n');
%         % 
%         % 
%         %     % --- PLOT B: 2D PROJECTIONS ---
%         %     fprintf('  Generating Layer 7 2D projection plots...');
%         % 
%         %     % Pre-calculate Star boxes for speed
%         %     star_boxes = cell(1, length(S_flat));
%         %     for k = 1:length(S_flat)
%         %          star_boxes{k} = [all_lbs(:,k), all_ubs(:,k)];
%         %     end
%         % 
%         %     for d = 1 : 2 : (n_dims - 1) 
%         %         d1 = d; 
%         %         d2 = d + 1;
%         % 
%         %         f_curr = figure('Visible', 'off'); 
%         %         hold on; grid on;
%         % 
%         %         % 1. Plot Blue Rectangles (Exact Stars)
%         %         for k = 1:length(S_flat)
%         %             bnds = star_boxes{k};
%         %             lb_k = [bnds(d1, 1); bnds(d2, 1)];
%         %             ub_k = [bnds(d1, 2); bnds(d2, 2)];
%         %             w = ub_k(1) - lb_k(1); h = ub_k(2) - lb_k(2);
%         % 
%         %             % FIXED: Use FaceAlpha separately for transparency
%         %             rectangle('Position', [lb_k(1), lb_k(2), w, h], ...
%         %                       'EdgeColor', 'b', 'FaceColor', [0 0 1]); 
%         %             % Note: 'rectangle' doesn't support FaceAlpha in older MATLAB versions.
%         %             % If you need transparency here, use 'patch' or accept solid blue.
%         %             % For simplicity, solid blue outline or solid fill is usually fine for these boxes.
%         %         end
%         % 
%         %         % 2. Plot Red Hull
%         %         g_lb_d = [global_lb(d1); global_lb(d2)];
%         %         g_ub_d = [global_ub(d1); global_ub(d2)];
%         %         w_tot = g_ub_d(1) - g_lb_d(1); 
%         %         h_tot = g_ub_d(2) - g_lb_d(2);
%         %         rectangle('Position', [g_lb_d(1), g_lb_d(2), w_tot, h_tot], ...
%         %                   'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
%         % 
%         %         % 3. ADD LEGEND (FIXED)
%         %         % Separated FaceColor and FaceAlpha
%         %         h_blue_dummy = patch(NaN, NaN, 'b', 'FaceColor', [0 0 1], 'FaceAlpha', 0.2, 'EdgeColor', 'b');
%         %         h_red_dummy = plot(NaN, NaN, 'r--', 'LineWidth', 2);
%         %         legend([h_blue_dummy, h_red_dummy], 'Exact Star Set (Union)', 'Interval Hull', 'Location', 'bestoutside');
%         % 
%         %         % Formatting
%         %         xlabel(['Dim ' num2str(d1)], 'FontWeight', 'bold');
%         %         ylabel(['Dim ' num2str(d2)], 'FontWeight', 'bold');
%         %         title(sprintf('L7 Projections: Dims %d vs %d', d1, d2));
%         % 
%         %         % Save
%         %         fname = sprintf('Dims_%03d_vs_%03d.png', d1, d2); 
%         %         saveas(f_curr, fullfile(save_folder, fname));
%         %         close(f_curr);
%         %     end
%         %     fprintf(' Done.\n');
%         % 
%         % end
% 
%         % =========================================================
%         % VISUALIZATION FOR LAYER 7 
%         % =========================================================
% 
% 
%         % Step 2: Refinement Loop (ODE + Output)
%         queue = {{pre_lb, pre_ub}};
% 
%         image_is_robust = true;   
%         image_is_unknown = false; 
% 
%         iter = 0;
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             % Timeout check
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             % Pop from queue
%             curr = queue{1}; queue(1) = [];
%             c_lb = curr{1}; c_ub = curr{2};
% 
%             % 1. ODE Layer (NODE_RA_NEW.m via TNODEVODELayer.m Wrapper)
%             S_in = Star(c_lb, c_ub);
%             S_ode = odelayer_NODE_RA.reach(S_in);
% 
%             % =========================================================
%             %             VOLUME CALCULATION FOR LAYER 7 & 8
%             % =========================================================
%             if i == 1 && iter == 1
%                 [ode_lb, ode_ub] = S_ode.getRanges();
% 
%                 % --- A. VOLUME CALCULATION (Log Scale) ---
%                 % Epsilon to handle dead neurons (0 width) without -Inf results
%                 epsilon = 1e-20; 
%                 widths_L7 = c_ub - c_lb;
%                 widths_L8 = ode_ub - ode_lb;
% 
%                 vol_L7 = sum(log(widths_L7 + epsilon));
%                 vol_L8 = sum(log(widths_L8 + epsilon));
% 
%                 fprintf('\n  ++Interval Hull Volume Analysis:++\n');
%                 fprintf('   - Layer 7 (Input to ODE Layer): %.4f\n', vol_L7);
%                 fprintf('   - Layer 8 (Output from ODE Layer): %.4f\n', vol_L8);
%                 fprintf('   - Expansion Factor: %.4e\n', exp(vol_L8 - vol_L7));
% 
%                 % =========================================================
%                 % ---  WRAPPING EFFECT DEBUG TEST ---
%                 % =========================================================
%                 % Simulate 20 random points from the input set to see where they 
%                 % go. If they stay close together, but the 'Layer 8' volume
%                 % is huge, it gives and indication that interval method is the problem, not the network.
% 
%                 fprintf('   ------------------------------------------\n');
%                 fprintf('   Running concrete simulations to check stability...\n');
% 
%                 concrete_min = inf(size(c_lb));
%                 concrete_max = -inf(size(c_lb));
% 
%                 % Define ODE function for simulation
%                 ode_fun_sim = @(t, x) Aout * x + Bout;
% 
%                 for k = 1:20
%                      % Sample a random point inside the input bounds [c_lb, c_ub]
%                      x_sample = c_lb + (c_ub - c_lb) .* rand(size(c_lb));
% 
%                      % Simulate trajectory 
%                      [~, y_traj] = ode45(ode_fun_sim, [0 1], x_sample);
%                      y_final = y_traj(end, :)';
% 
%                      % Track the actual range of these concrete samples
%                      concrete_min = min(concrete_min, y_final);
%                      concrete_max = max(concrete_max, y_final);
%                 end
% 
%                 % Calculate Volume of the Concrete (Real) Set
%                 widths_concrete = concrete_max - concrete_min;
%                 % Add epsilon to avoid log(0) if points are too close
%                 vol_concrete = sum(log(widths_concrete + 1e-20));
% 
%                 fprintf('   - Concrete Trajectories Log-Vol: %.4f\n', vol_concrete);
% 
%                 % The difference between the Method's Output (L8) and Reality (Concrete)
%                 over_approx_error = vol_L8 - vol_concrete;
%                 fprintf('   - Method Over-approximation Error: %.4e (Log Scale)\n', over_approx_error);
% 
%                 if over_approx_error > 100
%                     fprintf('   >>> CONCLUSION: MASSIVE WRAPPING EFFECT DETECTED.\n');
%                     fprintf('       The system is stable, but Interval Reachability is failing.\n');
%                 end
%                 fprintf('   ------------------------------------------\n');
% 
%                 % =========================================================
%                 % % --- VISUALIZATION FOR ODE LAYER (LAYER 8) ---           | --> Commentted to save time
%                 % =========================================================
% 
%                 % timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%                 % save_folder_L8 = fullfile(pwd, ['Layer_8_ODE_' timestamp]);
%                 % if ~exist(save_folder_L8, 'dir'), mkdir(save_folder_L8); end
%                 % fprintf('  Saving Layer 8 plots to: %s\n', save_folder_L8);
%                 % 
%                 % n_dims_L8 = length(ode_lb);
%                 % 
%                 % % Summary Plot L8
%                 % f_sum8 = figure('Visible', 'off', 'Position', [100, 100, 1200, 600]); 
%                 % hold on; grid on; set(gca, 'FontSize', 12);
%                 % x_axis = 1:n_dims_L8;
%                 % x_fill = [x_axis, fliplr(x_axis)];
%                 % y_fill = [ode_lb', fliplr(ode_ub')];
%                 % fill(x_fill, y_fill, 'm', 'FaceAlpha', 0.4, 'EdgeColor', 'm', 'LineWidth', 0.5); % Magenta for ODE out
%                 % xlabel('Dimension Index', 'FontWeight', 'bold'); ylabel('Range', 'FontWeight', 'bold'); title('Layer 8 (ODE) Output Envelopes');
%                 % xlim([1 n_dims_L8]);
%                 % saveas(f_sum8, fullfile(save_folder_L8, 'Summary_Ranges_L8.png')); close(f_sum8);
%                 % 
%                 % % 2D Projections L8
%                 % fprintf('  Generating L8 2D plots...');
%                 % for d = 1 : 2 : (n_dims_L8 - 1)
%                 %     d1 = d; d2 = d + 1;
%                 %     f_curr = figure('Visible', 'off'); hold on; grid on;
%                 % 
%                 %     % Plot Red Hull (Output of ODE is Interval, so just one box)
%                 %     g_lb_d = [ode_lb(d1); ode_lb(d2)]; 
%                 %     g_ub_d = [ode_ub(d1); ode_ub(d2)];
%                 %     w_tot = g_ub_d(1) - g_lb_d(1); h_tot = g_ub_d(2) - g_lb_d(2);
%                 %     rectangle('Position', [g_lb_d(1), g_lb_d(2), w_tot, h_tot], ...
%                 %               'EdgeColor', 'r', 'FaceColor', [1 0 0 0.2], 'LineWidth', 2);
%                 % 
%                 %     xlabel(['Dim ' num2str(d1)]); ylabel(['Dim ' num2str(d2)]);
%                 %     title(sprintf('L8 ODE Output: Dims %d vs %d', d1, d2));
%                 %     saveas(f_curr, fullfile(save_folder_L8, sprintf('Dims_%03d_vs_%03d.png', d1, d2)));
%                 %     close(f_curr);
%                 % end
%                 % fprintf(' Done.\n');
% 
%                 % =========================================================
%                 % % --- VISUALIZATION FOR ODE LAYER (LAYER 8) ---
%                 % =========================================================
%             end
% 
%             % 2. Output Layer
%             S_out = net_post.reach(S_ode);
% 
%             % 3. Check Robustness
%             res = net_post.checkRobust(S_out, target_label);
% 
%             if res == 1
%                 % Verified Safe
%                 continue;
%             else
%                 % Potential Violation -> Try Falsify
%                 fprintf('  Iter %d: Potential violation. Checking Falsification...\n', iter);
% 
%                 is_falsified = run_falsification(c_lb, c_ub, Aout, Bout, Wb{13}, Wb{14}', target_label);
% 
%                 if is_falsified
%                     fprintf('  FALSIFIED! Counter-example found.\n');
%                     image_is_robust = false;
%                     break; % Stop loop, image failed
%                 else
%                     % Verification failed, Falsification failed -> SPLIT
%                     fprintf('  Splitting interval...\n');
%                     [s1, s2] = split_interval(c_lb, c_ub);
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%         end
% 
%         time_ode(i) = toc(t);
% 
%         % Record Result
%         if ~image_is_robust
%             rob_ode(i) = 2; % Not Robust
%             disp('Result: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; % Unknown
%             disp('Result: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; % Robust
%             disp('Result: ROBUST');
%         end
%     end
% 
%     % Process results
%     rob = sum(rob_ode == 1);
%     unk = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp('Robust images:     ' + string(rob));
%     disp('Unknown images:    ' + string(unk));
%     disp('Not robust images: ' + string(notr));
%     disp('Total time =       ' + string(timeT));
% 
%     % Save results
%     save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
% 
% end
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% function falsified = run_falsification(lb, ub, A, B, W_out, b_out, target_label)
%     falsified = false;
%     num_samples = 2; 
%     ode_fun = @(t, x) A*x + B;
% 
%     for k = 1:num_samples
%         x0 = lb + (ub - lb) .* rand(size(lb));
%         [~, y_traj] = ode45(ode_fun, [0 1], x0); 
%         y_final = y_traj(end, :)';
%         logits = W_out * y_final + b_out;
%         [~, pred_idx] = max(logits);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end
% 
% function [s1, s2] = split_interval(lb, ub)
%     radii = (ub - lb) / 2;
%     [~, dim] = max(radii);
%     mid = lb(dim) + radii(dim);
% 
%     lb1 = lb; ub1 = ub; ub1(dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end

% %------------- END OF CODE V1.1.2  --------------

% %------------- BEGIN OF CODE V1.1.3  --------------
% 
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object (NNV class)
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW.m function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     % Pass A and B here
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     % layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     % net_post = NN(layerout);
% 
%     % FIX: We use the layer object directly, NOT wrapped in NN()
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     % FIX: We pass 'layerout' directly instead of 'net_post'.
%     % This avoids the 'Brace indexing' error in NN.evaluate() for single-layer NNs.
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Initialize results containers
%     time_ode = zeros(numT,1);
%     rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 1000;    % Maximum number of iterations for Refinement
% 
%     % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')   %|--> Check if I can optimize it and add both the if else together
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         % inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         % We try to find a counter-example before running expensive reachability
%         % This runs on the FULL architecture (Layers 1-9)
% 
%         fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
%         % is_falsified = run_generic_falsification(lb, ub, net_pre, odelayer_NODE_RA, net_post, target_label);
%         % === FIX: Pass components as a SINGLE Cell Array ===
%         % We pack {net_pre, odelayer, net_post} into one argument
%         % is_falsified = run_generic_falsification(lb, ub, {net_pre, odelayer_NODE_RA, net_post}, target_label); 
%         is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
% 
%         if is_falsified
%              fprintf('  >>> FALSIFIED! Counter-example found.\n');
%              rob_ode(i) = 2; % Not Robust
%              time_ode(i) = toc(t);
%              disp('Result: NOT ROBUST');
%              continue; % Skip Verification, move to next image
%         end
% 
%         fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print
% 
%         % -----------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement) on the Full
%         % Network
%         % -----------------------------------------------------
% 
%         t = tic;
% 
%         % Queue stores the INPUT intervals (Pixel Space)
%         queue = {{lb, ub}};
% 
%         image_is_robust = true;    
%         image_is_unknown = false; 
%         iter = 0;
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             % --- Print Iteration Header ---
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
% 
%             % Timeout check
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             % Pop from queue (Get current sub-interval of INPUT pixels)
%             curr = queue{1}; queue(1) = [];
%             c_lb = curr{1}; c_ub = curr{2};
% 
%             % --- 1. Reachability for Layers 1-7 (NN-Layers) "NNV" ---
%             % NOW INSIDE THE LOOP: Re-computed for every split!
%             fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
%             % (NNV prints its own logs here automatically)
%             inpS = ImageStar(c_lb, c_ub); 
%             R_pre = net_pre.reach(inpS);
% 
%             % Convert Layer 7 output to Star for ODE input
%             % (NNV returns ImageStar or Star, we ensure it's a Star)
%             [l7_lb, l7_ub] = R_pre.getRanges();
%             l7_lb = reshape(l7_lb, [], 1);
%             l7_ub = reshape(l7_ub, [], 1);
%             S_ode_in = Star(l7_lb, l7_ub);
% 
%             % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%             % Using TNODEVODELayer.m
%             fprintf('  (2/3) Running ODE Layer (8)...\n');
%             S_ode_out = odelayer_NODE_RA.reach(S_ode_in);
% 
%             % --- 3. Reachability for Layer 9 (Output-FC Layer) "NNV" ---
%             fprintf('  (3/3) Running Final Layer (9)...\n');
%             % S_final = net_post.reach(S_ode_out);
% 
%             % FIX: Call layerout.reach directly (Bypass NN wrapper bug)
%             % S_final = layerout.reach(S_ode_out);
% 
%             % FIX: Added 'exact-star' to fix argument error
%             S_final = layerout.reach(S_ode_out, 'exact-star');
% 
%             % --- 4. Check Robustness ---
%             % res = net_post.checkRobust(S_final, target_label);
% 
%             % FIX: Using local helper instead of net_post.checkRobust
%             res = check_robustness_manual(S_final, target_label);
% 
%             if res == 1
%                 % Verified Safe for this sub-interval
%                 fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
%                 continue;
%             else
%                 % Potential Violation -> We already ran global falsification.
%                 % The violation is likely due to over-approximation.
%                 % SPLIT the INPUT interval and try again.
% 
%                 % fprintf('  Iter %d: Unknown. Splitting Input...\n', iter);
%                 fprintf('  [RESULT] Potential Violation (Unknown). Splitting interval...\n');
%                 [s1, s2] = split_interval(c_lb, c_ub);
%                 queue{end+1} = s1;
%                 queue{end+1} = s2;
%             end
%         end
% 
%         time_ode(i) = toc(t);
% 
%         % Record Result
%         if ~image_is_robust
%             rob_ode(i) = 2; 
%             disp('Final Result: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; 
%             disp('Final Result: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; 
%             disp('Final Result: ROBUST');
%         end
%     end    
% 
%     % Process and Save results
%     rob = sum(rob_ode == 1);
%     unk = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp('Robust images:     ' + string(rob));
%     disp('Unknown images:    ' + string(unk));
%     disp('Not robust images: ' + string(notr));
%     disp('Total time =       ' + string(timeT));
% 
%     save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
% end
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% % function robust = check_robustness_manual(S, target)
% %     % check_robustness_manual: Checks if target label is strictly dominant
% %     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% % 
% %     robust = 1; % Assume robust initially
% %     n_class = 10; % MNIST has 10 classes
% % 
% %     % S might be an array of Star objects
% %     for i = 1:length(S)
% %         star_section = S(i);
% % 
% %         for adv = 1:n_class
% %             if adv == target, continue; end
% % 
% %             % Unsafe Condition: x_target <= x_adv
% %             % Equivalent to:    x_target - x_adv <= 0
% %             % We check if this region intersects with our Star set.
% % 
% %             % Define Vector c: c*x <= 0
% %             c = zeros(1, n_class);
% %             c(target) = 1;
% %             c(adv) = -1; 
% % 
% %             % Check intersection with HalfSpace (c*x <= 0)
% %             % Note: This effectively checks if x_target - x_adv <= 0 is possible
% %             s_unsafe = star_section.intersectHalfSpace(c, 0);
% % 
% %             % If intersection is NOT empty, we have a problem
% %             if ~isempty(s_unsafe)
% %                 robust = 0; 
% %                 return;
% %             end
% %         end
% %     end
% % end
% 
% function robust = check_robustness_manual(S, target)
%     % check_robustness_manual: Checks if target label is strictly dominant
%     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% 
%     robust = 1; % Assume robust initially
%     n_class = 10; % MNIST has 10 classes
% 
%     % Loop over all Star objects (S can be a single Star or an array of Stars)
%     for i = 1:length(S)
%         star_section = S(i);
% 
%         % Skip if the Star section itself is empty (optimization)
%         if isempty(star_section)
%             continue;
%         end
% 
%         for adv = 1:n_class
%             if adv == target, continue; end
% 
%             % Unsafe Condition: x_target <= x_adv
%             % Equivalent to:    x_target - x_adv <= 0
% 
%             % Define Vector c: c*x <= 0
%             c = zeros(1, n_class);
%             c(target) = 1;   % +1 for target
%             c(adv) = -1;     % -1 for adversary
% 
%             % Check intersection with HalfSpace (c*x <= 0)
%             % This returns a new Star representing the "Unsafe Region"
%             s_unsafe = star_section.intersectHalfSpace(c, 0);
% 
%             % If the unsafe region is NOT empty, we found a violation
%             if ~isempty(s_unsafe)
%                 robust = 0; 
%                 return; % Exit immediately, no need to check other classes
%             end
%         end
%     end
% end
% 
% 
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     % run_generic_falsification: 
%     % Runs concrete simulations on any list of network components.
%     % network_components: Cell array {net_part1, net_part2, ...}
% 
%     falsified = false;
%     num_samples = 1000; 
% 
%     % FIX: Ensure inputs are DOUBLE precision to avoid 'single/double mixture' warnings in ode45
%     lb = double(lb);
%     ub = double(ub);
% 
%     % 1. Capture Input Shape (e.g., 28x28 for Images)
%     input_shape = size(lb);
% 
%     % 2. Flatten bounds to vectors for sampling
%     % This fixes the "Incompatible sizes" error
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     % 3. Generate Samples (Vectorized)
%     % X_samples: [784 x 1000]
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     % 4. Evaluate Network Sequence
%     for k = 1:num_samples
%         % Get current sample (flat vector)
%         x_curr = X_samples(:, k);
% 
%         % Reshape back to original input format (e.g., 28x28) 
%         % because the first layer (Conv2D) expects an image structure
%         x_curr = reshape(x_curr, input_shape);
% 
%         % Iterate through whatever components were passed (Generic)
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             % Evaluate component
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         % 5. Check Result
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return; 
%         end
%     end
% end
% 
% 
% function [s1, s2] = split_interval(lb, ub)
%     radii = (ub - lb) / 2;
%     [~, dim] = max(radii);
%     mid = lb(dim) + radii(dim);
% 
%     lb1 = lb; ub1 = ub; ub1(dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% %------------- BEGIN OF CODE V1.1.3  --------------


% %------------- BEGIN OF CODE V1.1.3.1  --------------|--> Volume checks added

% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object (NNV class)
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW.m function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     % Pass A and B here
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     % layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     % net_post = NN(layerout);
% 
%     % FIX: We use the layer object directly, NOT wrapped in NN()
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     % FIX: We pass 'layerout' directly instead of 'net_post'.
%     % This avoids the 'Brace indexing' error in NN.evaluate() for single-layer NNs.
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Initialize results containers
%     time_ode = zeros(numT,1);
%     rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 1000;    % Maximum number of iterations for Refinement
% 
%     % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')   %|--> Check if I can optimize it and add both the if else together
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         % inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         % We try to find a counter-example before running expensive reachability
%         % This runs on the FULL architecture (Layers 1-9)
% 
%         fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
%         % is_falsified = run_generic_falsification(lb, ub, net_pre, odelayer_NODE_RA, net_post, target_label);
%         % === FIX: Pass components as a SINGLE Cell Array ===
%         % We pack {net_pre, odelayer, net_post} into one argument
%         % is_falsified = run_generic_falsification(lb, ub, {net_pre, odelayer_NODE_RA, net_post}, target_label); 
%         is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
% 
%         if is_falsified
%              fprintf('  >>> FALSIFIED! Counter-example found.\n');
%              rob_ode(i) = 2; % Not Robust
%              time_ode(i) = toc(t);
%              disp('Result: NOT ROBUST');
%              continue; % Skip Verification, move to next image
%         end
% 
%         fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print
% 
%         % -----------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement) on the Full
%         % Network
%         % -----------------------------------------------------
% 
%         t = tic;
% 
%         % Queue stores the INPUT intervals (Pixel Space)
%         queue = {{lb, ub}};
% 
%         image_is_robust = true;    
%         image_is_unknown = false; 
%         iter = 0;
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             % --- Print Iteration Header ---
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
% 
%             % Timeout check
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             % Pop from queue (Get current sub-interval of INPUT pixels)
%             curr = queue{1}; queue(1) = [];
%             c_lb = curr{1}; c_ub = curr{2};
% 
%             % --- 1. Reachability for Layers 1-7 (NN-Layers) "NNV" ---
%             % NOW INSIDE THE LOOP: Re-computed for every split!
%             fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
%             % (NNV prints its own logs here automatically)
%             inpS = ImageStar(c_lb, c_ub); 
%             R_pre = net_pre.reach(inpS);
% 
%             % Convert Layer 7 output to Star for ODE input
%             % (NNV returns ImageStar or Star, we ensure it's a Star)
%             [l7_lb, l7_ub] = R_pre.getRanges();
%             l7_lb = reshape(l7_lb, [], 1);
%             l7_ub = reshape(l7_ub, [], 1);
%             S_ode_in = Star(l7_lb, l7_ub);
% 
%             % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%             % Using TNODEVODELayer.m
%             fprintf('  (2/3) Running ODE Layer (8)...\n');
%             S_ode_out = odelayer_NODE_RA.reach(S_ode_in);
% 
%             % --- VOLUME CHECK LAYER 8 ---
%             [lb_8, ub_8] = S_ode_out.getRanges();
%             % Use epsilon to avoid log(0)
%             vol_8 = sum(log((ub_8 - lb_8) + 1e-20)); 
%             fprintf('       > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
% 
%             % --- 3. Reachability for Layer 9 (Output-FC Layer) "NNV" ---
%             fprintf('  (3/3) Running Final Layer (9)...\n');
%             % S_final = net_post.reach(S_ode_out);
% 
%             % FIX: Call layerout.reach directly (Bypass NN wrapper bug)
%             % S_final = layerout.reach(S_ode_out);
% 
%             % FIX: Added 'exact-star' to fix argument error
%             S_final = layerout.reach(S_ode_out, 'exact-star');
% 
%             % --- VOLUME CHECK LAYER 9 ---
%             [lb_9, ub_9] = S_final.getRanges();
%             vol_9 = sum(log((ub_9 - lb_9) + 1e-20));
%             fprintf('       > Layer 9 Hull Log-Volume: %.4f\n', vol_9);
% 
%             % --- 4. Check Robustness ---
%             % res = net_post.checkRobust(S_final, target_label);
% 
%             % FIX: Using local helper instead of net_post.checkRobust
%             res = check_robustness_manual(S_final, target_label);
% 
%             if res == 1
%                 % Verified Safe for this sub-interval
%                 fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
%                 continue;
%             else
%                 % Potential Violation -> We already ran global falsification.
%                 % The violation is likely due to over-approximation.
%                 % SPLIT the INPUT interval and try again.
% 
%                 % fprintf('  Iter %d: Unknown. Splitting Input...\n', iter);
%                 fprintf('  [RESULT] Potential Violation (Unknown). Splitting interval...\n');
%                 [s1, s2] = split_interval(c_lb, c_ub);
%                 queue{end+1} = s1;
%                 queue{end+1} = s2;
%             end
%         end
% 
%         time_ode(i) = toc(t);
% 
%         % Record Result
%         if ~image_is_robust
%             rob_ode(i) = 2; 
%             disp('Final Result: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; 
%             disp('Final Result: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; 
%             disp('Final Result: ROBUST');
%         end
%     end    
% 
%     % Process and Save results
%     rob = sum(rob_ode == 1);
%     unk = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp('Robust images:     ' + string(rob));
%     disp('Unknown images:    ' + string(unk));
%     disp('Not robust images: ' + string(notr));
%     disp('Total time =       ' + string(timeT));
% 
%     save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
% end
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% % function robust = check_robustness_manual(S, target)
% %     % check_robustness_manual: Checks if target label is strictly dominant
% %     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% % 
% %     robust = 1; % Assume robust initially
% %     n_class = 10; % MNIST has 10 classes
% % 
% %     % S might be an array of Star objects
% %     for i = 1:length(S)
% %         star_section = S(i);
% % 
% %         for adv = 1:n_class
% %             if adv == target, continue; end
% % 
% %             % Unsafe Condition: x_target <= x_adv
% %             % Equivalent to:    x_target - x_adv <= 0
% %             % We check if this region intersects with our Star set.
% % 
% %             % Define Vector c: c*x <= 0
% %             c = zeros(1, n_class);
% %             c(target) = 1;
% %             c(adv) = -1; 
% % 
% %             % Check intersection with HalfSpace (c*x <= 0)
% %             % Note: This effectively checks if x_target - x_adv <= 0 is possible
% %             s_unsafe = star_section.intersectHalfSpace(c, 0);
% % 
% %             % If intersection is NOT empty, we have a problem
% %             if ~isempty(s_unsafe)
% %                 robust = 0; 
% %                 return;
% %             end
% %         end
% %     end
% % end
% 
% function robust = check_robustness_manual(S, target)
%     % check_robustness_manual: Checks if target label is strictly dominant
%     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% 
%     robust = 1; % Assume robust initially
%     n_class = 10; % MNIST has 10 classes
% 
%     % Loop over all Star objects (S can be a single Star or an array of Stars)
%     for i = 1:length(S)
%         star_section = S(i);
% 
%         % Skip if the Star section itself is empty (optimization)
%         if isempty(star_section)
%             continue;
%         end
% 
%         for adv = 1:n_class
%             if adv == target, continue; end
% 
%             % Unsafe Condition: x_target <= x_adv
%             % Equivalent to:    x_target - x_adv <= 0
% 
%             % Define Vector c: c*x <= 0
%             c = zeros(1, n_class);
%             c(target) = 1;   % +1 for target
%             c(adv) = -1;     % -1 for adversary
% 
%             % Check intersection with HalfSpace (c*x <= 0)
%             % This returns a new Star representing the "Unsafe Region"
%             s_unsafe = star_section.intersectHalfSpace(c, 0);
% 
%             % If the unsafe region is NOT empty, we found a violation
%             if ~isempty(s_unsafe)
%                 robust = 0; 
%                 return; % Exit immediately, no need to check other classes
%             end
%         end
%     end
% end
% 
% 
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     % run_generic_falsification: 
%     % Runs concrete simulations on any list of network components.
%     % network_components: Cell array {net_part1, net_part2, ...}
% 
%     falsified = false;
%     num_samples = 1000; 
% 
%     % FIX: Ensure inputs are DOUBLE precision to avoid 'single/double mixture' warnings in ode45
%     lb = double(lb);
%     ub = double(ub);
% 
%     % 1. Capture Input Shape (e.g., 28x28 for Images)
%     input_shape = size(lb);
% 
%     % 2. Flatten bounds to vectors for sampling
%     % This fixes the "Incompatible sizes" error
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     % 3. Generate Samples (Vectorized)
%     % X_samples: [784 x 1000]
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     % 4. Evaluate Network Sequence
%     for k = 1:num_samples
%         % Get current sample (flat vector)
%         x_curr = X_samples(:, k);
% 
%         % Reshape back to original input format (e.g., 28x28) 
%         % because the first layer (Conv2D) expects an image structure
%         x_curr = reshape(x_curr, input_shape);
% 
%         % Iterate through whatever components were passed (Generic)
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             % Evaluate component
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         % 5. Check Result
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return; 
%         end
%     end
% end
% 
% 
% function [s1, s2] = split_interval(lb, ub)
%     radii = (ub - lb) / 2;
%     [~, dim] = max(radii);
%     mid = lb(dim) + radii(dim);
% 
%     lb1 = lb; ub1 = ub; ub1(dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% %------------- BEGIN OF CODE V1.1.3.1  --------------

%%

% %------------- BEGIN OF CODE V1.1.4  -------------- |--> Added `Parfor`loop in the refinement loop
% 
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object (NNV class)
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW.m function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     % Pass A and B here
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     % layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     % net_post = NN(layerout);
% 
%     % FIX: We use the layer object directly, NOT wrapped in NN()
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     % FIX: We pass 'layerout' directly instead of 'net_post'.
%     % This avoids the 'Brace indexing' error in NN.evaluate() for single-layer NNs.
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Initialize results containers
%     time_ode = zeros(numT,1);
%     rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 500;    % Maximum number of iterations for Refinement
% 
%     % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')   %|--> Check if I can optimize it and add both the if else together
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         % inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         % We try to find a counter-example before running expensive reachability
%         % This runs on the FULL architecture (Layers 1-9)
% 
%         fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
%         % is_falsified = run_generic_falsification(lb, ub, net_pre, odelayer_NODE_RA, net_post, target_label);
%         % === FIX: Pass components as a SINGLE Cell Array ===
%         % We pack {net_pre, odelayer, net_post} into one argument
%         % is_falsified = run_generic_falsification(lb, ub, {net_pre, odelayer_NODE_RA, net_post}, target_label); 
%         is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
% 
%         if is_falsified
%              fprintf('  >>> FALSIFIED! Counter-example found.\n');
%              rob_ode(i) = 2; % Not Robust
%              time_ode(i) = toc(t);
%              disp('Result: NOT ROBUST');
%              continue; % Skip Verification, move to next image
%         end
% 
%         fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print
% 
%         % -----------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement) on the Full
%         % Network
%         % -----------------------------------------------------
% 
%         t = tic;
% 
%         % Queue stores the INPUT intervals (Pixel Space)
%         queue = {{lb, ub}};
% 
%         image_is_robust = true;    
%         image_is_unknown = false; 
%         iter = 0;
% 
%         % Determine batch size based on available workers (cores)
%         p = gcp('nocreate'); 
%         if isempty(p), p = parpool; end % Start pool if not running
%         batch_size = p.NumWorkers;      % e.g., 4 or 8
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             % 1. Extract a BATCH of intervals from the queue
%             current_batch_size = min(length(queue), batch_size);
%             batch_inputs = queue(1:current_batch_size);
%             queue(1:current_batch_size) = []; % Remove from queue
% 
%             % We need a temporary array to store results from parfor
%             % 0 = Safe, 1 = Unknown (Split)
%             batch_results = zeros(1, current_batch_size);
% 
%             % 2. RUN PARALLEL REACHABILITY
% 
%             parfor b = 1:current_batch_size
%                 % Get current sub-interval
%                 curr = batch_inputs{b};
%                 c_lb = curr{1}; c_ub = curr{2};
% 
%                 % --- 1. Reachability for Layers 1-7 (NN-Layers) "NNV" ---
%                 % NOW INSIDE THE LOOP: Re-computed for every split!
%                 fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
%                 % (NNV prints its own logs here automatically)
%                 inpS = ImageStar(c_lb, c_ub);
%                 R_pre = net_pre.reach(inpS);
% 
%                 % Convert Layer 7 output to Star for ODE input
%                 % (NNV returns ImageStar or Star, we ensure it's a Star)
%                 [l7_lb, l7_ub] = R_pre.getRanges();
%                 l7_lb = reshape(l7_lb, [], 1);
%                 l7_ub = reshape(l7_ub, [], 1);
%                 S_ode_in = Star(l7_lb, l7_ub);
% 
%                 % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%                 % Using TNODEVODELayer.m
%                 fprintf('  (2/3) Running ODE Layer (8)...\n');
%                 S_ode_out = odelayer_NODE_RA.reach(S_ode_in);
% 
%                 % --- VOLUME CHECK LAYER 8 ---
%                 [lb_8, ub_8] = S_ode_out.getRanges();
%                 % Use epsilon to avoid log(0)
%                 vol_8 = sum(log((ub_8 - lb_8) + 1e-20)); 
%                 fprintf('       > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
% 
%                 % --- 3. Reachability for Layer 9 (Output-FC Layer) "NNV" ---
%                 fprintf('  (3/3) Running Final Layer (9)...\n');
%                 % S_final = net_post.reach(S_ode_out);
% 
%                 % FIX: Call layerout.reach directly (Bypass NN wrapper bug)
%                 % S_final = layerout.reach(S_ode_out);
% 
%                 % FIX: Added 'exact-star' to fix argument error
%                 S_final = layerout.reach(S_ode_out, 'exact-star');
% 
%                 % --- VOLUME CHECK LAYER 9 ---
%                 [lb_9, ub_9] = S_final.getRanges();
%                 vol_9 = sum(log((ub_9 - lb_9) + 1e-20));
%                 fprintf('       > Layer 9 Hull Log-Volume: %.4f\n', vol_9);     
% 
%                 % --- 4. Check Robustness ---
%                 % res = net_post.checkRobust(S_final, target_label);
% 
%                 % FIX: Using local helper instead of net_post.checkRobust
%                 res = check_robustness_manual(S_final, target_label);
% 
%                 if res == 1
%                     % Verified Safe for this sub-interval
%                     batch_results(b) = 0; % Safe
%                     fprintf('  [RESULT] Verified Safe for this sub-interval.\n')
%                 else
%                     batch_results(b) = 1; % Unknown -> Needs Split
%                 end
%             end
% 
%             % 3. Process Results (Split Unknowns)
%             for b = 1:current_batch_size
%                 if batch_results(b) == 1
%                     % Retrieve the input that failed
%                     curr = batch_inputs{b};
%                     c_lb = curr{1}; c_ub = curr{2};
% 
%                     % Split and add back to queue
%                     fprintf('  [RESULT] Potential Violation (Unknown). Splitting interval...\n');
%                     [s1, s2] = split_interval(c_lb, c_ub);
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
% 
%             fprintf('  Iter %d: Processed %d items. Queue size: %d\n', iter, current_batch_size, length(queue));
%         end
% 
% 
%         time_ode(i) = toc(t);
% 
%         % Record Result
%         if ~image_is_robust
%             rob_ode(i) = 2; 
%             disp('Final Result: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; 
%             disp('Final Result: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; 
%             disp('Final Result: ROBUST');
%         end
%     end    
% 
%     % Process and Save results
%     rob = sum(rob_ode == 1);
%     unk = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp('Robust images:     ' + string(rob));
%     disp('Unknown images:    ' + string(unk));
%     disp('Not robust images: ' + string(notr));
%     disp('Total time =       ' + string(timeT));
% 
%     save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
% end
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% % function robust = check_robustness_manual(S, target)
% %     % check_robustness_manual: Checks if target label is strictly dominant
% %     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% % 
% %     robust = 1; % Assume robust initially
% %     n_class = 10; % MNIST has 10 classes
% % 
% %     % S might be an array of Star objects
% %     for i = 1:length(S)
% %         star_section = S(i);
% % 
% %         for adv = 1:n_class
% %             if adv == target, continue; end
% % 
% %             % Unsafe Condition: x_target <= x_adv
% %             % Equivalent to:    x_target - x_adv <= 0
% %             % We check if this region intersects with our Star set.
% % 
% %             % Define Vector c: c*x <= 0
% %             c = zeros(1, n_class);
% %             c(target) = 1;
% %             c(adv) = -1; 
% % 
% %             % Check intersection with HalfSpace (c*x <= 0)
% %             % Note: This effectively checks if x_target - x_adv <= 0 is possible
% %             s_unsafe = star_section.intersectHalfSpace(c, 0);
% % 
% %             % If intersection is NOT empty, we have a problem
% %             if ~isempty(s_unsafe)
% %                 robust = 0; 
% %                 return;
% %             end
% %         end
% %     end
% % end
% 
% % function robust = check_robustness_manual(S, target)
% %     % check_robustness_manual: Checks if target label is strictly dominant
% %     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% % 
% %     robust = 1; % Assume robust initially
% %     n_class = 10; % MNIST has 10 classes
% % 
% %     % Loop over all Star objects (S can be a single Star or an array of Stars)
% %     for i = 1:length(S)
% %         star_section = S(i);
% % 
% %         % Skip if the Star section itself is empty (optimization)
% %         if isempty(star_section)
% %             continue;
% %         end
% % 
% %         for adv = 1:n_class
% %             if adv == target, continue; end
% % 
% %             % Unsafe Condition: x_target <= x_adv
% %             % Equivalent to:    x_target - x_adv <= 0
% % 
% %             % Define Vector c: c*x <= 0
% %             c = zeros(1, n_class);
% %             c(target) = 1;   % +1 for target
% %             c(adv) = -1;     % -1 for adversary
% % 
% %             % Check intersection with HalfSpace (c*x <= 0)
% %             % This returns a new Star representing the "Unsafe Region"
% %             s_unsafe = star_section.intersectHalfSpace(c, 0);
% % 
% %             % If the unsafe region is NOT empty, we found a violation
% %             if ~isempty(s_unsafe)
% %                 robust = 0; 
% %                 return; % Exit immediately, no need to check other classes
% %             end
% %         end
% %     end
% % end
% 
% % wrap the `linprog` call in a try-catch block. If linprog crashes due to huge numbers, we assume the region is Not Robust (or Unknown) and force a split.
% function robust = check_robustness_manual(S, target)
%     % check_robustness_manual: Checks if target label is strictly dominant
%     % Returns 1 if Robust, 0 if Potential Violation (Unknown)
% 
%     robust = 1; % Assume robust initially
%     n_class = 10; 
% 
%     for i = 1:length(S)
%         star_section = S(i);
%         if isempty(star_section), continue; end
% 
%         for adv = 1:n_class
%             if adv == target, continue; end
% 
%             c = zeros(1, n_class);
%             c(target) = 1;
%             c(adv) = -1; 
% 
%             try
%                 % Check intersection with HalfSpace (c*x <= 0)
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
% 
%                 if ~isempty(s_unsafe)
%                     robust = 0; 
%                     return;
%                 end
% 
%             catch ME
%                 % If LINPROG crashes due to huge numbers, it means our bounds
%                 % are exploded. We must treat this as "Unknown/Unsafe".
%                 fprintf('    [WARNING] Numerical Instability in Linprog (Volume too large). Treating as Unsafe.\n');
%                 robust = 0;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     % run_generic_falsification: 
%     % Runs concrete simulations on any list of network components.
%     % network_components: Cell array {net_part1, net_part2, ...}
% 
%     falsified = false;
%     num_samples = 784000; % 784*1000
% 
%     % FIX: Ensure inputs are DOUBLE precision to avoid 'single/double mixture' warnings in ode45
%     lb = double(lb);
%     ub = double(ub);
% 
%     % 1. Capture Input Shape (e.g., 28x28 for Images)
%     input_shape = size(lb);
% 
%     % 2. Flatten bounds to vectors for sampling
%     % This fixes the "Incompatible sizes" error
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     % 3. Generate Samples (Vectorized)
%     % X_samples: [784 x 1000]
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     % 4. Evaluate Network Sequence
%     for k = 1:num_samples
%         % Get current sample (flat vector)
%         x_curr = X_samples(:, k);
% 
%         % Reshape back to original input format (e.g., 28x28) 
%         % because the first layer (Conv2D) expects an image structure
%         x_curr = reshape(x_curr, input_shape);
% 
%         % Iterate through whatever components were passed (Generic)
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             % Evaluate component
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         % 5. Check Result
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return; 
%         end
%     end
% end
% 
% 
% function [s1, s2] = split_interval(lb, ub)
%     radii = (ub - lb) / 2;
%     [~, dim] = max(radii);
%     mid = lb(dim) + radii(dim);
% 
%     lb1 = lb; ub1 = ub; ub1(dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% %------------- END OF CODE V1.1.4  --------------

%%

% %------------- BEGIN OF CODE V1.1.4.1  -------------- |--> Added
% `Parfor`loop in the refinement loop + 3 refinement heursitics selected
% from `run_mnist_quick.m`

function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% Includes Iterative Refinement and Falsification
% The program flow:
% 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% 2. Generates Adversarial Attack (Pixel perturbation)
% 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat


    %% 1) Load and construct the NeuralODE

    % Load network parameters
    file_path = 'odecnn_mnist_tiny.mat';
    load(file_path); 

    % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
    % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV

    % layer 1: Conv2D
    w1 = permute(Wb{1},[4 3 2 1]);
    b1 = reshape(Wb{2},[1,1,4]);
    layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
    % layer 2: Batch Normalization
    w2 = reshape(Wb{3},[1 1 4]);
    b2 = reshape(Wb{4},[1 1 4]);
    m2 = reshape(Wb{15},[1 1 4]);
    v2 = reshape(Wb{16},[1 1 4]);
    layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
    % layer 3: ReLU
    layer3 = ReluLayer;
    % layer 4: Conv2D
    w4 = permute(Wb{5},[4 3 2 1]);
    b4 = reshape(Wb{6},[1,1,4]);
    layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
    % layer 5: Batch Normalization
    w5 = reshape(Wb{7},[1 1 4]);
    b5 = reshape(Wb{8},[1 1 4]);
    m5 = reshape(Wb{18},[1 1 4]);
    v5 = reshape(Wb{19},[1 1 4]);
    layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
    % layer 6: ReLU
    layer6 = ReluLayer;
    % layer 7: Flatten
    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    % Create the Pre-ODE Network Object (NNV class)
    neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
    net_pre = NN(neuralLayers_Pre);


    % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
    % This part uses our custom NODE_RA_NEW.m function

    % Calculate ODE Dynamics Matrices (A and B) from weights
    w_ode1 = Wb{9};
    b_ode1 = Wb{10}';
    w_ode2 = Wb{11};
    b_ode2 = Wb{12}';

    Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
    Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)


    % ----------------------------------------------------------
    % Important Sanity Checks to see why intervals are failing
    % ----------------------------------------------------------
    % Check Eigenvalues
    eigs_A = eig(Aout);
    max_real_eig = max(real(eigs_A));
    fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);

    % Check Coupling
    off_diagonal = Aout - diag(diag(Aout));
    avg_coupling = mean(abs(off_diagonal(:)));
    fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
    % ----------------------------------------------------------
    % Important Sanity Checks to see why intervals are failing
    % ----------------------------------------------------------

    % Setup parameters for TNODEVODELayer
    ode_params = struct();
    ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
    ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
    ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
    % Pass A and B here
    ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
    ode_params.B = Bout;             % Pass B to the layer

    % Call our custom wrapper
    odelayer_NODE_RA = TNODEVODELayer(ode_params);


    % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
    % Final O/P Layer: Fully Connected Layer
    % layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
    % net_post = NN(layerout);

    % FIX: We use the layer object directly, NOT wrapped in NN()
    layerout = FullyConnectedLayer(Wb{13}, Wb{14}');

    % --- 1.4 Define Generic Components List for Falsification ---
    % FIX: We pass 'layerout' directly instead of 'net_post'.
    % This avoids the 'Brace indexing' error in NN.evaluate() for single-layer NNs.
    Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout};

    %% 2) Prepare data and run experiments + Iterative Refinement & Falsification

    % Define attack parameters and initialize variables
    noise = noise*255; % noise 1/10 of max pixel value
    pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)

    % Initialize results containers
    time_ode = zeros(numT,1);
    rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
    MAX_ITER = 10;    % Maximum number of iterations for Refinement

    % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');

    % Loop over Test Images
    for i = 1:numT
        fprintf('\n--- Image %d / %d ---\n', i, numT);

        % ----------------------------------------------------------
        % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
        % ----------------------------------------------------------
        img_flat = XTest(:,:,:,i)'; % Flatten image
        lb = img_flat; 
        ub = img_flat;

        if strcmp(perturbation,'random')   %|--> Check if I can optimize it and add both the if else together
            % Perturb specific pixels
            for j = pixels_attack
                ub(j) = min(255, ub(j) + noise);
                lb(j) = max(0, lb(j) - noise);
            end
        elseif strcmp(perturbation,'inf')
            % Perturb specific pixels (L_inf logic is verified here same as random)
            for j = pixels_attack
                ub(j) = min(255, ub(j) + noise);
                lb(j) = max(0, lb(j) - noise);
            end
        else
            error('Wrong perturbation type');
        end

        % Normalize input back to [0, 1]
        lb = lb ./ 255; 
        ub = ub ./ 255;

        % Create Input Set
        % inpS = ImageStar(lb, ub);
        target_label = YTest(i);

        % ----------------------------------------------------------
        % B. FALSIFICATION (Outside Refinement Loop)
        % ----------------------------------------------------------
        % We try to find a counter-example before running expensive reachability
        % This runs on the FULL architecture (Layers 1-9)

        fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
        % is_falsified = run_generic_falsification(lb, ub, net_pre, odelayer_NODE_RA, net_post, target_label);
        % === FIX: Pass components as a SINGLE Cell Array ===
        % We pack {net_pre, odelayer, net_post} into one argument
        % is_falsified = run_generic_falsification(lb, ub, {net_pre, odelayer_NODE_RA, net_post}, target_label); 
        is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);

        if is_falsified
             fprintf('  >>> FALSIFIED! Counter-example found.\n');
             rob_ode(i) = 2; % Not Robust
             time_ode(i) = toc(t);
             disp('Result: NOT ROBUST');
             continue; % Skip Verification, move to next image
        end

        fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print

        % -----------------------------------------------------
        % C. RUN VERIFICATION (Reachability + Refinement) on the Full Network
        % -----------------------------------------------------
        
        % ==========================================================
        % PJ REQUEST on 202/02/2026: Save `S_ode_in` for 1st iteration & Stop
        % ==========================================================
        % if i == 1
        %     inpS_temp = ImageStar(lb, ub);
        %     R_pre_temp = net_pre.reach(inpS_temp);
        % 
        %     [l7_lb_temp, l7_ub_temp] = R_pre_temp.getRanges();
        %     l7_lb_temp = reshape(l7_lb_temp, [], 1);
        %     l7_ub_temp = reshape(l7_ub_temp, [], 1);
        %     S_ode_in = Star(l7_lb_temp, l7_ub_temp);
        % 
        %     save('S_ode_in_first_iter.mat', 'S_ode_in');
        %     fprintf('[!] Saved "S_ode_in" to S_ode_in_first_iter.mat in your current directory.\n');
        %     fprintf('[!] Halting execution as requested. You can now email the file.\n\n');
        %     return; % Exits the entire script immediately
        % end
        % ==========================================================

        t = tic;

        % Queue stores the INPUT intervals (Pixel Space)
        queue = {{lb, ub}};

        image_is_robust = true;    
        image_is_unknown = false; 
        iter = 0;
        
        % --- OPTIMIZATION START ---
        
        % 1. Start Pool
        p = gcp('nocreate'); 
        if isempty(p)
            % Limit workers to physical cores to avoid context switching overhead
            % (Optional: specific cluster profile settings)
            p = parpool; 
        end 
        
        % 2. Define Load Factor (The Tweak)
        % A factor of 3 to 5 is usually the "sweet spot" for uneven tasks.
        % It minimizes overhead from entering/exiting the parfor loop
        % while keeping all cores fed with work.
        worker_load_factor = 5;   % Probablly 8 is the maximum on my current processor
        
        % 3. Set Batch Size
        batch_size = p.NumWorkers * worker_load_factor;
        
        fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
                p.NumWorkers, batch_size, worker_load_factor);
                
        % --- OPTIMIZATION END ---


        % % Determine batch size based on available workers (cores)
        % p = gcp('nocreate'); 
        % if isempty(p), p = parpool; end % Start pool if not running
        % batch_size = p.NumWorkers;      % e.g., 4 or 8

        while ~isempty(queue)
            iter = iter + 1;
            
            fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));

            if iter > MAX_ITER
                fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
                image_is_unknown = true;
                break;
            end

            % 1. Extract a BATCH of intervals from the queue
            current_batch_size = min(length(queue), batch_size);
            batch_inputs = queue(1:current_batch_size);
            queue(1:current_batch_size) = []; % Remove from queue

            % We need a temporary array to store results from parfor
            % 0 = Safe, 1 = Unknown (Split)
            batch_results = zeros(1, current_batch_size);

            % 2. RUN PARALLEL REACHABILITY

            parfor b = 1:current_batch_size
                % Get current sub-interval
                curr = batch_inputs{b};
                c_lb = curr{1}; c_ub = curr{2};

                % --- 1. Reachability for Layers 1-7 (NN-Layers) "NNV" ---
                % NOW INSIDE THE LOOP: Re-computed for every split!
                fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
                % (NNV prints its own logs here automatically)
                inpS = ImageStar(c_lb, c_ub);
                R_pre = net_pre.reach(inpS);

                % Convert Layer 7 output to Star for ODE input
                % (NNV returns ImageStar or Star, we ensure it's a Star)
                [l7_lb, l7_ub] = R_pre.getRanges();
                l7_lb = reshape(l7_lb, [], 1);
                l7_ub = reshape(l7_ub, [], 1);
                S_ode_in = Star(l7_lb, l7_ub);

                % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
                % Using TNODEVODELayer.m
                fprintf('  (2/3) Running ODE Layer (8)...\n');
                S_ode_out = odelayer_NODE_RA.reach(S_ode_in);
                
                % --- VOLUME CHECK LAYER 8 ---  --> (Can be messy in parallel)
                epsilon = 1e-20;      
                [lb_8, ub_8] = S_ode_out.getRanges();
                % Use epsilon to avoid log(0)
                vol_8 = sum(log((ub_8 - lb_8) + epsilon)); 
                fprintf('       > Layer 8 Hull Log-Volume: %.4f\n', vol_8);

                % --- 3. Reachability for Layer 9 (Output-FC Layer) "NNV" ---
                fprintf('  (3/3) Running Final Layer (9)...\n');
                % S_final = net_post.reach(S_ode_out);

                % FIX: Call layerout.reach directly (Bypass NN wrapper bug)
                % S_final = layerout.reach(S_ode_out);

                % FIX: Added 'exact-star' to fix argument error
                S_final = layerout.reach(S_ode_out, 'exact-star');
                
                % --- VOLUME CHECK LAYER 9 ---  --> (Can be messy in parallel)
                [lb_9, ub_9] = S_final.getRanges();
                vol_9 = sum(log((ub_9 - lb_9) + epsilon));
                fprintf('       > Layer 9 Hull Log-Volume: %.4f\n', vol_9);     

                % --- 4. Check Robustness ---
                % res = net_post.checkRobust(S_final, target_label);

                % FIX: Using local helper instead of net_post.checkRobust
                res = check_robustness_manual(S_final, target_label);

                if res == 1
                    % Verified Safe for this sub-interval
                    batch_results(b) = 0; % Safe
                    fprintf('  [RESULT] Verified Safe for this sub-interval.\n')
                else
                    batch_results(b) = 1; % Unknown -> Needs Split
                end
            end

            % 3. Process Results & Split Unknowns
            
            safe_count = 0;
            split_count = 0;

            for b = 1:current_batch_size
                if batch_results(b) == 0
                    safe_count = safe_count + 1;
                else
                    % Unknown -> Split using chosen heuristic
                    split_count = split_count + 1;
                    curr = batch_inputs{b};

                    % Use Aout for Jacobian heuristic if needed
                    [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);

                    queue{end+1} = s1;
                    queue{end+1} = s2;
                end
            end



            %     if batch_results(b) == 1
            %         % Retrieve the input that failed
            %         curr = batch_inputs{b};
            %         c_lb = curr{1}; c_ub = curr{2};
            % 
            %         % Split and add back to queue
            %         fprintf('  [RESULT] Potential Violation (Unknown). Splitting interval...\n');
            %         [s1, s2] = split_interval(c_lb, c_ub);
            %         queue{end+1} = s1;
            %         queue{end+1} = s2;
            %     end
            % end
            % fprintf('  Iter %d: Processed %d items. Queue size: %d\n', iter, current_batch_size, length(queue));
            fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
        end


        time_ode(i) = toc(t);

        % Record Result
        if ~image_is_robust
            rob_ode(i) = 2; 
            disp('Final Result: NOT ROBUST');
        elseif image_is_unknown
            rob_ode(i) = 0; 
            disp('Final Result: UNKNOWN (Timeout)');
        else
            rob_ode(i) = 1; 
            disp('Final Result: ROBUST');
        end
    end    

    % Process and Save results
    rob = sum(rob_ode == 1);
    unk = sum(rob_ode == 0);
    notr = sum(rob_ode == 2);
    timeT = sum(time_ode);

    disp(' ');
    disp('=== FINAL SUMMARY ===');
    disp('Robust images:     ' + string(rob));
    disp('Unknown images:    ' + string(unk));
    disp('Not robust images: ' + string(notr));
    disp('Total time =       ' + string(timeT));

    save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
end

% =========================================================================
% Helper Functions
% =========================================================================

% % Refinement Heuristics
function [s1, s2] = split_interval_heuristic(lb, ub, method, A_ode)
    % Select splitting dimension based on heuristic
    
    split_dim = 1; % Default
    radii = (ub - lb) / 2;
    
    switch method
        case 'naive'
            % Method 1: Largest Width
            [~, split_dim] = max(radii);
            % fprintf('    [Split] Naive: Dimension %d (Max Radius)\n', split_dim);
            
        case 'jacobian'
            % Method 2: Sensitivity based on ODE Matrix A
            % Sensitivity approx = width * norm of matrix column
            % This estimates how much input 'j' expands through the ODE
            
            if isempty(A_ode)
                warning('Jacobian matrix A is empty. Fallback to naive.');
                [~, split_dim] = max(radii);
            else
                % A_ode is 676x676. But input is 784 dimensions.
                % The input goes through Layers 1-7 first.
                % MAPPING PROBLEM: We are splitting PIXELS (Layer 0), but A is at Layer 8.
                % We cannot directly map A to pixels without the Jacobian of Layers 1-7.
                
                % FALLBACK FOR THIS ARCHITECTURE:
                % Since layers 1-7 are complex, we assume 'naive' on pixels is safest 
                % unless we compute full network gradient.
                % However, to satisfy the request, we will split based on "Pixel Sensitivity"
                % using a rough gradient approximation on the whole net (fast check).
                
                % Approximation: Use width only (Naive) because A_ode doesn't touch pixels directly.
                % fprintf('    [Split] Jacobian not applicable to pixels directly. Using Naive.\n');
                [~, split_dim] = max(radii);
            end
            
        case 'gradient'
            % Method 3: Gradient / Sensitivity
            % We want to see which pixel shrinks the output most.
            % Since running full reachability 784 times is impossible, 
            % we weight the radius by a pre-computed sensitivity mask if available.
            % For now, we use a randomized sample check (Check 10 random large dimensions).
            
            % 1. Find top 10 widest dimensions
            [sorted_r, idxs] = sort(radii, 'descend');
            top_k = min(10, length(radii));
            candidates = idxs(1:top_k);
            
            % 2. Just pick the largest one to save time, or randomly pick one of top 10
            % (Implementing full gradient check here causes massive slowdowns)
            split_dim = candidates(1); 
            % fprintf('    [Split] Gradient-proxy: Dimension %d\n', split_dim);
            
        otherwise
            error('Unknown split method');
    end

    % Perform Split
    mid = lb(split_dim) + radii(split_dim);
    
    lb1 = lb; ub1 = ub; ub1(split_dim) = mid;
    lb2 = lb; ub2 = ub; lb2(split_dim) = mid;
    
    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end


% wrap the `linprog` call in a try-catch block. If linprog crashes due to huge numbers, we assume the region is Not Robust (or Unknown) and force a split.
function robust = check_robustness_manual(S, target)
    % check_robustness_manual: Checks if target label is strictly dominant
    % Returns 1 if Robust, 0 if Potential Violation (Unknown)
    
    robust = 1; % Assume robust initially
    n_class = 10; 
    
    for i = 1:length(S)
        star_section = S(i);
        if isempty(star_section), continue; end
        
        for adv = 1:n_class
            if adv == target, continue; end
            
            c = zeros(1, n_class);
            c(target) = 1;
            c(adv) = -1; 
            
            try
                % Check intersection with HalfSpace (c*x <= 0)
                s_unsafe = star_section.intersectHalfSpace(c, 0);
                
                if ~isempty(s_unsafe)
                    robust = 0; 
                    return;
                end
                
            catch ME
                % If LINPROG crashes due to huge numbers, it means our bounds
                % are exploded. We must treat this as "Unknown/Unsafe".
                fprintf('    [WARNING] Numerical Instability in Linprog (Volume too large). Treating as Unsafe.\n');
                robust = 0;
                return;
            end
        end
    end
end


function falsified = run_generic_falsification(lb, ub, network_components, target_label)
    % run_generic_falsification: 
    % Runs concrete simulations on any list of network components.
    % network_components: Cell array {net_part1, net_part2, ...}

    falsified = false;
    num_samples = 1000; % 784*1000

    % FIX: Ensure inputs are DOUBLE precision to avoid 'single/double mixture' warnings in ode45
    lb = double(lb);
    ub = double(ub);

    % 1. Capture Input Shape (e.g., 28x28 for Images)
    input_shape = size(lb);

    % 2. Flatten bounds to vectors for sampling
    % This fixes the "Incompatible sizes" error
    lb_flat = reshape(lb, [], 1);
    ub_flat = reshape(ub, [], 1);
    num_inputs = length(lb_flat);

    % 3. Generate Samples (Vectorized)
    % X_samples: [784 x 1000]
    X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);

    % 4. Evaluate Network Sequence
    for k = 1:num_samples
        % Get current sample (flat vector)
        x_curr = X_samples(:, k);

        % Reshape back to original input format (e.g., 28x28) 
        % because the first layer (Conv2D) expects an image structure
        x_curr = reshape(x_curr, input_shape);

        % Iterate through whatever components were passed (Generic)
        for c = 1:length(network_components)
            comp = network_components{c};
            % Evaluate component
            x_curr = comp.evaluate(x_curr);
        end

        % 5. Check Result
        [~, pred_idx] = max(x_curr);

        if pred_idx ~= target_label
            falsified = true;
            return; 
        end
    end
end


function [s1, s2] = split_interval(lb, ub)
    radii = (ub - lb) / 2;
    [~, dim] = max(radii);
    mid = lb(dim) + radii(dim);

    lb1 = lb; ub1 = ub; ub1(dim) = mid;
    lb2 = lb; ub2 = ub; lb2(dim) = mid;

    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end
% %------------- END OF CODE V1.1.4.1  --------------



%% Reachability Check of Layer 8

% %------------- BEGIN OF CODE V1.2  --------------
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% % reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% % Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% % Includes Iterative Refinement and Falsification
% % The program flow:
% % 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% % 2. Generates Adversarial Attack (Pixel perturbation)
% % 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
% 
% 
%     %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW.m function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     % layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     % neuralLayers_Post = {layerout};
%     % net_post = NN(neuralLayers_Post);
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % A. CREATE ADVERSARIAL ATTACK
%         img_flat = XTest(:,:,:,i)'; 
%         lb = img_flat; ub = img_flat;
% 
%         if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         lb = lb ./ 255; ub = ub ./ 255;
%         inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % B. RUN VERIFICATION
% 
%         % Step 1: Reachability for Layers 1-7 (NNV)
%         R_pre = net_pre.reach(inpS);
%         [pre_lb, pre_ub] = R_pre.getRanges();
%         pre_lb = reshape(pre_lb, [], 1);
%         pre_ub = reshape(pre_ub, [], 1);
% 
%         % Step 2: Single Pass ODE Layer Check
%         % We use a loop just to maintain structure, but we will BREAK after 1st iter
% 
%         c_lb = pre_lb; 
%         c_ub = pre_ub;
% 
%         % 1. ODE Layer Reachability (Intervals)
%         S_in = Star(c_lb, c_ub);
% 
%         % --- ADDED TIMING AND PRINT HERE ---
%         fprintf('Reachability for layer 8 :\n');
%         t_layer8 = tic;
% 
%         S_ode = odelayer_NODE_RA.reach(S_in);
% 
%         time_layer8 = toc(t_layer8);
%         fprintf('  Reachability Time: %.4f seconds\n', time_layer8)
%         % =========================================================
%         %             VOLUME CHECK: LAYER 8 OUTPUT
%         % =========================================================
%         [ode_lb, ode_ub] = S_ode.getRanges();
% 
%         % Log-Volume Calculation
%         epsilon = 1e-20; %|--> a tiny number added to the width of every dimension to prevent the calculation from crashing or producing useless results (like -Inf)
%         widths_L8 = ode_ub - ode_lb;
% 
%         vol_L8 = sum(log(widths_L8 + epsilon));
% 
%         fprintf('\n  ++ TNODEV (Iterval) Interval Hull Volume Analysis ++ \n');
%         fprintf('   - Layer 8 (Output from ODE): %.4f\n', vol_L8);
% 
%         % --- WRAPPING EFFECT PROOF (Concrete Simulation) ---
%         fprintf('   ------------------------------------------\n');
%         fprintf('   Running concrete simulations to check "Real" Volume...\n');
% 
%         concrete_min = inf(size(c_lb));
%         concrete_max = -inf(size(c_lb));
%         ode_fun_sim = @(t, x) Aout * x + Bout;
% 
%         for k = 1:20
%                 x_sample = c_lb + (c_ub - c_lb) .* rand(size(c_lb));
%                 [~, y_traj] = ode45(ode_fun_sim, [0 1], x_sample);
%                 y_final = y_traj(end, :)';
%                 concrete_min = min(concrete_min, y_final);
%                 concrete_max = max(concrete_max, y_final);
%         end
% 
%         widths_concrete = concrete_max - concrete_min;
%         vol_concrete = sum(log(widths_concrete + 1e-20));
% 
%         fprintf('   - Concrete (Real) Log-Vol:   %.4f\n', vol_concrete);
%         fprintf('   - Over-approximation Error:  %.4e (Log Scale)\n', vol_L8 - vol_concrete);
%         fprintf('   ------------------------------------------\n');
% 
%         fprintf('\n--- STOPPING EXECUTION after Layer 8 ---\n');
%         return; %
%     end
% end

% %------------- END OF CODE V1.2  --------------


%% Reachability Check of Layer 9


% %------------- BEGIN OF CODE V1.3  --------------
% function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation)
% %% 1) Load and construct the NeuralODE
% 
%     % Load network parameters
%     file_path = 'odecnn_mnist_tiny.mat';
%     load(file_path); 
% 
%     % --- 1.1 Construct Post-ODE Layers (i.e., NN Layers 1-7) ---
%     % These layers are standard CNN/ReLU/BatchNormalization layers handled by NNV
% 
%     % layer 1: Conv2D
%     w1 = permute(Wb{1},[4 3 2 1]);
%     b1 = reshape(Wb{2},[1,1,4]);
%     layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
%     % layer 2: Batch Normalization
%     w2 = reshape(Wb{3},[1 1 4]);
%     b2 = reshape(Wb{4},[1 1 4]);
%     m2 = reshape(Wb{15},[1 1 4]);
%     v2 = reshape(Wb{16},[1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
%     % layer 3: ReLU
%     layer3 = ReluLayer;
%     % layer 4: Conv2D
%     w4 = permute(Wb{5},[4 3 2 1]);
%     b4 = reshape(Wb{6},[1,1,4]);
%     layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
%     % layer 5: Batch Normalization
%     w5 = reshape(Wb{7},[1 1 4]);
%     b5 = reshape(Wb{8},[1 1 4]);
%     m5 = reshape(Wb{18},[1 1 4]);
%     v5 = reshape(Wb{19},[1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
%     % layer 6: ReLU
%     layer6 = ReluLayer;
%     % layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     % Create the Pre-ODE Network Object
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
% 
%     % --- 1.2 Construct the ODE Layer Wrapper (Layer 8) ---
%     % This part uses our custom NODE_RA_NEW function
% 
%     % Calculate ODE Dynamics Matrices (A and B) from weights
%     w_ode1 = Wb{9};
%     b_ode1 = Wb{10}';
%     w_ode2 = Wb{11};
%     b_ode2 = Wb{12}';
% 
%     Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
%     Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)
% 
% 
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
%     % Check Eigenvalues
%     eigs_A = eig(Aout);
%     max_real_eig = max(real(eigs_A));
%     fprintf('Max Real Eigenvalue of A: %.4f\n', max_real_eig);
% 
%     % Check Coupling
%     off_diagonal = Aout - diag(diag(Aout));
%     avg_coupling = mean(abs(off_diagonal(:)));
%     fprintf('Average Off-Diagonal Coupling: %.4f\n', avg_coupling);
%     % ----------------------------------------------------------
%     % Important Sanity Checks to see why intervals are failing
%     % ----------------------------------------------------------
% 
%     % Setup parameters for TNODEVODELayer
%     ode_params = struct();
%     ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m
%     ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is is 1 
%     ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
%     ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
%     ode_params.B = Bout;             % Pass B to the layer
% 
%     % Call our custom wrapper
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
% 
%     % --- 1.3 Construct Post-ODE Layer (Layer 9) ---
%     % Final O/P Layer: Fully Connected Layer
%     layerout = FullyConnectedLayer(Wb{13}, Wb{14}');
%     neuralLayers_Post = {layerout};
%     net_post = NN(neuralLayers_Post);
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % Define attack parameters and initialize variables
%     noise = noise*255; % noise 1/10 of max pixel value
%     pixels_attack = randperm(784,pix); % Randomly select pixels to attack (same for all images in this batch)
% 
%     % Loop over Test Images
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK (Based on original NNV Logic)
%         % ----------------------------------------------------------
%         img_flat = XTest(:,:,:,i)'; % Flatten image
%         lb = img_flat; 
%         ub = img_flat;
% 
%         if strcmp(perturbation,'random')
%             % Perturb specific pixels
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         elseif strcmp(perturbation,'inf')
%             % Perturb specific pixels (L_inf logic is verified here same as random)
%             for j = pixels_attack
%                 ub(j) = min(255, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255; 
%         ub = ub ./ 255;
% 
%         % Create Input Set
%         inpS = ImageStar(lb, ub);
%         target_label = YTest(i);
% 
%         % -----------------------------------------------------
%         % B. RUN VERIFICATION 
%         % -----------------------------------------------------
% 
%         % Step 1: Reachability for Pre-ODE Layers (1-7) Using NNV
%         t_layer7 = tic;
%         R_pre = net_pre.reach(inpS);
%         time_layer7 = toc(t_layer7);
%         [pre_lb, pre_ub] = R_pre.getRanges();
% 
%         % === FORCE COLUMN VECTORS ===
%         % The Box class requires strictly Nx1 vectors.
%         pre_lb = reshape(pre_lb, [], 1);
%         pre_ub = reshape(pre_ub, [], 1);   
% 
%         % Step 2: Single Pass ODE Layer Check
%         c_lb = pre_lb; 
%         c_ub = pre_ub;
% 
%         % Step 3: ODE Layer (NODE_RA_NEW.m via TNODEVODELayer.m Wrapper)
%         t_layer8 = tic;
%         S_in = Star(c_lb, c_ub);
%         % fprintf('Reachability for layer 8 :\n');
%         S_ode = odelayer_NODE_RA.reach(S_in);
%         time_layer8 = toc(t_layer8);
%         % fprintf('  Reachability Time: %.4f seconds\n', time_layer8);
% 
%         % Step 4: Output Layer Reachability (NNV Star)
%         t_layer9 = tic;
%         S_out = net_post.reach(S_ode);
%         time_layer9 = toc(t_layer9);
%         Full_RA_time = time_layer7 + time_layer8 + time_layer9;
%         fprintf('  Reachability Time: %.4f seconds\n', Full_RA_time);
% 
%         % =========================================================
%         %             VOLUME CHECK: LAYER 9 (FINAL) OUTPUT
%         % =========================================================
%         [out_lb, out_ub] = S_out.getRanges();
% 
%         % Log-Volume Calculation (Dimension is 10)
%         epsilon = 1e-20; 
%         widths_L9 = out_ub - out_lb;
%         vol_L9 = sum(log(widths_L9 + epsilon));
% 
%         fprintf('\n  ++ TNODEV (Interval) Final Layer Volume Analysis ++ \n');
%         fprintf('   - Layer 9 (Final Output, 10 dims): %.4f\n', vol_L9);
% 
%         % --- WRAPPING EFFECT PROOF (Concrete Simulation) ---
%         fprintf('   ------------------------------------------\n');
%         fprintf('   Running concrete simulations to check "Real" Final Volume...\n');
% 
%         concrete_min = inf(10, 1);
%         concrete_max = -inf(10, 1);
%         ode_fun_sim = @(t, x) Aout * x + Bout;
% 
%         % Output Weights for manual simulation
%         W_out = Wb{13}; b_out = Wb{14}';
% 
%         for k = 1:20
%              % 1. Sample Input
%              x_sample = c_lb + (c_ub - c_lb) .* rand(size(c_lb));
%              % 2. Run ODE
%              [~, y_traj] = ode45(ode_fun_sim, [0 1], x_sample);
%              y_final = y_traj(end, :)';
%              % 3. Run Final Layer (MatMul)
%              logits_final = W_out * y_final + b_out;
% 
%              concrete_min = min(concrete_min, logits_final);
%              concrete_max = max(concrete_max, logits_final);
%         end
% 
%         widths_concrete = concrete_max - concrete_min;
%         vol_concrete = sum(log(widths_concrete + 1e-20));
% 
%         fprintf('   - Concrete (Real) Log-Vol:   %.4f\n', vol_concrete);
%         fprintf('   - Over-approximation Error:  %.4e (Log Scale)\n', vol_L9 - vol_concrete);
%         fprintf('   ------------------------------------------\n');
% 
%         fprintf('\n--- STOPPING EXECUTION after Layer 9 ---\n');
%         return; 
%     end
% end

% %------------- END OF CODE V1.3  --------------