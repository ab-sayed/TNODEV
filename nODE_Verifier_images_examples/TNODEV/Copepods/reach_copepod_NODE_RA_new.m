% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 29 March 2026
% Last update: 
% Last revision: 


% function reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, perturbation, splitting_heuristic)
% % reach_copepod_NODE_RA_new - Robustness verification for Copepod Neural ODE
% % Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST example)
% %
% % Network:
% %   Pre-ODE:  Conv(3->4,s=1) -> BN -> ReLU -> Conv(4->4,s=2) -> BN -> ReLU -> Flatten
% %   ODE:      dx/dt = W2 * ReLU(W1*x + b1) + b2   [256-dim, 16-dim bottleneck]
% %   Post-ODE: FC(256->64) -> ReLU -> FC(64->7)
% %
% % Input format matches run_mnist_quick.m:
% %   pix     - number of pixels to attack
% %   numT    - number of images
% %   noise   - L_inf radius (scaled by 255 internally)
% %   XTest   - test images [16, 16, 3, numT]
% %   YTest   - test labels [numT, 1] (1-indexed)
% %   perturbation - 'inf' for L_inf
% %   splitting_heuristic - 'naive', 'most-sensitive-input-radius', or 'interval-norm-gradient'
% 
%     %% 1) Load and construct the network
% 
%     file_path = 'odenet_tiny_copepod_weights.mat';
%     load(file_path);
% 
%     % --- 1.1 Pre-ODE Layers (1-7) ---
% 
%     % Layer 1: Conv2d(3->4, 3x3, stride=1, padding=1)
%     w1 = pre_conv1_W;  % [3, 3, 3, 4] already in MATLAB format
%     b1 = reshape(pre_conv1_b, [1, 1, 4]);
%     layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);
% 
%     % Layer 2: BatchNorm2d(4)
%     w2 = reshape(pre_bn1_weight, [1, 1, 4]);
%     b2 = reshape(pre_bn1_bias, [1, 1, 4]);
%     m2 = reshape(pre_bn1_running_mean, [1, 1, 4]);
%     v2 = reshape(pre_bn1_running_var, [1, 1, 4]);
%     layer2 = BatchNormalizationLayer('Offset', b2, 'Scale', w2, ...
%              'TrainedMean', m2, 'TrainedVariance', v2, ...
%              'NumChannels', 4, 'Epsilon', 4);
% 
%     % Layer 3: ReLU
%     layer3 = ReluLayer;
% 
%     % Layer 4: Conv2d(4->4, 3x3, stride=2, padding=1)
%     w4 = pre_conv2_W;  % [3, 3, 4, 4]
%     b4 = reshape(pre_conv2_b, [1, 1, 4]);
%     layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);
% 
%     % Layer 5: BatchNorm2d(4)
%     w5 = reshape(pre_bn2_weight, [1, 1, 4]);
%     b5 = reshape(pre_bn2_bias, [1, 1, 4]);
%     m5 = reshape(pre_bn2_running_mean, [1, 1, 4]);
%     v5 = reshape(pre_bn2_running_var, [1, 1, 4]);
%     layer5 = BatchNormalizationLayer('Offset', b5, 'Scale', w5, ...
%              'TrainedMean', m5, 'TrainedVariance', v5, ...
%              'NumChannels', 4, 'Epsilon', 4);
% 
%     % Layer 6: ReLU
%     layer6 = ReluLayer;
% 
%     % Layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
%     % --- 1.2 ODE Layer (Layer 8) ---
%     % Nonlinear dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
%     % System_description.m case 19
% 
%     global W1_ode b1_ode W2_ode b2_ode
%     W1_ode = ode_W1;          % 16 x 256
%     b1_ode = ode_b1';         % 16 x 1 (column vector)
%     W2_ode = ode_W2;          % 256 x 16
%     b2_ode = ode_b2';         % 256 x 1 (column vector)
% 
%     % Linearized A for splitting heuristic
%     Aout = W2_ode * W1_ode;   % 256 x 256 (rank-16)
% 
%     ode_params = struct();
%     ode_params.system_choice = 19;
%     ode_params.t_final = 1;
%     ode_params.timeStep = 20;
%     ode_params.W1 = W1_ode;
%     ode_params.b1 = b1_ode;
%     ode_params.W2 = W2_ode;
%     ode_params.b2 = b2_ode;
% 
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
%     % --- 1.3 Post-ODE Layers (9-11) ---
%     % FC(256->64) -> ReLU -> FC(64->7)
%     layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
%     layerout_relu = ReluLayer;
%     layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
%     % Full network components for reference
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};
% 
%     %% 2) Run verification
% 
%     noise = noise * 255;  % scale noise (same as MNIST example)
%     n_classes = 7;
%     total_pixels = 16 * 16 * 3;  % 768 total pixels in 16x16x3 image
%     pixels_attack = randperm(total_pixels, min(pix, total_pixels));
% 
%     time_ode  = zeros(numT, 1);
%     rob_ode   = zeros(numT, 1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER  = 1000;
% 
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % A. Get image and ground truth
%         img = XTest(:,:,:,i);             % [16, 16, 3]
%         target_label = YTest(i);          % 1-indexed (copepod = 2)
% 
%         fprintf('  True label: %d\n', target_label);
% 
%         % B. Build perturbed input bounds
%         img_lb = img;
%         img_ub = img;
% 
%         for p = 1:length(pixels_attack)
%             pidx = pixels_attack(p);
%             [row, col, ch] = ind2sub([16, 16, 3], pidx);
%             img_lb(row, col, ch) = max(0, img(row, col, ch) - noise);
%             img_ub(row, col, ch) = min(255, img(row, col, ch) + noise);
%         end
% 
%         % C. Iterative refinement verification
%         t = tic;
%         queue_lb = {img_lb};
%         queue_ub = {img_ub};
%         image_is_robust = true;
%         image_is_unknown = false;
%         total_verified = 0;
%         total_split = 0;
% 
%         for iter = 1:MAX_ITER
%             if isempty(queue_lb)
%                 break;  % all sub-intervals verified
%             end
% 
%             new_queue_lb = {};
%             new_queue_ub = {};
% 
%             for q = 1:length(queue_lb)
%                 c_lb = queue_lb{q};
%                 c_ub = queue_ub{q};
% 
%                 try
%                     % 1. Pre-ODE Layers (1-7) via NNV
%                     inpS = ImageStar(c_lb, c_ub);
%                     R_pre = net_pre.reach(inpS);
% 
%                     % 2. ODE Layer (8) via TNODEV
%                     S_ode_out = odelayer_NODE_RA.reach(R_pre);
% 
%                     % 3. Post-ODE Layers (9-11) via NNV
%                     S_fc1 = layerout_fc1.reach(S_ode_out, 'exact-star');
%                     S_relu = layerout_relu.reach(S_fc1, 'exact-star');
%                     S_final = layerout_fc2.reach(S_relu, 'exact-star');
% 
%                     % 4. Check robustness
%                     rob = check_robustness_copepod(S_final, target_label, n_classes);
% 
%                     if rob == 0
%                         % Verified safe for this sub-interval
%                         total_verified = total_verified + 1;
%                     elseif rob == 2
%                         % Counterexample found
%                         image_is_robust = false;
%                         fprintf('  NOT ROBUST (counterexample at iter %d)\n', iter);
%                         break;
%                     else
%                         % Unknown — need to split
%                         [s1_lb, s1_ub, s2_lb, s2_ub] = split_interval_copepod( ...
%                             c_lb, c_ub, splitting_heuristic, Aout);
%                         new_queue_lb{end+1} = s1_lb;
%                         new_queue_ub{end+1} = s1_ub;
%                         new_queue_lb{end+1} = s2_lb;
%                         new_queue_ub{end+1} = s2_ub;
%                         total_split = total_split + 1;
%                     end
% 
%                 catch ME
%                     fprintf('  ERROR: %s\n', ME.message);
%                     image_is_unknown = true;
%                     break;
%                 end
% 
%                 if ~image_is_robust
%                     break;
%                 end
%             end
% 
%             if ~image_is_robust || image_is_unknown
%                 break;
%             end
% 
%             queue_lb = new_queue_lb;
%             queue_ub = new_queue_ub;
% 
%             if mod(iter, 10) == 0
%                 fprintf('  Iter %d: verified=%d, split=%d, queue=%d\n', ...
%                     iter, total_verified, total_split, length(queue_lb));
%             end
% 
%             % Timeout: 10 min per image
%             if toc(t) > 600
%                 fprintf('  TIMEOUT at iter %d\n', iter);
%                 image_is_unknown = true;
%                 break;
%             end
%         end
% 
%         time_ode(i) = toc(t);
% 
%         if ~image_is_robust
%             rob_ode(i) = 2;
%             fprintf('Final Result: NOT ROBUST (%.2f s)\n', time_ode(i));
%         elseif image_is_unknown || ~isempty(queue_lb)
%             rob_ode(i) = 0;
%             fprintf('Final Result: UNKNOWN (%.2f s, %d remaining)\n', ...
%                 time_ode(i), length(queue_lb));
%         else
%             rob_ode(i) = 1;
%             fprintf('Final Result: ROBUST (%.2f s, %d verified, %d splits)\n', ...
%                 time_ode(i), total_verified, total_split);
%         end
%     end
% 
%     %% 3) Summary and Save
%     rob  = sum(rob_ode == 1);
%     unk  = sum(rob_ode == 0);
%     notr = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     fprintf('\n=== FINAL SUMMARY ===\n');
%     fprintf('Robust images:     %d\n', rob);
%     fprintf('Unknown images:    %d\n', unk);
%     fprintf('Not robust images: %d\n', notr);
%     fprintf('Total time =       %.2f s\n', timeT);
% 
%     save("copepod_NODE_RA_NEW_" + string(perturbation) + "_" + string(noise) + ".mat", ...
%          'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise');
% 
% end
% 
% 
% % =========================================================================
% % Helper: Robustness check
% % =========================================================================
% function robust = check_robustness_copepod(S, target, n_class)
%     robust = 0;  % 0 = Safe (verified robust)
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
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
%                 if ~isempty(s_unsafe)
%                     robust = 1;  % 1 = Unknown (potential violation)
%                     return;
%                 end
%             catch
%                 robust = 1;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% % =========================================================================
% % Helper: Splitting heuristic
% % =========================================================================
% function [s1_lb, s1_ub, s2_lb, s2_ub] = split_interval_copepod(lb, ub, method, Aout)
%     radii = (ub - lb) / 2;
% 
%     switch method
%         case 'naive'
%             % Split along dimension with largest radius
%             [~, split_dim] = max(radii(:));
% 
%         case 'most-sensitive-input-radius'
%             % Split along dimension where ODE is most sensitive
%             radii_flat = radii(:);
%             nonzero_dims = find(radii_flat > 0);
%             if isempty(nonzero_dims)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 % Use row norms of Aout weighted by input radii
%                 sensitivity = sum(abs(Aout), 1)';  % column sum = input sensitivity
%                 scores = sensitivity(nonzero_dims) .* radii_flat(nonzero_dims);
%                 [~, best] = max(scores);
%                 split_dim = nonzero_dims(best);
%             end
% 
%         case 'interval-norm-gradient'
%             % Split along dimension with largest gradient norm contribution
%             radii_flat = radii(:);
%             nonzero_dims = find(radii_flat > 0);
%             if isempty(nonzero_dims)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 % Use Frobenius norm of Jacobian columns weighted by radii
%                 jac_col_norms = sqrt(sum(Aout.^2, 1))';  % ||J(:,i)||_2
%                 scores = jac_col_norms(nonzero_dims) .* radii_flat(nonzero_dims);
%                 [~, best] = max(scores);
%                 split_dim = nonzero_dims(best);
%             end
% 
%         otherwise
%             warning('Unknown method "%s". Defaulting to naive.', method);
%             [~, split_dim] = max(radii(:));
%     end
% 
%     [row, col, ch] = ind2sub(size(lb), split_dim);
%     mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;
% 
%     s1_lb = lb; s1_ub = ub;
%     s2_lb = lb; s2_ub = ub;
% 
%     s1_ub(row, col, ch) = mid;
%     s2_lb(row, col, ch) = mid;
% end

%%

% % %------------- BEGIN OF CODE V1.0 -------------- | |--> Parallelized version
% % reach_copepod_NODE_RA_new.m
% % Robustness verification for Copepod Neural ODE classifier
% % Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST V1.1.4.3)
% %
% % Network:
% %   Pre-ODE (Layers 1-7): Conv(3->4,s=1)->BN->ReLU->Conv(4->4,s=2)->BN->ReLU->Flatten
% %   ODE (Layer 8):         dx/dt = W2 * ReLU(W1*x + b1) + b2  [256-dim, 16-dim bottleneck]
% %   Post-ODE (Layers 9-11): FC(256->64)->ReLU->FC(64->7)
% %
% % ODE state dim: 256 | Bottleneck: 16 | Classes: 7
% % Copepod = class 2 (1-indexed)
% 
% function reach_copepod_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% 
%     %% 1) Load and construct the NeuralODE
% 
%     file_path = 'odenet_tiny_copepod_weights.mat';
%     load(file_path);
% 
%     % Explicit local copies for parfor visibility
%     local_ode_W1 = ode_W1;
%     local_ode_b1 = ode_b1;
%     local_ode_W2 = ode_W2;
%     local_ode_b2 = ode_b2;
% 
%     % --- 1.1 Construct Pre-ODE Layers (Layers 1-7) ---
% 
%     % Layer 1: Conv2d(3->4, 3x3, stride=1, padding=1)
%     w1 = pre_conv1_W;  % [3, 3, 3, 4] already in MATLAB format
%     b1 = reshape(pre_conv1_b, [1, 1, 4]);
%     layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);
% 
%     % Layer 2: BatchNorm2d(4)
%     w2 = reshape(pre_bn1_weight, [1 1 4]);
%     b2 = reshape(pre_bn1_bias, [1 1 4]);
%     m2 = reshape(pre_bn1_running_mean, [1 1 4]);
%     v2 = reshape(pre_bn1_running_var, [1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
% 
%     % Layer 3: ReLU
%     layer3 = ReluLayer;
% 
%     % Layer 4: Conv2d(4->4, 3x3, stride=2, padding=1)
%     w4 = pre_conv2_W;  % [3, 3, 4, 4]
%     b4 = reshape(pre_conv2_b, [1, 1, 4]);
%     layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);
% 
%     % Layer 5: BatchNorm2d(4)
%     w5 = reshape(pre_bn2_weight, [1 1 4]);
%     b5 = reshape(pre_bn2_bias, [1 1 4]);
%     m5 = reshape(pre_bn2_running_mean, [1 1 4]);
%     v5 = reshape(pre_bn2_running_var, [1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
% 
%     % Layer 6: ReLU
%     layer6 = ReluLayer;
% 
%     % Layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
%     % --- 1.2 Construct ODE Layer Wrapper (Layer 8) ---
%     % Nonlinear dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
%     % System_description.m case 19
% 
%     global W1_ode b1_ode W2_ode b2_ode
%     W1_ode = ode_W1;          % 16 x 256
%     b1_ode = ode_b1';         % 16 x 1 (column vector)
%     W2_ode = ode_W2;          % 256 x 16
%     b2_ode = ode_b2';         % 256 x 1 (column vector)
% 
%     % Linearized A for splitting heuristic
%     Aout = W2_ode * W1_ode;   % 256 x 256 (rank-16)
% 
%     ode_params = struct();
%     ode_params.system_choice = 19;
%     ode_params.t_final = 1;
%     ode_params.timeStep = 20;        % dt = t_final/timeStep = 1/20 = 0.05
%     ode_params.W1 = W1_ode;
%     ode_params.b1 = b1_ode;
%     ode_params.W2 = W2_ode;
%     ode_params.b2 = b2_ode;
% 
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
%     % --- 1.3 Construct Post-ODE Layers (Layers 9-11) ---
%     % FC(256->64) -> ReLU -> FC(64->7)
%     layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
%     layerout_relu = ReluLayer;
%     layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     noise = noise * 255;
%     total_pixels = 16 * 16 * 3;  % 768 total pixels in 16x16x3
%     pixels_attack = randperm(total_pixels, min(pix, total_pixels));
% 
%     time_ode = zeros(numT, 1);
%     rob_ode = zeros(numT, 1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 1000;
%     n_class = 7;
% 
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK
%         % ----------------------------------------------------------
%         % img_flat = XTest(:,:,:,i)';
%         % img_flat = reshape(XTest(:,:,:,i), [], 1);
%         % lb = img_flat;
%         % ub = img_flat;
% 
%         img = XTest(:,:,:,i);  % [16, 16, 3]
%         lb = img;
%         ub = img;
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
%         % Normalize input back to [0, 1]
%         lb = lb ./ 255;
%         ub = ub ./ 255;
% 
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         fprintf('  Running Global Falsification (1000 samples)...\n');
% 
%         is_falsified = false;
% 
%         % is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
%         % 
%         % if is_falsified
%         %     fprintf('  >>> FALSIFIED! Counter-example found.\n');
%         %     rob_ode(i) = 2;
%         %     time_ode(i) = 0;
%         %     disp('Result: NOT ROBUST');
%         %     continue;
%         % end
%         % 
%         % fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n');
% 
%         % ----------------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement)
%         % ----------------------------------------------------------
%         t = tic;
% 
%         % queue = {{lb, ub}};
% 
%         image_is_robust = true;
%         image_is_unknown = false;
%         iter = 0;
% 
%         % --- PARALLELIZATION START ---
% 
%         p = gcp('nocreate');
%         if isempty(p)
%             p = parpool;
%         end
% 
%         worker_load_factor = 5;
%         batch_size = p.NumWorkers * worker_load_factor;
% 
%         fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%                 p.NumWorkers, batch_size, worker_load_factor);
% 
%         % --- PARALLELIZATION END ---
% 
%         % Pre-split to fill all workers from the start
%         queue = {{lb, ub}};
%         num_presplits = ceil(log2(p.NumWorkers));
%         for ps = 1:num_presplits
%             new_queue = {};
%             for qi = 1:length(queue)
%                 [s1, s2] = split_interval_heuristic(queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
%                 new_queue{end+1} = s1;
%                 new_queue{end+1} = s2;
%             end
%             queue = new_queue;
%         end
%         fprintf('  Pre-split into %d sub-intervals for %d workers.\n', length(queue), p.NumWorkers);
% 
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
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
%             queue(1:current_batch_size) = [];
% 
%             batch_results = zeros(1, current_batch_size);
% 
%             % 2. RUN PARALLEL REACHABILITY
% 
%             parfor b = 1:current_batch_size
%                 curr = batch_inputs{b};
%                 c_lb = curr{1}; c_ub = curr{2};
% 
%                 % --- 1. Reachability for Layers 1-7 (Pre-ODE) "NNV" ---
%                 fprintf('  (1/4) Running Pre-ODE Layers (1-7)...\n');
%                 inpS = ImageStar(c_lb, c_ub);
%                 R_pre = net_pre.reach(inpS);
% 
%                 % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%                 fprintf('  (2/4) Running ODE Layer (8)...\n');
%                 % init_worker_globals(ode_W1, ode_b1', ode_W2, ode_b2');
%                 init_worker_globals(local_ode_W1, local_ode_b1', local_ode_W2, local_ode_b2');
%                 S_ode_out = odelayer_NODE_RA.reach(R_pre);
% 
%                 % --- VOLUME CHECK LAYER 8 ---
%                 epsilon = 1e-20;
%                 lb_all_8 = []; ub_all_8 = [];
%                 for s_idx = 1:length(S_ode_out)
%                     B8 = S_ode_out(s_idx).getBoxFast();
%                     lb_all_8(:, s_idx) = B8.lb;
%                     ub_all_8(:, s_idx) = B8.ub;
%                 end
%                 lb_8 = min(lb_all_8, [], 2);
%                 ub_8 = max(ub_all_8, [], 2);
%                 vol_8 = sum(log((ub_8 - lb_8) + epsilon));
%                 fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
% 
%                 % --- 3. Reachability for Layers 9-11 (Post-ODE) "NNV" ---
%                 fprintf('  (3/4) Running Post-ODE Layers (9-11)...\n');
%                 S_fc1 = layerout_fc1.reach(S_ode_out, 'exact-star');
%                 S_relu = layerout_relu.reach(S_fc1, 'exact-star');
%                 S_final = layerout_fc2.reach(S_relu, 'exact-star');
% 
%                 % --- VOLUME CHECK LAYER 11 ---
%                 lb_all_11 = []; ub_all_11 = [];
%                 for s_idx = 1:length(S_final)
%                     B11 = S_final(s_idx).getBoxFast();
%                     lb_all_11(:, s_idx) = B11.lb;
%                     ub_all_11(:, s_idx) = B11.ub;
%                 end
%                 lb_11 = min(lb_all_11, [], 2);
%                 ub_11 = max(ub_all_11, [], 2);
%                 vol_11 = sum(log((ub_11 - lb_11) + epsilon));
%                 fprintf('        > Layer 11 Hull Log-Volume: %.4f\n', vol_11);
% 
%                 % --- 4. Check Robustness ---
%                 res = check_robustness_manual(S_final, target_label, n_class);
% 
%                 if res == 1
%                     batch_results(b) = 0;  % Safe
%                     fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
%                 else
%                     batch_results(b) = 1;  % Unknown -> Needs Split
%                 end
%             end
% 
%             % 3. Process Results & Split Unknowns
% 
%             safe_count = 0;
%             split_count = 0;
% 
%             for b = 1:current_batch_size
%                 if batch_results(b) == 0
%                     safe_count = safe_count + 1;
%                 else
%                     split_count = split_count + 1;
%                     curr = batch_inputs{b};
% 
%                     [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);
% 
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%             fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
%         end
% 
%         time_ode(i) = toc(t);
% 
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
%     save("copepod_NODE_RA_NEW_" + string(perturbation) + "_" + string(noise) + ".mat", ...
%          'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise');
% 
% end
% 
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% function init_worker_globals(W1, b1, W2, b2)
%     global W1_ode b1_ode W2_ode b2_ode system_choice
%     W1_ode = W1;
%     b1_ode = b1;
%     W2_ode = W2;
%     b2_ode = b2;
%     system_choice = 19;
% end
% 
% % Refinement Heuristics
% function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)
% 
%     radii = (ub - lb) / 2;
% 
%     switch method
%         case 'naive'
%             % Method 1: Largest Width
%             [~, split_dim] = max(radii);
% 
%         case 'most-sensitive-input-radius'
%             % Method 2: ODE sensitivity weighted by input radius
%             nonzero = find(radii > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii);
%             else
%                 sensitivity = sum(abs(Aout), 1)';
%                 scores = sensitivity(nonzero) .* radii(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         case 'interval-norm-gradient'
%             % Method 3: Jacobian column norm weighted by radius
%             nonzero = find(radii > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii);
%             else
%                 jac_col_norms = sqrt(sum(Aout.^2, 1))';
%                 scores = jac_col_norms(nonzero) .* radii(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         otherwise
%             warning('Method "%s" not supported. Defaulting to naive.', method);
%             [~, split_dim] = max(radii);
%     end
% 
%     mid = lb(split_dim) + radii(split_dim);
% 
%     lb1 = lb; ub1 = ub; ub1(split_dim) = mid;
%     lb2 = lb; ub2 = ub; lb2(split_dim) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% 
% 
% % Robustness check via HalfSpace intersection
% function robust = check_robustness_manual(S, target, n_class)
%     robust = 1;  % Assume robust initially
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
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
%                 if ~isempty(s_unsafe)
%                     robust = 0;
%                     return;
%                 end
%             catch ME
%                 fprintf('    [WARNING] Numerical Instability in Linprog. Treating as Unsafe.\n');
%                 robust = 0;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% % Generic falsification via random sampling
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     falsified = false;
%     num_samples = 1000;
% 
%     lb = double(lb);
%     ub = double(ub);
% 
%     input_shape = size(lb);
% 
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     for k = 1:num_samples
%         x_curr = X_samples(:, k);
%         x_curr = reshape(x_curr, input_shape);
% 
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end
% 
% % %------------- END OF CODE V1.0 --------------

%%

% % %------------- BEGIN OF CODE V1.1 -------------- | |--> Parallelized version 
% % reach_copepod_NODE_RA_new.m
% % Robustness verification for Copepod Neural ODE classifier
% % Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST V1.1.4.3)
% %
% % Network:
% %   Pre-ODE (Layers 1-7): Conv(3->2,s=1)->BN->ReLU->Conv(2->2,s=2)->BN->ReLU->Flatten
% %   ODE (Layer 8):         dx/dt = W2 * ReLU(W1*x + b1) + b2  [32-dim, 8-dim bottleneck]
% %   Post-ODE (Layers 9-11): FC(32->64)->ReLU->FC(64->7)
% %
% % ODE state dim: 32 | Bottleneck: 8 | Classes: 7
% % Copepod = class 2 (1-indexed)
% 
% function reach_copepod_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% 
%     %% 1) Load and construct the NeuralODE
% 
%     file_path = 'odenet_tiny_copepod_weights.mat';
%     load(file_path);
% 
%     % Explicit local copies for parfor visibility
%     local_ode_W1 = ode_W1;
%     local_ode_b1 = ode_b1;
%     local_ode_W2 = ode_W2;
%     local_ode_b2 = ode_b2;
% 
%     % --- 1.1 Construct Pre-ODE Layers (Layers 1-7) ---
% 
%     % Layer 1: Conv2d(3->2, 3x3, stride=1, padding=1)
%     w1 = pre_conv1_W;  % [3, 3, 3, 2] already in MATLAB format
%     b1 = reshape(pre_conv1_b, [1, 1, 2]);
%     layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);
% 
%     % Layer 2: BatchNorm2d(2)
%     w2 = reshape(pre_bn1_weight, [1 1 2]);
%     b2 = reshape(pre_bn1_bias, [1 1 2]);
%     m2 = reshape(pre_bn1_running_mean, [1 1 2]);
%     v2 = reshape(pre_bn1_running_var, [1 1 2]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',2,'Epsilon',2);
% 
%     % Layer 3: ReLU
%     layer3 = ReluLayer;
% 
%     % Layer 4: Conv2d(2->2, 3x3, stride=2, padding=1)
%     w4 = pre_conv2_W;  % [3, 3, 2, 2]
%     b4 = reshape(pre_conv2_b, [1, 1, 2]);
%     layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);
% 
%     % Layer 5: BatchNorm2d(2)
%     w5 = reshape(pre_bn2_weight, [1 1 2]);
%     b5 = reshape(pre_bn2_bias, [1 1 2]);
%     m5 = reshape(pre_bn2_running_mean, [1 1 2]);
%     v5 = reshape(pre_bn2_running_var, [1 1 2]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',2,'Epsilon',2);
% 
%     % Layer 6: ReLU
%     layer6 = ReluLayer;
% 
%     % Layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
%     % --- 1.2 Construct ODE Layer Wrapper (Layer 8) ---
%     % Nonlinear dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
%     % System_description.m case 19
% 
%     global W1_ode b1_ode W2_ode b2_ode
%     W1_ode = ode_W1;          % 8 x 32
%     b1_ode = ode_b1';         % 8 x 1 (column vector)
%     W2_ode = ode_W2;          % 32 x 8
%     b2_ode = ode_b2';         % 32 x 1 (column vector)
% 
%     % Linearized A for splitting heuristic
%     Aout = W2_ode * W1_ode;   % 32 x 32 (rank-8)
% 
%     ode_params = struct();
%     ode_params.system_choice = 19;
%     ode_params.t_final = 1;
%     ode_params.timeStep = 20;        % dt = t_final/timeStep = 1/20 = 0.05
%     ode_params.W1 = W1_ode;
%     ode_params.b1 = b1_ode;
%     ode_params.W2 = W2_ode;
%     ode_params.b2 = b2_ode;
% 
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
%     % --- 1.3 Construct Post-ODE Layers (Layers 9-11) ---
%     % FC(32->64) -> ReLU -> FC(64->7)
%     layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
%     layerout_relu = ReluLayer;
%     layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % noise = noise * 255;
%     total_pixels = 8 * 8 * 3;  % 192 total pixels in 8x8x3
%     pixels_attack = randperm(total_pixels, min(pix, total_pixels));
% 
%     time_ode = zeros(numT, 1);
%     rob_ode = zeros(numT, 1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 1000;
%     n_class = 7;
% 
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK
%         % ----------------------------------------------------------
%         img = XTest(:,:,:,i);  % [8, 8, 3]
%         lb = img;
%         ub = img;
% 
%         % if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
%         %     for j = pixels_attack
%         %         ub(j) = min(255, ub(j) + noise);
%         %         lb(j) = max(0, lb(j) - noise);
%         %     end
%         % else
%         %     error('Wrong perturbation type');
%         % end
%         % 
%         % % Normalize input back to [0, 1]
%         % lb = lb ./ 255;
%         % ub = ub ./ 255;
% 
%         if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
%             for j = pixels_attack
%                 ub(j) = min(1, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         fprintf('  Running Global Falsification (1000 samples)...\n');
% 
%         % is_falsified = false; % To stop the falsification and pass directly to the iterative refinement step 
% 
%         is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
% 
%         if is_falsified
%             fprintf('  >>> FALSIFIED! Counter-example found.\n');
%             rob_ode(i) = 2;
%             time_ode(i) = 0;
%             disp('Result: NOT ROBUST');
%             continue;
%         end
% 
%         fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n');
% 
%         % ----------------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement)
%         % ----------------------------------------------------------
%         t = tic;
% 
%         image_is_robust = true;
%         image_is_unknown = false;
%         iter = 0;
% 
%         % --- PARALLELIZATION START ---
% 
%         p = gcp('nocreate');
%         if isempty(p)
%             p = parpool;
%         end
% 
%         worker_load_factor = 5;
%         batch_size = p.NumWorkers * worker_load_factor;
% 
%         fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%                 p.NumWorkers, batch_size, worker_load_factor);
% 
%         % --- PARALLELIZATION END ---
% 
%         % Pre-split to fill all workers from the start
%         queue = {{lb, ub}};
%         num_presplits = ceil(log2(p.NumWorkers));
%         for ps = 1:num_presplits
%             new_queue = {};
%             for qi = 1:length(queue)
%                 [s1, s2] = split_interval_heuristic(queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
%                 new_queue{end+1} = s1;
%                 new_queue{end+1} = s2;
%             end
%             queue = new_queue;
%         end
%         fprintf('  Pre-split into %d sub-intervals for %d workers.\n', length(queue), p.NumWorkers);
% 
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
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
%             queue(1:current_batch_size) = [];
% 
%             batch_results = zeros(1, current_batch_size);
% 
%             % 2. RUN PARALLEL REACHABILITY
% 
%             parfor b = 1:current_batch_size
%                 curr = batch_inputs{b};
%                 c_lb = curr{1}; c_ub = curr{2};
% 
%                 % --- 1. Reachability for Layers 1-7 (Pre-ODE) "NNV" ---
%                 fprintf('  (1/4) Running Pre-ODE Layers (1-7)...\n');
%                 inpS = ImageStar(c_lb, c_ub);
%                 R_pre = net_pre.reach(inpS);
% 
%                 % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%                 fprintf('  (2/4) Running ODE Layer (8)...\n');
%                 init_worker_globals(local_ode_W1, local_ode_b1', local_ode_W2, local_ode_b2');
%                 S_ode_out = odelayer_NODE_RA.reach(R_pre);
% 
%                 % --- VOLUME CHECK LAYER 8 ---
%                 epsilon = 1e-20;
%                 lb_all_8 = []; ub_all_8 = [];
%                 for s_idx = 1:length(S_ode_out)
%                     B8 = S_ode_out(s_idx).getBoxFast();
%                     lb_all_8(:, s_idx) = B8.lb;
%                     ub_all_8(:, s_idx) = B8.ub;
%                 end
%                 lb_8 = min(lb_all_8, [], 2);
%                 ub_8 = max(ub_all_8, [], 2);
%                 vol_8 = sum(log((ub_8 - lb_8) + epsilon));
%                 fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
% 
%                 % --- 3. Reachability for Layers 9-11 (Post-ODE) "NNV" ---
%                 fprintf('  (3/4) Running Post-ODE Layers (9-11)...\n');
%                 S_fc1 = layerout_fc1.reach(S_ode_out, 'exact-star');
%                 S_relu = layerout_relu.reach(S_fc1, 'exact-star');
%                 S_final = layerout_fc2.reach(S_relu, 'exact-star');
% 
%                 % --- VOLUME CHECK LAYER 11 ---
%                 lb_all_11 = []; ub_all_11 = [];
%                 for s_idx = 1:length(S_final)
%                     B11 = S_final(s_idx).getBoxFast();
%                     lb_all_11(:, s_idx) = B11.lb;
%                     ub_all_11(:, s_idx) = B11.ub;
%                 end
%                 lb_11 = min(lb_all_11, [], 2);
%                 ub_11 = max(ub_all_11, [], 2);
%                 vol_11 = sum(log((ub_11 - lb_11) + epsilon));
%                 fprintf('        > Layer 11 Hull Log-Volume: %.4f\n', vol_11);
% 
%                 % --- 4. Check Robustness ---
%                 res = check_robustness_manual(S_final, target_label, n_class);
% 
%                 if res == 1
%                     batch_results(b) = 0;  % Safe
%                     fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
%                 else
%                     batch_results(b) = 1;  % Unknown -> Needs Split
%                 end
%             end
% 
%             % 3. Process Results & Split Unknowns
% 
%             safe_count = 0;
%             split_count = 0;
% 
%             for b = 1:current_batch_size
%                 if batch_results(b) == 0
%                     safe_count = safe_count + 1;
%                 else
%                     split_count = split_count + 1;
%                     curr = batch_inputs{b};
% 
%                     [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);
% 
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%             fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
%         end
% 
%         time_ode(i) = toc(t);
% 
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
%     save("copepod_NODE_RA_NEW_" + string(perturbation) + "_" + string(noise) + ".mat", ...
%          'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise');
% 
% end
% 
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% function init_worker_globals(W1, b1, W2, b2)
%     global W1_ode b1_ode W2_ode b2_ode system_choice
%     W1_ode = W1;
%     b1_ode = b1;
%     W2_ode = W2;
%     b2_ode = b2;
%     system_choice = 19;
% end
% 
% % Refinement Heuristics
% function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)
% 
%     radii = (ub - lb) / 2;
% 
%     switch method
%         case 'naive'
%             % Method 1: Largest Width
%             [~, split_dim] = max(radii(:));
% 
%         case 'most-sensitive-input-radius'
%             % Method 2: ODE sensitivity weighted by input radius
%             radii_flat = radii(:);
%             nonzero = find(radii_flat > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 sensitivity = sum(abs(Aout), 1)';
%                 scores = sensitivity(nonzero) .* radii_flat(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         case 'interval-norm-gradient'
%             % Method 3: Jacobian column norm weighted by radius
%             radii_flat = radii(:);
%             nonzero = find(radii_flat > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 jac_col_norms = sqrt(sum(Aout.^2, 1))';
%                 scores = jac_col_norms(nonzero) .* radii_flat(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         otherwise
%             warning('Method "%s" not supported. Defaulting to naive.', method);
%             [~, split_dim] = max(radii(:));
%     end
% 
%     % Convert linear index back to subscripts for 3D image array
%     [row, col, ch] = ind2sub(size(lb), split_dim);
%     mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;
% 
%     lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
%     lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% 
% 
% % Robustness check via HalfSpace intersection
% function robust = check_robustness_manual(S, target, n_class)
%     robust = 1;  % Assume robust initially
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
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
%                 if ~isempty(s_unsafe)
%                     robust = 0;
%                     return;
%                 end
%             catch ME
%                 fprintf('    [WARNING] Numerical Instability in Linprog. Treating as Unsafe.\n');
%                 robust = 0;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% % Generic falsification via random sampling
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     falsified = false;
%     num_samples = 1000;
% 
%     lb = double(lb);
%     ub = double(ub);
% 
%     input_shape = size(lb);
% 
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     for k = 1:num_samples
%         x_curr = X_samples(:, k);
%         x_curr = reshape(x_curr, input_shape);
% 
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end
% 
% % %------------- END OF CODE V1.1 --------------

%% Correct ordering similar to PyTorch

% %-------------1st nODE Network `IMG_SIZE = 8`-------------% % 

% %------------- BEGIN OF CODE V1.2 -------------- | |--> Parallelized version 
% reach_copepod_NODE_RA_new.m
% Robustness verification for Copepod Neural ODE classifier
% Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST V1.1.4.3)
%
% Network:
%   Pre-ODE (Layers 1-7): Conv(3->2,s=1)->BN->ReLU->Conv(2->2,s=2)->BN->ReLU->Flatten
%   ODE (Layer 8):         dx/dt = W2 * ReLU(W1*x + b1) + b2  [32-dim, 8-dim bottleneck]
%   Post-ODE (Layers 9-11): FC(32->64)->ReLU->FC(64->7)
%
% ODE state dim: 32 | Bottleneck: 8 | Classes: 7
% Copepod = class 2 (1-indexed)

function reach_copepod_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)

    %% 1) Load and construct the NeuralODE

    file_path = 'odenet_tiny_copepod_weights.mat';
    load(file_path);

    % === FIX: Permute weights for PyTorch→MATLAB flatten order ===
    % PyTorch flattens [C,H,W] row-major; MATLAB flattens [H,W,C] column-major
    % Build permutation: perm(matlab_idx) = pytorch_idx
    H_flat = 4; W_flat = 4; C_flat = 2;  % spatial dims after stride-2 conv on 8x8
    n_flat = H_flat * W_flat * C_flat;    % 32
    perm = zeros(n_flat, 1);
    for c = 0:C_flat-1
        for w = 0:W_flat-1
            for h = 0:H_flat-1
                ml_idx = h + H_flat*w + H_flat*W_flat*c + 1;
                pt_idx = c*H_flat*W_flat + h*W_flat + w + 1;
                perm(ml_idx) = pt_idx;
            end
        end
    end

    % Permute ODE input weights (columns match MATLAB flatten order)
    ode_W1 = ode_W1(:, perm);
    % Permute ODE output weights (rows map back to MATLAB order)
    ode_W2 = ode_W2(perm, :);
    ode_b2 = ode_b2(perm);
    % Permute post-FC input weights (columns match MATLAB order from ODE)
    post_fc1_W = post_fc1_W(:, perm);

    fprintf('  Applied PyTorch→MATLAB flatten permutation (32-dim)\n');
    % === END FIX ===

    % Explicit local copies for parfor visibility
    local_ode_W1 = ode_W1;
    local_ode_b1 = ode_b1;
    local_ode_W2 = ode_W2;
    local_ode_b2 = ode_b2;

    % --- 1.1 Construct Pre-ODE Layers (Layers 1-7) ---

    % Layer 1: Conv2d(3->2, 3x3, stride=1, padding=1)
    w1 = pre_conv1_W;  % [3, 3, 3, 2] already in MATLAB format
    b1 = reshape(pre_conv1_b, [1, 1, 2]);
    layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);

    % Layer 2: BatchNorm2d(2)
    w2 = reshape(pre_bn1_weight, [1 1 2]);
    b2 = reshape(pre_bn1_bias, [1 1 2]);
    m2 = reshape(pre_bn1_running_mean, [1 1 2]);
    v2 = reshape(pre_bn1_running_var, [1 1 2]);
    % layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',2,'Epsilon',2);
    layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',2,'Epsilon',pre_bn1_eps);

    % Layer 3: ReLU
    layer3 = ReluLayer;

    % Layer 4: Conv2d(2->2, 3x3, stride=2, padding=1)
    w4 = pre_conv2_W;  % [3, 3, 2, 2]
    b4 = reshape(pre_conv2_b, [1, 1, 2]);
    layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);

    % Layer 5: BatchNorm2d(2)
    w5 = reshape(pre_bn2_weight, [1 1 2]);
    b5 = reshape(pre_bn2_bias, [1 1 2]);
    m5 = reshape(pre_bn2_running_mean, [1 1 2]);
    v5 = reshape(pre_bn2_running_var, [1 1 2]);
    % layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',2,'Epsilon',2);
    layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',2,'Epsilon',pre_bn2_eps);

    % Layer 6: ReLU
    layer6 = ReluLayer;

    % Layer 7: Flatten
    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
    net_pre = NN(neuralLayers_Pre);

    % --- 1.2 Construct ODE Layer Wrapper (Layer 8) ---
    % Nonlinear dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
    % System_description.m case 19

    global W1_ode b1_ode W2_ode b2_ode
    W1_ode = ode_W1;          % 8 x 32
    b1_ode = ode_b1';         % 8 x 1 (column vector)
    W2_ode = ode_W2;          % 32 x 8
    b2_ode = ode_b2';         % 32 x 1 (column vector)

    % Linearized A for splitting heuristic
    Aout = W2_ode * W1_ode;   % 32 x 32 (rank-8)

    ode_params = struct();
    ode_params.system_choice = 19;
    ode_params.t_final = 1;
    ode_params.timeStep = 20;        % dt = t_final/timeStep = 1/20 = 0.05
    ode_params.W1 = W1_ode;
    ode_params.b1 = b1_ode;
    ode_params.W2 = W2_ode;
    ode_params.b2 = b2_ode;

    odelayer_NODE_RA = TNODEVODELayer(ode_params);

    % --- 1.3 Construct Post-ODE Layers (Layers 9-11) ---
    % FC(32->64) -> ReLU -> FC(64->7)
    layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
    layerout_relu = ReluLayer;
    layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');

    % --- 1.4 Define Generic Components List for Falsification ---
    Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};

    %% 2) Prepare data and run experiments + Iterative Refinement & Falsification

    % noise = noise * 255;
    total_pixels = 8 * 8 * 3;  % 192 total pixels in 8x8x3
    pixels_attack = randperm(total_pixels, min(pix, total_pixels));

    time_ode = zeros(numT, 1);
    rob_ode = zeros(numT, 1);   % 1=Robust, 0=Unknown, 2=Not Robust
    MAX_ITER = 1000;
    n_class = 7;

    for i = 1:numT
        fprintf('\n--- Image %d / %d ---\n', i, numT);

        % ----------------------------------------------------------
        % A. CREATE ADVERSARIAL ATTACK
        % ----------------------------------------------------------
        img = XTest(:,:,:,i);  % [8, 8, 3]
        lb = img;
        ub = img;

        % if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
        %     for j = pixels_attack
        %         ub(j) = min(255, ub(j) + noise);
        %         lb(j) = max(0, lb(j) - noise);
        %     end
        % else
        %     error('Wrong perturbation type');
        % end
        % 
        % % Normalize input back to [0, 1]
        % lb = lb ./ 255;
        % ub = ub ./ 255;

        if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
            for j = pixels_attack
                ub(j) = min(1, ub(j) + noise);
                lb(j) = max(0, lb(j) - noise);
            end
        else
            error('Wrong perturbation type');
        end

        target_label = YTest(i);

        % ----------------------------------------------------------
        % B. FALSIFICATION (Outside Refinement Loop)
        % ----------------------------------------------------------
        fprintf('  Running Global Falsification (1000 samples)...\n');

        % is_falsified = false; % To stop the falsification and pass directly to the iterative refinement step 

        is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);

        if is_falsified
            fprintf('  >>> FALSIFIED! Counter-example found.\n');
            rob_ode(i) = 2;
            time_ode(i) = 0;
            disp('Result: NOT ROBUST');
            continue;
        end

        fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n');

        % ----------------------------------------------------------
        % C. RUN VERIFICATION (Reachability + Refinement)
        % ----------------------------------------------------------
        t = tic;

        image_is_robust = true;
        image_is_unknown = false;
        iter = 0;

        % --- PARALLELIZATION START ---

        p = gcp('nocreate');
        if isempty(p)
            p = parpool;
        end

        worker_load_factor = 5;
        batch_size = p.NumWorkers * worker_load_factor;

        fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
                p.NumWorkers, batch_size, worker_load_factor);

        % --- PARALLELIZATION END ---

        % Pre-split to fill all workers from the start
        queue = {{lb, ub}};
        num_presplits = ceil(log2(p.NumWorkers));
        for ps = 1:num_presplits
            new_queue = {};
            for qi = 1:length(queue)
                [s1, s2] = split_interval_heuristic(queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
                new_queue{end+1} = s1;
                new_queue{end+1} = s2;
            end
            queue = new_queue;
        end
        fprintf('  Pre-split into %d sub-intervals for %d workers.\n', length(queue), p.NumWorkers);


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
            queue(1:current_batch_size) = [];

            batch_results = zeros(1, current_batch_size);

            % 2. RUN PARALLEL REACHABILITY

            parfor b = 1:current_batch_size
                curr = batch_inputs{b};
                c_lb = curr{1}; c_ub = curr{2};

                % --- 1. Reachability for Layers 1-7 (Pre-ODE) "NNV" ---
                fprintf('  (1/4) Running Pre-ODE Layers (1-7)...\n');
                inpS = ImageStar(c_lb, c_ub);
                R_pre = net_pre.reach(inpS);

                % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
                fprintf('  (2/4) Running ODE Layer (8)...\n');
                init_worker_globals(local_ode_W1, local_ode_b1', local_ode_W2, local_ode_b2');
                S_ode_out = odelayer_NODE_RA.reach(R_pre);

                % --- VOLUME CHECK LAYER 8 ---
                epsilon = 1e-20;
                lb_all_8 = []; ub_all_8 = [];
                for s_idx = 1:length(S_ode_out)
                    B8 = S_ode_out(s_idx).getBoxFast();
                    lb_all_8(:, s_idx) = B8.lb;
                    ub_all_8(:, s_idx) = B8.ub;
                end
                lb_8 = min(lb_all_8, [], 2);
                ub_8 = max(ub_all_8, [], 2);
                vol_8 = sum(log((ub_8 - lb_8) + epsilon));
                fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);

                % --- 3. Reachability for Layers 9-11 (Post-ODE) "NNV" ---
                fprintf('  (3/4) Running Post-ODE Layers (9-11)...\n');
                S_fc1 = layerout_fc1.reach(S_ode_out, 'exact-star');
                S_relu = layerout_relu.reach(S_fc1, 'exact-star');
                S_final = layerout_fc2.reach(S_relu, 'exact-star');

                % --- VOLUME CHECK LAYER 11 ---
                lb_all_11 = []; ub_all_11 = [];
                for s_idx = 1:length(S_final)
                    B11 = S_final(s_idx).getBoxFast();
                    lb_all_11(:, s_idx) = B11.lb;
                    ub_all_11(:, s_idx) = B11.ub;
                end
                lb_11 = min(lb_all_11, [], 2);
                ub_11 = max(ub_all_11, [], 2);
                vol_11 = sum(log((ub_11 - lb_11) + epsilon));
                fprintf('        > Layer 11 Hull Log-Volume: %.4f\n', vol_11);

                % --- 4. Check Robustness ---
                res = check_robustness_manual(S_final, target_label, n_class);

                if res == 1
                    batch_results(b) = 0;  % Safe
                    fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
                else
                    batch_results(b) = 1;  % Unknown -> Needs Split
                end
            end

            % 3. Process Results & Split Unknowns

            safe_count = 0;
            split_count = 0;

            for b = 1:current_batch_size
                if batch_results(b) == 0
                    safe_count = safe_count + 1;
                else
                    split_count = split_count + 1;
                    curr = batch_inputs{b};

                    [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);

                    queue{end+1} = s1;
                    queue{end+1} = s2;
                end
            end
            fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
        end

        time_ode(i) = toc(t);

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

    save("copepod_NODE_RA_NEW_" + string(perturbation) + "_" + string(noise) + ".mat", ...
         'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise');

end


% =========================================================================
% Helper Functions
% =========================================================================

function init_worker_globals(W1, b1, W2, b2)
    global W1_ode b1_ode W2_ode b2_ode system_choice
    W1_ode = W1;
    b1_ode = b1;
    W2_ode = W2;
    b2_ode = b2;
    system_choice = 19;
    warning('off', 'MATLAB:odearguments:InconsistentDataType');
end

% Refinement Heuristics
function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)

    radii = (ub - lb) / 2;

    switch method
        case 'naive'
            % Method 1: Largest Width
            [~, split_dim] = max(radii(:));

        case 'most-sensitive-input-radius'
            % Method 2: ODE sensitivity weighted by input radius
            radii_flat = radii(:);
            nonzero = find(radii_flat > 0);
            if isempty(nonzero)
                [~, split_dim] = max(radii_flat);
            else
                sensitivity = sum(abs(Aout), 1)';
                scores = sensitivity(nonzero) .* radii_flat(nonzero);
                [~, best] = max(scores);
                split_dim = nonzero(best);
            end

        case 'interval-norm-gradient'
            % Method 3: Jacobian column norm weighted by radius
            radii_flat = radii(:);
            nonzero = find(radii_flat > 0);
            if isempty(nonzero)
                [~, split_dim] = max(radii_flat);
            else
                jac_col_norms = sqrt(sum(Aout.^2, 1))';
                scores = jac_col_norms(nonzero) .* radii_flat(nonzero);
                [~, best] = max(scores);
                split_dim = nonzero(best);
            end

        otherwise
            warning('Method "%s" not supported. Defaulting to naive.', method);
            [~, split_dim] = max(radii(:));
    end

    % Convert linear index back to subscripts for 3D image array
    [row, col, ch] = ind2sub(size(lb), split_dim);
    mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;

    lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
    lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;

    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end


% Robustness check via HalfSpace intersection
function robust = check_robustness_manual(S, target, n_class)
    robust = 1;  % Assume robust initially

    for i = 1:length(S)
        star_section = S(i);
        if isempty(star_section), continue; end

        for adv = 1:n_class
            if adv == target, continue; end

            c = zeros(1, n_class);
            c(target) = 1;
            c(adv) = -1;

            try
                s_unsafe = star_section.intersectHalfSpace(c, 0);
                if ~isempty(s_unsafe)
                    robust = 0;
                    return;
                end
            catch ME
                fprintf('    [WARNING] Numerical Instability in Linprog. Treating as Unsafe.\n');
                robust = 0;
                return;
            end
        end
    end
end


% Generic falsification via random sampling
function falsified = run_generic_falsification(lb, ub, network_components, target_label)
    falsified = false;
    num_samples = 1000;

    lb = double(lb);
    ub = double(ub);

    input_shape = size(lb);

    lb_flat = reshape(lb, [], 1);
    ub_flat = reshape(ub, [], 1);
    num_inputs = length(lb_flat);

    X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);

    for k = 1:num_samples
        x_curr = X_samples(:, k);
        x_curr = reshape(x_curr, input_shape);

        for c = 1:length(network_components)
            comp = network_components{c};
            x_curr = comp.evaluate(x_curr);
        end

        [~, pred_idx] = max(x_curr);

        if pred_idx ~= target_label
            falsified = true;
            return;
        end
    end
end

% % %------------- END OF CODE V1.2 --------------

%% Correct ordering similar to PyTorch

% %-------------2nd nODE Network `IMG_SIZE = 16`-------------% % 

% %------------- BEGIN OF CODE V1.3 -------------- | |--> Parallelized version 
% reach_copepod_NODE_RA_new.m
% Robustness verification for Copepod Neural ODE classifier
% Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST V1.1.4.3)
%
% Network:
%   Pre-ODE (Layers 1-7): Conv(3->2,s=1)->BN->ReLU->Conv(2->2,s=2)->BN->ReLU->Flatten
%   ODE (Layer 8):         dx/dt = W2 * ReLU(W1*x + b1) + b2  [32-dim, 8-dim bottleneck]
%   Post-ODE (Layers 9-11): FC(32->64)->ReLU->FC(64->7)
%
% ODE state dim: 32 | Bottleneck: 8 | Classes: 7
% Copepod = class 2 (1-indexed)

% function reach_copepod_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% 
%     %% 1) Load and construct the NeuralODE
% 
%     file_path = 'odenet_tiny_copepod_weights.mat';
%     load(file_path);
% 
%     % === FIX: Permute weights for PyTorch→MATLAB flatten order ===
%     % PyTorch flattens [C,H,W] row-major; MATLAB flattens [H,W,C] column-major
%     % Build permutation: perm(matlab_idx) = pytorch_idx
%     H_flat = 8; W_flat = 8; C_flat = 4;  % spatial dims after stride-2 conv on 16x16
%     n_flat = H_flat * W_flat * C_flat;    % 256
%     perm = zeros(n_flat, 1);
%     for c = 0:C_flat-1
%         for w = 0:W_flat-1
%             for h = 0:H_flat-1
%                 ml_idx = h + H_flat*w + H_flat*W_flat*c + 1;
%                 pt_idx = c*H_flat*W_flat + h*W_flat + w + 1;
%                 perm(ml_idx) = pt_idx;
%             end
%         end
%     end
% 
%     % Permute ODE input weights (columns match MATLAB flatten order)
%     ode_W1 = ode_W1(:, perm);
%     % Permute ODE output weights (rows map back to MATLAB order)
%     ode_W2 = ode_W2(perm, :);
%     ode_b2 = ode_b2(perm);
%     % Permute post-FC input weights (columns match MATLAB order from ODE)
%     post_fc1_W = post_fc1_W(:, perm);
% 
%     fprintf('  Applied PyTorch→MATLAB flatten permutation (256-dim)\n');
%     % === END FIX ===
% 
%     % Explicit local copies for parfor visibility
%     local_ode_W1 = ode_W1;
%     local_ode_b1 = ode_b1;
%     local_ode_W2 = ode_W2;
%     local_ode_b2 = ode_b2;
% 
%     % --- 1.1 Construct Pre-ODE Layers (Layers 1-7) ---
% 
%     % Layer 1: Conv2d(3->2, 3x3, stride=1, padding=1)
%     w1 = pre_conv1_W;  % [3, 3, 3, 2] already in MATLAB format
%     b1 = reshape(pre_conv1_b, [1, 1, 4]);
%     layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);
% 
%     % Layer 2: BatchNorm2d(2)
%     w2 = reshape(pre_bn1_weight, [1 1 4]);
%     b2 = reshape(pre_bn1_bias, [1 1 4]);
%     m2 = reshape(pre_bn1_running_mean, [1 1 4]);
%     v2 = reshape(pre_bn1_running_var, [1 1 4]);
%     layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',pre_bn1_eps);
% 
%     % Layer 3: ReLU
%     layer3 = ReluLayer;
% 
%     % Layer 4: Conv2d(2->2, 3x3, stride=2, padding=1)
%     w4 = pre_conv2_W;  % [3, 3, 2, 2]
%     b4 = reshape(pre_conv2_b, [1, 1, 4]);
%     layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);
% 
%     % Layer 5: BatchNorm2d(2)
%     w5 = reshape(pre_bn2_weight, [1 1 4]);
%     b5 = reshape(pre_bn2_bias, [1 1 4]);
%     m5 = reshape(pre_bn2_running_mean, [1 1 4]);
%     v5 = reshape(pre_bn2_running_var, [1 1 4]);
%     layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',pre_bn2_eps);
% 
%     % Layer 6: ReLU
%     layer6 = ReluLayer;
% 
%     % Layer 7: Flatten
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
%     net_pre = NN(neuralLayers_Pre);
% 
%     % --- 1.2 Construct ODE Layer Wrapper (Layer 8) ---
%     % Nonlinear dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2
%     % System_description.m case 19
% 
%     global W1_ode b1_ode W2_ode b2_ode
%     W1_ode = ode_W1;          % 8 x 32
%     b1_ode = ode_b1';         % 8 x 1 (column vector)
%     W2_ode = ode_W2;          % 32 x 8
%     b2_ode = ode_b2';         % 32 x 1 (column vector)
% 
%     % Linearized A for splitting heuristic
%     Aout = W2_ode * W1_ode;   % 32 x 32 (rank-8)
% 
%     ode_params = struct();
%     ode_params.system_choice = 19;
%     ode_params.t_final = 1;
%     ode_params.timeStep = 20;        % dt = t_final/timeStep = 1/20 = 0.05
%     ode_params.W1 = W1_ode;
%     ode_params.b1 = b1_ode;
%     ode_params.W2 = W2_ode;
%     ode_params.b2 = b2_ode;
% 
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
%     % --- 1.3 Construct Post-ODE Layers (Layers 9-11) ---
%     % FC(32->64) -> ReLU -> FC(64->7)
%     layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
%     layerout_relu = ReluLayer;
%     layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
%     % --- 1.4 Define Generic Components List for Falsification ---
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};
% 
%     %% 2) Prepare data and run experiments + Iterative Refinement & Falsification
% 
%     % noise = noise * 255;
%     total_pixels = 16 * 16 * 3;  % 768 total pixels in 16x16x3
%     pixels_attack = randperm(total_pixels, min(pix, total_pixels));
% 
%     time_ode = zeros(numT, 1);
%     rob_ode = zeros(numT, 1);   % 1=Robust, 0=Unknown, 2=Not Robust
%     MAX_ITER = 1000;
%     n_class = 7;
% 
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % ----------------------------------------------------------
%         % A. CREATE ADVERSARIAL ATTACK
%         % ----------------------------------------------------------
%         img = XTest(:,:,:,i);  % [8, 8, 3]
%         lb = img;
%         ub = img;
% 
%         % if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
%         %     for j = pixels_attack
%         %         ub(j) = min(255, ub(j) + noise);
%         %         lb(j) = max(0, lb(j) - noise);
%         %     end
%         % else
%         %     error('Wrong perturbation type');
%         % end
%         % 
%         % % Normalize input back to [0, 1]
%         % lb = lb ./ 255;
%         % ub = ub ./ 255;
% 
%         if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
%             for j = pixels_attack
%                 ub(j) = min(1, ub(j) + noise);
%                 lb(j) = max(0, lb(j) - noise);
%             end
%         else
%             error('Wrong perturbation type');
%         end
% 
%         target_label = YTest(i);
% 
%         % ----------------------------------------------------------
%         % B. FALSIFICATION (Outside Refinement Loop)
%         % ----------------------------------------------------------
%         fprintf('  Running Global Falsification (1000 samples)...\n');
% 
%         % is_falsified = false; % To stop the falsification and pass directly to the iterative refinement step 
% 
%         is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
% 
%         if is_falsified
%             fprintf('  >>> FALSIFIED! Counter-example found.\n');
%             rob_ode(i) = 2;
%             time_ode(i) = 0;
%             disp('Result: NOT ROBUST');
%             continue;
%         end
% 
%         fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n');
% 
%         % ----------------------------------------------------------
%         % C. RUN VERIFICATION (Reachability + Refinement)
%         % ----------------------------------------------------------
%         t = tic;
% 
%         image_is_robust = true;
%         image_is_unknown = false;
%         iter = 0;
% 
%         % --- PARALLELIZATION START ---
% 
%         p = gcp('nocreate');
%         if isempty(p)
%             p = parpool;
%         end
% 
%         worker_load_factor = 5;
%         batch_size = p.NumWorkers * worker_load_factor;
% 
%         fprintf('  [Parallel Config] Workers: %d | Batch Size: %d (Load Factor: %dx)\n', ...
%                 p.NumWorkers, batch_size, worker_load_factor);
% 
%         % --- PARALLELIZATION END ---
% 
%         % Pre-split to fill all workers from the start
%         queue = {{lb, ub}};
%         num_presplits = ceil(log2(p.NumWorkers));
%         for ps = 1:num_presplits
%             new_queue = {};
%             for qi = 1:length(queue)
%                 [s1, s2] = split_interval_heuristic(queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
%                 new_queue{end+1} = s1;
%                 new_queue{end+1} = s2;
%             end
%             queue = new_queue;
%         end
%         fprintf('  Pre-split into %d sub-intervals for %d workers.\n', length(queue), p.NumWorkers);
% 
% 
%         while ~isempty(queue)
%             iter = iter + 1;
% 
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
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
%             queue(1:current_batch_size) = [];
% 
%             batch_results = zeros(1, current_batch_size);
% 
%             % 2. RUN PARALLEL REACHABILITY
% 
%             parfor b = 1:current_batch_size
%                 curr = batch_inputs{b};
%                 c_lb = curr{1}; c_ub = curr{2};
% 
%                 % --- 1. Reachability for Layers 1-7 (Pre-ODE) "NNV" ---
%                 fprintf('  (1/4) Running Pre-ODE Layers (1-7)...\n');
%                 inpS = ImageStar(c_lb, c_ub);
%                 R_pre = net_pre.reach(inpS);
% 
%                 % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
%                 fprintf('  (2/4) Running ODE Layer (8)...\n');
%                 init_worker_globals(local_ode_W1, local_ode_b1', local_ode_W2, local_ode_b2');
%                 S_ode_out = odelayer_NODE_RA.reach(R_pre);
% 
%                 % --- VOLUME CHECK LAYER 8 ---
%                 epsilon = 1e-20;
%                 lb_all_8 = []; ub_all_8 = [];
%                 for s_idx = 1:length(S_ode_out)
%                     B8 = S_ode_out(s_idx).getBoxFast();
%                     lb_all_8(:, s_idx) = B8.lb;
%                     ub_all_8(:, s_idx) = B8.ub;
%                 end
%                 lb_8 = min(lb_all_8, [], 2);
%                 ub_8 = max(ub_all_8, [], 2);
%                 vol_8 = sum(log((ub_8 - lb_8) + epsilon));
%                 fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
% 
%                 % --- 3. Reachability for Layers 9-11 (Post-ODE) "NNV" ---
%                 fprintf('  (3/4) Running Post-ODE Layers (9-11)...\n');
%                 S_fc1 = layerout_fc1.reach(S_ode_out, 'exact-star');
%                 S_relu = layerout_relu.reach(S_fc1, 'exact-star');
%                 S_final = layerout_fc2.reach(S_relu, 'exact-star');
% 
%                 % --- VOLUME CHECK LAYER 11 ---
%                 lb_all_11 = []; ub_all_11 = [];
%                 for s_idx = 1:length(S_final)
%                     B11 = S_final(s_idx).getBoxFast();
%                     lb_all_11(:, s_idx) = B11.lb;
%                     ub_all_11(:, s_idx) = B11.ub;
%                 end
%                 lb_11 = min(lb_all_11, [], 2);
%                 ub_11 = max(ub_all_11, [], 2);
%                 vol_11 = sum(log((ub_11 - lb_11) + epsilon));
%                 fprintf('        > Layer 11 Hull Log-Volume: %.4f\n', vol_11);
% 
%                 % --- 4. Check Robustness ---
%                 res = check_robustness_manual(S_final, target_label, n_class);
% 
%                 if res == 1
%                     batch_results(b) = 0;  % Safe
%                     fprintf('  [RESULT] Verified Safe for this sub-interval.\n');
%                 else
%                     batch_results(b) = 1;  % Unknown -> Needs Split
%                 end
%             end
% 
%             % 3. Process Results & Split Unknowns
% 
%             safe_count = 0;
%             split_count = 0;
% 
%             for b = 1:current_batch_size
%                 if batch_results(b) == 0
%                     safe_count = safe_count + 1;
%                 else
%                     split_count = split_count + 1;
%                     curr = batch_inputs{b};
% 
%                     [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);
% 
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%             fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
%         end
% 
%         time_ode(i) = toc(t);
% 
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
%     save("copepod_NODE_RA_NEW_" + string(perturbation) + "_" + string(noise) + ".mat", ...
%          'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise');
% 
% end
% 
% 
% % =========================================================================
% % Helper Functions
% % =========================================================================
% 
% function init_worker_globals(W1, b1, W2, b2)
%     global W1_ode b1_ode W2_ode b2_ode system_choice
%     W1_ode = W1;
%     b1_ode = b1;
%     W2_ode = W2;
%     b2_ode = b2;
%     system_choice = 19;
% end
% 
% % Refinement Heuristics
% function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)
% 
%     radii = (ub - lb) / 2;
% 
%     switch method
%         case 'naive'
%             % Method 1: Largest Width
%             [~, split_dim] = max(radii(:));
% 
%         case 'most-sensitive-input-radius'
%             % Method 2: ODE sensitivity weighted by input radius
%             radii_flat = radii(:);
%             nonzero = find(radii_flat > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 sensitivity = sum(abs(Aout), 1)';
%                 scores = sensitivity(nonzero) .* radii_flat(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         case 'interval-norm-gradient'
%             % Method 3: Jacobian column norm weighted by radius
%             radii_flat = radii(:);
%             nonzero = find(radii_flat > 0);
%             if isempty(nonzero)
%                 [~, split_dim] = max(radii_flat);
%             else
%                 jac_col_norms = sqrt(sum(Aout.^2, 1))';
%                 scores = jac_col_norms(nonzero) .* radii_flat(nonzero);
%                 [~, best] = max(scores);
%                 split_dim = nonzero(best);
%             end
% 
%         otherwise
%             warning('Method "%s" not supported. Defaulting to naive.', method);
%             [~, split_dim] = max(radii(:));
%     end
% 
%     % Convert linear index back to subscripts for 3D image array
%     [row, col, ch] = ind2sub(size(lb), split_dim);
%     mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;
% 
%     lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
%     lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;
% 
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% 
% 
% % Robustness check via HalfSpace intersection
% function robust = check_robustness_manual(S, target, n_class)
%     robust = 1;  % Assume robust initially
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
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
%                 if ~isempty(s_unsafe)
%                     robust = 0;
%                     return;
%                 end
%             catch ME
%                 fprintf('    [WARNING] Numerical Instability in Linprog. Treating as Unsafe.\n');
%                 robust = 0;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% % Generic falsification via random sampling
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     falsified = false;
%     num_samples = 1000;
% 
%     lb = double(lb);
%     ub = double(ub);
% 
%     input_shape = size(lb);
% 
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
% 
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     for k = 1:num_samples
%         x_curr = X_samples(:, k);
%         x_curr = reshape(x_curr, input_shape);
% 
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             x_curr = comp.evaluate(x_curr);
%         end
% 
%         [~, pred_idx] = max(x_curr);
% 
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end

% %------------- END OF CODE V1.3 --------------