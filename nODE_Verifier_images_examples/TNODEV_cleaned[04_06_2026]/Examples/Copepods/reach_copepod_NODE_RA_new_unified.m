% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 27 April 2026
% Last update: 
% Last revision: 

% % ------------- BEGIN reach_copepod_NODE_RA_new_unified.m ----------------
% % Unified reachability-based robustness verifier for the TNODEV pipeline.
% %
% % Replaces:
% %   reach_copepod_NODE_RA_new_tanh.m  (tanh, n_filters=2, n_class=7 hardcoded)
% %   reach_copepod_NODE_RA_new.m       (relu version)
% %
% % Reads architecture metadata from the weights .mat:
% %   * activation             -> 'tanh' (case 20) or 'relu' (case 19)
% %   * n_filters              -> 2 or 4 (or any other)
% %   * num_classes            -> 2 (binary) or 7 (full)
% %   * img_size               -> 8 or 16
% %   * spatial_after_downsample -> img_size / 2
% %
% % Network structure (unchanged from before, but parameterized):
% %   Pre-ODE:   Conv(3->nf,s=1) -> BN -> Act -> Conv(nf->nf,s=2) -> BN -> Act -> Flatten
% %   ODE:       dx/dt = W2 * act(W1*x + b1) + b2     [state_dim = nf*4*4 = 32 or 64]
% %   Post-ODE:  FC(state_dim->64) -> Act -> FC(64->n_class)
% %
% % Inputs:
% %   weights_file        - path to weights .mat (e.g. 'odenet_..._weights.mat')
% %   pix, numT, noise    - verification config (as before)
% %   XTest, YTest        - test data (as before)
% %   perturbation        - 'inf' | 'random'
% %   splitting_heuristic - 'naive' | 'most-sensitive-input-radius'
% %                       | 'interval-norm-gradient'
% 
% function reach_copepod_NODE_RA_new_unified(weights_file, pix, numT, noise, ...
%                                     XTest, YTest, perturbation, ...
%                                     splitting_heuristic)
% 
%     %% 1) Load weights + introspect architecture
%     fprintf('  Loading weights: %s\n', weights_file);
%     W = load(weights_file);
% 
%     pre_conv1_W = W.pre_conv1_W;          pre_conv1_b = W.pre_conv1_b;
%     pre_bn1_weight = W.pre_bn1_weight;    pre_bn1_bias = W.pre_bn1_bias;
%     pre_bn1_running_mean = W.pre_bn1_running_mean;
%     pre_bn1_running_var  = W.pre_bn1_running_var;
%     pre_bn1_eps = W.pre_bn1_eps;
%     pre_conv2_W = W.pre_conv2_W;          pre_conv2_b = W.pre_conv2_b;
%     pre_bn2_weight = W.pre_bn2_weight;    pre_bn2_bias = W.pre_bn2_bias;
%     pre_bn2_running_mean = W.pre_bn2_running_mean;
%     pre_bn2_running_var  = W.pre_bn2_running_var;
%     pre_bn2_eps = W.pre_bn2_eps;
%     ode_W1 = W.ode_W1;   ode_b1 = W.ode_b1;
%     ode_W2 = W.ode_W2;   ode_b2 = W.ode_b2;
%     post_fc1_W = W.post_fc1_W;            post_fc1_b = W.post_fc1_b;
%     post_fc2_W = W.post_fc2_W;            post_fc2_b = W.post_fc2_b;
% 
%     % --- Architecture introspection (same logic as run_copepod_quick) ---
%     if isfield(W, 'n_filters'),   nf = double(W.n_filters);
%     else,                         nf = size(pre_conv1_W, 4); end
% 
%     if isfield(W, 'num_classes'), n_class = double(W.num_classes);
%     else,                         n_class = size(post_fc2_W, 1); end
% 
%     if isfield(W, 'img_size'),    img_size = double(W.img_size);
%     else,                         img_size = size(XTest, 1); end
% 
%     if isfield(W, 'spatial_after_downsample')
%         H_flat = double(W.spatial_after_downsample);
%     else
%         H_flat = img_size / 2;
%     end
%     W_flat = H_flat;
%     C_flat = nf;
%     n_flat = H_flat * W_flat * C_flat;
% 
%     if isfield(W, 'activation')
%         activation = char(string(W.activation));
%     else
%         activation = 'tanh';
%     end
%     activation = lower(strtrim(activation));
%     switch activation
%         case 'tanh', system_choice = 20;
%         case 'relu', system_choice = 19;
%         otherwise
%             error('reach_copepod_NODE_RA_new_unified:unknown_activation', ...
%                   'Unknown activation ''%s''', activation);
%     end
% 
%     fprintf('  Architecture: activation=%s (case %d) | nf=%d | img=%dx%d | flat=%d | n_class=%d\n', ...
%             activation, system_choice, nf, img_size, img_size, n_flat, n_class);
% 
%     %% 1.1) Permute weights for PyTorch -> MATLAB flatten order
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
%     ode_W1     = ode_W1(:, perm);
%     ode_W2     = ode_W2(perm, :);
%     ode_b2     = ode_b2(perm);
%     post_fc1_W = post_fc1_W(:, perm);
%     fprintf('  Applied PyTorch->MATLAB flatten permutation (%d-dim)\n', n_flat);
% 
%     % parfor-visible local copies
%     local_ode_W1 = ode_W1;
%     local_ode_b1 = ode_b1;
%     local_ode_W2 = ode_W2;
%     local_ode_b2 = ode_b2;
%     local_system_choice = system_choice;
% 
%     %% 1.2) Construct Pre-ODE network
%     b1_l = reshape(pre_conv1_b, [1, 1, nf]);
%     layer1 = Conv2DLayer(pre_conv1_W, b1_l, [1,1,1,1], [1,1], [1,1]);
% 
%     w2_l = reshape(pre_bn1_weight,       [1 1 nf]);
%     b2_l = reshape(pre_bn1_bias,         [1 1 nf]);
%     m2_l = reshape(pre_bn1_running_mean, [1 1 nf]);
%     v2_l = reshape(pre_bn1_running_var,  [1 1 nf]);
%     layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l, ...
%                                       'TrainedMean',m2_l,'TrainedVariance',v2_l, ...
%                                       'NumChannels',nf, ...
%                                       'Epsilon',pre_bn1_eps);
% 
%     layer3 = make_activation_layer(activation);
% 
%     b4_l = reshape(pre_conv2_b, [1, 1, nf]);
%     layer4 = Conv2DLayer(pre_conv2_W, b4_l, [1,1,1,1], [2,2], [1,1]);
% 
%     w5_l = reshape(pre_bn2_weight,       [1 1 nf]);
%     b5_l = reshape(pre_bn2_bias,         [1 1 nf]);
%     m5_l = reshape(pre_bn2_running_mean, [1 1 nf]);
%     v5_l = reshape(pre_bn2_running_var,  [1 1 nf]);
%     layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l, ...
%                                       'TrainedMean',m5_l,'TrainedVariance',v5_l, ...
%                                       'NumChannels',nf, ...
%                                       'Epsilon',pre_bn2_eps);
% 
%     layer6 = make_activation_layer(activation);
% 
%     layer7 = FlattenLayer;
%     layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% 
%     net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});
% 
%     %% 1.3) Construct ODE layer
%     global W1_ode b1_ode W2_ode b2_ode
%     W1_ode = ode_W1;          % bottleneck x state_dim
%     b1_ode = ode_b1';         % bottleneck x 1
%     W2_ode = ode_W2;          % state_dim x bottleneck
%     b2_ode = ode_b2';         % state_dim x 1
% 
%     Aout = W2_ode * W1_ode;   % state_dim x state_dim (rank=bottleneck)
% 
%     ode_params = struct();
%     ode_params.system_choice = system_choice;
%     ode_params.t_final = 1;
%     ode_params.timeStep = 20;
%     ode_params.W1 = W1_ode;
%     ode_params.b1 = b1_ode;
%     ode_params.W2 = W2_ode;
%     ode_params.b2 = b2_ode;
%     odelayer_NODE_RA = TNODEVODELayer(ode_params);
% 
%     %% 1.4) Construct Post-ODE layers
%     layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
%     layerout_act  = make_activation_layer(activation);
%     layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
%     Full_Network_Components = {net_pre, odelayer_NODE_RA, ...
%                                 layerout_fc1, layerout_act, layerout_fc2};
% 
%     %% 2) Prepare data + run experiments
%     total_pixels = img_size * img_size * 3;
%     pixels_attack = randperm(total_pixels, min(pix, total_pixels));
% 
%     time_ode = zeros(numT, 1);
%     rob_ode  = zeros(numT, 1);   % 1 = Robust, 0 = Unknown, 2 = Not Robust
%     MAX_ITER = 1000;
% 
%     for i = 1:numT
%         fprintf('\n--- Image %d / %d ---\n', i, numT);
% 
%         % A. Build adversarial L_inf box
%         img = XTest(:,:,:,i);
%         lb = img;
%         ub = img;
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
%         % B. Falsification (1000 random samples)
%         fprintf('  Running Global Falsification (1000 samples)...\n');
%         is_falsified = run_generic_falsification(lb, ub, ...
%                                                   Full_Network_Components, ...
%                                                   target_label);
%         if is_falsified
%             fprintf('  >>> FALSIFIED. Counter-example found.\n');
%             rob_ode(i) = 2;
%             time_ode(i) = 0;
%             disp('Result: NOT ROBUST');
%             continue;
%         end
%         fprintf('  No counter-example. Proceeding to formal RA verification...\n');
% 
%         % C. Reachability + refinement
%         t = tic;
%         image_is_robust = true;
%         image_is_unknown = false;
%         iter = 0;
% 
%         p = gcp('nocreate');
%         if isempty(p), p = parpool; end
%         worker_load_factor = 5;
%         batch_size = p.NumWorkers * worker_load_factor;
%         fprintf('  [Parallel] Workers: %d | Batch: %d\n', p.NumWorkers, batch_size);
% 
%         % Pre-split to fill all workers
%         queue = {{lb, ub}};
%         num_presplits = ceil(log2(p.NumWorkers));
%         for ps = 1:num_presplits
%             new_queue = {};
%             for qi = 1:length(queue)
%                 [s1, s2] = split_interval_heuristic( ...
%                     queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
%                 new_queue{end+1} = s1;
%                 new_queue{end+1} = s2;
%             end
%             queue = new_queue;
%         end
%         fprintf('  Pre-split into %d sub-intervals.\n', length(queue));
% 
%         while ~isempty(queue)
%             iter = iter + 1;
%             fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', ...
%                     iter, length(queue));
% 
%             if iter > MAX_ITER
%                 fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
%                 image_is_unknown = true;
%                 break;
%             end
% 
%             current_batch_size = min(length(queue), batch_size);
%             batch_inputs = queue(1:current_batch_size);
%             queue(1:current_batch_size) = [];
% 
%             batch_results = zeros(1, current_batch_size);
% 
%             parfor b = 1:current_batch_size
%                 curr = batch_inputs{b};
%                 c_lb = curr{1}; c_ub = curr{2};
% 
%                 % --- 1) Pre-ODE
%                 fprintf('  (1/4) Pre-ODE...\n');
%                 inpS = ImageStar(c_lb, c_ub);
%                 R_pre = net_pre.reach(inpS);
% 
%                 % --- 2) ODE layer
%                 fprintf('  (2/4) ODE...\n');
%                 init_worker_globals(local_ode_W1, local_ode_b1', ...
%                                     local_ode_W2, local_ode_b2', ...
%                                     local_system_choice);
%                 S_ode_out = odelayer_NODE_RA.reach(R_pre);
% 
%                 % Volume check L8
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
%                 fprintf('        > L8 hull log-vol: %.4f\n', vol_8);
% 
%                 % --- 3) Post-ODE
%                 fprintf('  (3/4) Post-ODE...\n');
%                 S_fc1   = layerout_fc1.reach(S_ode_out, 'approx-star');
%                 S_act   = layerout_act.reach(S_fc1,    'approx-star');
%                 S_final = layerout_fc2.reach(S_act,    'approx-star');
% 
%                 % Volume check L11
%                 lb_all_11 = []; ub_all_11 = [];
%                 for s_idx = 1:length(S_final)
%                     B11 = S_final(s_idx).getBoxFast();
%                     lb_all_11(:, s_idx) = B11.lb;
%                     ub_all_11(:, s_idx) = B11.ub;
%                 end
%                 lb_11 = min(lb_all_11, [], 2);
%                 ub_11 = max(ub_all_11, [], 2);
%                 vol_11 = sum(log((ub_11 - lb_11) + epsilon));
%                 fprintf('        > L11 hull log-vol: %.4f\n', vol_11);
% 
%                 % --- 4) Robustness check
%                 res = check_robustness_manual(S_final, target_label, n_class);
% 
%                 if res == 1
%                     batch_results(b) = 0;   % Safe
%                     fprintf('  [RESULT] Sub-interval Safe.\n');
%                 else
%                     batch_results(b) = 1;   % Unknown -> split
%                 end
%             end
% 
%             % Process results & queue splits
%             safe_count = 0; split_count = 0;
%             for b = 1:current_batch_size
%                 if batch_results(b) == 0
%                     safe_count = safe_count + 1;
%                 else
%                     split_count = split_count + 1;
%                     curr = batch_inputs{b};
%                     [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, ...
%                                                        splitting_heuristic, Aout);
%                     queue{end+1} = s1;
%                     queue{end+1} = s2;
%                 end
%             end
%             fprintf('  Batch: %d Safe, %d Splitting.\n', safe_count, split_count);
%         end
% 
%         time_ode(i) = toc(t);
% 
%         if ~image_is_robust
%             rob_ode(i) = 2; disp('Final: NOT ROBUST');
%         elseif image_is_unknown
%             rob_ode(i) = 0; disp('Final: UNKNOWN (Timeout)');
%         else
%             rob_ode(i) = 1; disp('Final: ROBUST');
%         end
%     end
% 
%     % Final summary
%     rob   = sum(rob_ode == 1);
%     unk   = sum(rob_ode == 0);
%     notr  = sum(rob_ode == 2);
%     timeT = sum(time_ode);
% 
%     disp(' ');
%     disp('=== FINAL SUMMARY ===');
%     disp("Robust images:     " + string(rob));
%     disp("Unknown images:    " + string(unk));
%     disp("Not robust images: " + string(notr));
%     disp("Total time =       " + string(timeT));
% 
%     save("copepod_" + activation + "_NODE_RA_" + string(perturbation) + ...
%          "_" + string(noise) + ".mat", ...
%          'rob','rob_ode','timeT','pix','numT','noise', ...
%          'activation','nf','n_class','img_size');
% end
% 
% 
% % =========================================================================
% % Helpers
% % =========================================================================
% 
% function act_layer = make_activation_layer(activation)
%     switch lower(activation)
%         case 'tanh', act_layer = TanhLayer;
%         case 'relu', act_layer = ReluLayer;
%         otherwise
%             error('make_activation_layer:unknown', ...
%                   'Unknown activation ''%s''', activation);
%     end
% end
% 
% 
% function init_worker_globals(W1, b1, W2, b2, sc)
%     global W1_ode b1_ode W2_ode b2_ode system_choice
%     W1_ode = W1;
%     b1_ode = b1;
%     W2_ode = W2;
%     b2_ode = b2;
%     system_choice = sc;       % 19 = ReLU, 20 = tanh
%     warning('off', 'MATLAB:odearguments:InconsistentDataType');
%     warning('off', 'all');
% end
% 
% 
% function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)
%     radii = (ub - lb) / 2;
%     switch method
%         case 'naive'
%             [~, split_dim] = max(radii(:));
% 
%         case 'most-sensitive-input-radius'
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
%     [row, col, ch] = ind2sub(size(lb), split_dim);
%     mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;
%     lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
%     lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;
%     s1 = {lb1, ub1}; s2 = {lb2, ub2};
% end
% 
% 
% function robust = check_robustness_manual(S, target, n_class)
%     robust = 1;
%     for i = 1:length(S)
%         star_section = S(i);
%         if isempty(star_section), continue; end
%         for adv = 1:n_class
%             if adv == target, continue; end
%             c = zeros(1, n_class);
%             c(target) = 1;
%             c(adv)    = -1;
%             try
%                 s_unsafe = star_section.intersectHalfSpace(c, 0);
%                 if ~isempty(s_unsafe)
%                     robust = 0;
%                     return;
%                 end
%             catch
%                 fprintf('    [WARNING] Numerical instability in linprog. Treating as Unsafe.\n');
%                 robust = 0;
%                 return;
%             end
%         end
%     end
% end
% 
% 
% function falsified = run_generic_falsification(lb, ub, network_components, target_label)
%     falsified = false;
%     num_samples = 1000;
%     lb = double(lb); ub = double(ub);
%     input_shape = size(lb);
%     lb_flat = reshape(lb, [], 1);
%     ub_flat = reshape(ub, [], 1);
%     num_inputs = length(lb_flat);
%     X_samples = lb_flat + (ub_flat - lb_flat) .* rand(num_inputs, num_samples);
% 
%     for k = 1:num_samples
%         x_curr = X_samples(:, k);
%         x_curr = reshape(x_curr, input_shape);
%         for c = 1:length(network_components)
%             comp = network_components{c};
%             x_curr = comp.evaluate(x_curr);
%         end
%         [~, pred_idx] = max(x_curr);
%         if pred_idx ~= target_label
%             falsified = true;
%             return;
%         end
%     end
% end
% 
% % ------------- END reach_copepod_NODE_RA_new_unified.m ----------------

%% Aded the time out condition similar to Verifier_FINAL.m

% ------------- BEGIN reach_copepod_NODE_RA_new_unified.m ----------------
% Unified reachability-based robustness verifier for the TNODEV pipeline.
%
% Replaces:
%   reach_copepod_NODE_RA_new_tanh.m  (tanh, n_filters=2, n_class=7 hardcoded)
%   reach_copepod_NODE_RA_new.m       (relu version)
%
% Reads architecture metadata from the weights .mat:
%   * activation             -> 'tanh' (case 20) or 'relu' (case 19)
%   * n_filters              -> 2 or 4 (or any other)
%   * num_classes            -> 2 (binary) or 7 (full)
%   * img_size               -> 8 or 16
%   * spatial_after_downsample -> img_size / 2
%
% Network structure (unchanged from before, but parameterized):
%   Pre-ODE:   Conv(3->nf,s=1) -> BN -> Act -> Conv(nf->nf,s=2) -> BN -> Act -> Flatten
%   ODE:       dx/dt = W2 * act(W1*x + b1) + b2     [state_dim = nf*4*4 = 32 or 64]
%   Post-ODE:  FC(state_dim->64) -> Act -> FC(64->n_class)
%
% Inputs:
%   weights_file        - path to weights .mat (e.g. 'odenet_..._weights.mat')
%   pix, numT, noise    - verification config (as before)
%   XTest, YTest        - test data (as before)
%   perturbation        - 'inf' | 'random'
%   splitting_heuristic - 'naive' | 'most-sensitive-input-radius'
%                       | 'interval-norm-gradient'

function reach_copepod_NODE_RA_new_unified(weights_file, pix, numT, noise, ...
                                    XTest, YTest, perturbation, ...
                                    splitting_heuristic)

    %% 1) Load weights + introspect architecture
    fprintf('  Loading weights: %s\n', weights_file);
    W = load(weights_file);

    pre_conv1_W = W.pre_conv1_W;          pre_conv1_b = W.pre_conv1_b;
    pre_bn1_weight = W.pre_bn1_weight;    pre_bn1_bias = W.pre_bn1_bias;
    pre_bn1_running_mean = W.pre_bn1_running_mean;
    pre_bn1_running_var  = W.pre_bn1_running_var;
    pre_bn1_eps = W.pre_bn1_eps;
    pre_conv2_W = W.pre_conv2_W;          pre_conv2_b = W.pre_conv2_b;
    pre_bn2_weight = W.pre_bn2_weight;    pre_bn2_bias = W.pre_bn2_bias;
    pre_bn2_running_mean = W.pre_bn2_running_mean;
    pre_bn2_running_var  = W.pre_bn2_running_var;
    pre_bn2_eps = W.pre_bn2_eps;
    ode_W1 = W.ode_W1;   ode_b1 = W.ode_b1;
    ode_W2 = W.ode_W2;   ode_b2 = W.ode_b2;
    post_fc1_W = W.post_fc1_W;            post_fc1_b = W.post_fc1_b;
    post_fc2_W = W.post_fc2_W;            post_fc2_b = W.post_fc2_b;

    % --- Architecture introspection (same logic as run_copepod_quick) ---
    if isfield(W, 'n_filters'),   nf = double(W.n_filters);
    else,                         nf = size(pre_conv1_W, 4); end

    if isfield(W, 'num_classes'), n_class = double(W.num_classes);
    else,                         n_class = size(post_fc2_W, 1); end

    if isfield(W, 'img_size'),    img_size = double(W.img_size);
    else,                         img_size = size(XTest, 1); end

    if isfield(W, 'spatial_after_downsample')
        H_flat = double(W.spatial_after_downsample);
    else
        H_flat = img_size / 2;
    end
    W_flat = H_flat;
    C_flat = nf;
    n_flat = H_flat * W_flat * C_flat;

    if isfield(W, 'activation')
        activation = char(string(W.activation));
    else
        activation = 'tanh';
    end
    activation = lower(strtrim(activation));
    switch activation
        case 'tanh', system_choice = 20;
        case 'relu', system_choice = 19;
        otherwise
            error('reach_copepod_NODE_RA_new_unified:unknown_activation', ...
                  'Unknown activation ''%s''', activation);
    end

    fprintf('  Architecture: activation=%s (case %d) | nf=%d | img=%dx%d | flat=%d | n_class=%d\n', ...
            activation, system_choice, nf, img_size, img_size, n_flat, n_class);

    %% 1.1) Permute weights for PyTorch -> MATLAB flatten order
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
    ode_W1     = ode_W1(:, perm);
    ode_W2     = ode_W2(perm, :);
    ode_b2     = ode_b2(perm);
    post_fc1_W = post_fc1_W(:, perm);
    fprintf('  Applied PyTorch->MATLAB flatten permutation (%d-dim)\n', n_flat);

    % parfor-visible local copies
    local_ode_W1 = ode_W1;
    local_ode_b1 = ode_b1;
    local_ode_W2 = ode_W2;
    local_ode_b2 = ode_b2;
    local_system_choice = system_choice;

    %% 1.2) Construct Pre-ODE network
    b1_l = reshape(pre_conv1_b, [1, 1, nf]);
    layer1 = Conv2DLayer(pre_conv1_W, b1_l, [1,1,1,1], [1,1], [1,1]);

    w2_l = reshape(pre_bn1_weight,       [1 1 nf]);
    b2_l = reshape(pre_bn1_bias,         [1 1 nf]);
    m2_l = reshape(pre_bn1_running_mean, [1 1 nf]);
    v2_l = reshape(pre_bn1_running_var,  [1 1 nf]);
    layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l, ...
                                      'TrainedMean',m2_l,'TrainedVariance',v2_l, ...
                                      'NumChannels',nf, ...
                                      'Epsilon',pre_bn1_eps);

    layer3 = make_activation_layer(activation);

    b4_l = reshape(pre_conv2_b, [1, 1, nf]);
    layer4 = Conv2DLayer(pre_conv2_W, b4_l, [1,1,1,1], [2,2], [1,1]);

    w5_l = reshape(pre_bn2_weight,       [1 1 nf]);
    b5_l = reshape(pre_bn2_bias,         [1 1 nf]);
    m5_l = reshape(pre_bn2_running_mean, [1 1 nf]);
    v5_l = reshape(pre_bn2_running_var,  [1 1 nf]);
    layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l, ...
                                      'TrainedMean',m5_l,'TrainedVariance',v5_l, ...
                                      'NumChannels',nf, ...
                                      'Epsilon',pre_bn2_eps);

    layer6 = make_activation_layer(activation);

    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});

    %% 1.3) Construct ODE layer
    global W1_ode b1_ode W2_ode b2_ode
    W1_ode = ode_W1;          % bottleneck x state_dim
    b1_ode = ode_b1';         % bottleneck x 1
    W2_ode = ode_W2;          % state_dim x bottleneck
    b2_ode = ode_b2';         % state_dim x 1

    Aout = W2_ode * W1_ode;   % state_dim x state_dim (rank=bottleneck)

    ode_params = struct();
    ode_params.system_choice = system_choice;
    ode_params.t_final = 1;
    ode_params.timeStep = 20;
    ode_params.W1 = W1_ode;
    ode_params.b1 = b1_ode;
    ode_params.W2 = W2_ode;
    ode_params.b2 = b2_ode;
    odelayer_NODE_RA = TNODEVODELayer(ode_params);

    %% 1.4) Construct Post-ODE layers
    layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
    layerout_act  = make_activation_layer(activation);
    layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');

    Full_Network_Components = {net_pre, odelayer_NODE_RA, ...
                                layerout_fc1, layerout_act, layerout_fc2};

    %% 2) Prepare data + run experiments
    total_pixels = img_size * img_size * 3;
    pixels_attack = randperm(total_pixels, min(pix, total_pixels));

    time_ode = zeros(numT, 1);
    rob_ode  = zeros(numT, 1);   % 1 = Robust, 0 = Unknown, 2 = Not Robust
    % MAX_ITER = 1000;
    MAX_ITER        = 5000;       % refinement budget: max TOTAL sub-intervals processed (matches Verifier_FINAL)
    TIMEOUT_SECONDS = 120 * 60;   % per-image wall-clock cap on the refinement loop (120 minutes)

    for i = 1:numT
        fprintf('\n--- Image %d / %d ---\n', i, numT);

        % A. Build adversarial L_inf box
        img = XTest(:,:,:,i);
        lb = img;
        ub = img;

        if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
            for j = pixels_attack
                ub(j) = min(1, ub(j) + noise);
                lb(j) = max(0, lb(j) - noise);
            end
        else
            error('Wrong perturbation type');
        end

        target_label = YTest(i);

        % B. Falsification (1000 random samples)
        fprintf('  Running Global Falsification (1000 samples)...\n');
        is_falsified = run_generic_falsification(lb, ub, ...
                                                  Full_Network_Components, ...
                                                  target_label);
        if is_falsified
            fprintf('  >>> FALSIFIED. Counter-example found.\n');
            rob_ode(i) = 2;
            time_ode(i) = 0;
            disp('Result: NOT ROBUST');
            continue;
        end
        fprintf('  No counter-example. Proceeding to formal RA verification...\n');

        % C. Reachability + refinement
        t = tic;
        image_is_robust = true;
        image_is_unknown = false;
        iter = 0;

        p = gcp('nocreate');
        if isempty(p), p = parpool; end
        worker_load_factor = 5;
        batch_size = p.NumWorkers * worker_load_factor;
        fprintf('  [Parallel] Workers: %d | Batch: %d\n', p.NumWorkers, batch_size);

        % Pre-split to fill all workers
        queue = {{lb, ub}};
        num_presplits = ceil(log2(p.NumWorkers));
        for ps = 1:num_presplits
            new_queue = {};
            for qi = 1:length(queue)
                [s1, s2] = split_interval_heuristic( ...
                    queue{qi}{1}, queue{qi}{2}, splitting_heuristic, Aout);
                new_queue{end+1} = s1;
                new_queue{end+1} = s2;
            end
            queue = new_queue;
        end
        fprintf('  Pre-split into %d sub-intervals.\n', length(queue));
        
        total_subsets_processed = 0;   % counts sub-intervals popped & processed (the budget)
        loop_timer = tic;              % loop-only wall clock for the timeout

        while ~isempty(queue) ...
                && total_subsets_processed < MAX_ITER ...
                && toc(loop_timer) < TIMEOUT_SECONDS

            iter = iter + 1;
            
            % Bound this batch by the remaining budget so MAX_ITER is a hard
            % cap on the TOTAL number of sub-intervals processed.
            remaining_budget   = MAX_ITER - total_subsets_processed;
            current_batch_size = min([length(queue), batch_size, remaining_budget]);

            fprintf(['\n--- Refinement Iteration %d (Queue: %d, ' ...
                     'Processed: %d/%d, Elapsed: %.1f/%.0f min) ---\n'], ...
                    iter, length(queue), total_subsets_processed, MAX_ITER, ...
                    toc(loop_timer)/60, TIMEOUT_SECONDS/60);

            batch_inputs = queue(1:current_batch_size);
            queue(1:current_batch_size) = [];
            batch_results = zeros(1, current_batch_size);

            parfor b = 1:current_batch_size
                curr = batch_inputs{b};
                c_lb = curr{1}; c_ub = curr{2};

                % --- 1) Pre-ODE
                fprintf('  (1/4) Pre-ODE...\n');
                inpS = ImageStar(c_lb, c_ub);
                R_pre = net_pre.reach(inpS);

                % --- 2) ODE layer
                fprintf('  (2/4) ODE...\n');
                init_worker_globals(local_ode_W1, local_ode_b1', ...
                                    local_ode_W2, local_ode_b2', ...
                                    local_system_choice);
                S_ode_out = odelayer_NODE_RA.reach(R_pre);

                % Volume check L8
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
                fprintf('        > L8 hull log-vol: %.4f\n', vol_8);

                % --- 3) Post-ODE
                fprintf('  (3/4) Post-ODE...\n');
                S_fc1   = layerout_fc1.reach(S_ode_out, 'approx-star');
                S_act   = layerout_act.reach(S_fc1,    'approx-star');
                S_final = layerout_fc2.reach(S_act,    'approx-star');

                % Volume check L11
                lb_all_11 = []; ub_all_11 = [];
                for s_idx = 1:length(S_final)
                    B11 = S_final(s_idx).getBoxFast();
                    lb_all_11(:, s_idx) = B11.lb;
                    ub_all_11(:, s_idx) = B11.ub;
                end
                lb_11 = min(lb_all_11, [], 2);
                ub_11 = max(ub_all_11, [], 2);
                vol_11 = sum(log((ub_11 - lb_11) + epsilon));
                fprintf('        > L11 hull log-vol: %.4f\n', vol_11);

                % --- 4) Robustness check
                res = check_robustness_manual(S_final, target_label, n_class);

                if res == 1
                    batch_results(b) = 0;   % Safe
                    fprintf('  [RESULT] Sub-interval Safe.\n');
                else
                    batch_results(b) = 1;   % Unknown -> split
                end
            end

            % Process results & queue splits
            safe_count = 0; split_count = 0;
            for b = 1:current_batch_size
                if batch_results(b) == 0
                    safe_count = safe_count + 1;
                else
                    split_count = split_count + 1;
                    curr = batch_inputs{b};
                    [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, ...
                                                       splitting_heuristic, Aout);
                    queue{end+1} = s1;
                    queue{end+1} = s2;
                end
            end
            fprintf('  Batch: %d Safe, %d Splitting.\n', safe_count, split_count);
            total_subsets_processed = total_subsets_processed + current_batch_size;
        end
        
        % Non-empty queue at exit => we hit the budget or the timeout => UNKNOWN.
        if ~isempty(queue)
            image_is_unknown = true;
            if toc(loop_timer) >= TIMEOUT_SECONDS
                fprintf('  Stopped: 120-min wall-clock timeout reached (queue not empty -> UNKNOWN).\n');
            elseif total_subsets_processed >= MAX_ITER
                fprintf('  Stopped: refinement budget (%d sub-intervals) reached (queue not empty -> UNKNOWN).\n', MAX_ITER);
            end
        end

        time_ode(i) = toc(t);

        if ~image_is_robust
            rob_ode(i) = 2; disp('Final: NOT ROBUST');
        elseif image_is_unknown
            rob_ode(i) = 0; disp('Final: UNKNOWN (Timeout)');
        else
            rob_ode(i) = 1; disp('Final: ROBUST');
        end
    end

    % Final summary
    rob   = sum(rob_ode == 1);
    unk   = sum(rob_ode == 0);
    notr  = sum(rob_ode == 2);
    timeT = sum(time_ode);

    disp(' ');
    disp('=== FINAL SUMMARY ===');
    disp("Robust images:     " + string(rob));
    disp("Unknown images:    " + string(unk));
    disp("Not robust images: " + string(notr));
    disp("Total time =       " + string(timeT));

    save("copepod_" + activation + "_NODE_RA_" + string(perturbation) + ...
         "_" + string(noise) + ".mat", ...
         'rob','rob_ode','timeT','pix','numT','noise', ...
         'activation','nf','n_class','img_size');
end


% =========================================================================
% Helpers
% =========================================================================

function act_layer = make_activation_layer(activation)
    switch lower(activation)
        case 'tanh', act_layer = TanhLayer;
        case 'relu', act_layer = ReluLayer;
        otherwise
            error('make_activation_layer:unknown', ...
                  'Unknown activation ''%s''', activation);
    end
end


function init_worker_globals(W1, b1, W2, b2, sc)
    global W1_ode b1_ode W2_ode b2_ode system_choice
    W1_ode = W1;
    b1_ode = b1;
    W2_ode = W2;
    b2_ode = b2;
    system_choice = sc;       % 19 = ReLU, 20 = tanh
    warning('off', 'MATLAB:odearguments:InconsistentDataType');
    warning('off', 'all');
end


function [s1, s2] = split_interval_heuristic(lb, ub, method, Aout)
    radii = (ub - lb) / 2;
    switch method
        case 'naive'
            [~, split_dim] = max(radii(:));

        case 'most-sensitive-input-radius'
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

    [row, col, ch] = ind2sub(size(lb), split_dim);
    mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;
    lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
    lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;
    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end


function robust = check_robustness_manual(S, target, n_class)
    robust = 1;
    for i = 1:length(S)
        star_section = S(i);
        if isempty(star_section), continue; end
        for adv = 1:n_class
            if adv == target, continue; end
            c = zeros(1, n_class);
            c(target) = 1;
            c(adv)    = -1;
            try
                s_unsafe = star_section.intersectHalfSpace(c, 0);
                if ~isempty(s_unsafe)
                    robust = 0;
                    return;
                end
            catch
                fprintf('    [WARNING] Numerical instability in linprog. Treating as Unsafe.\n');
                robust = 0;
                return;
            end
        end
    end
end


function falsified = run_generic_falsification(lb, ub, network_components, target_label)
    falsified = false;
    num_samples = 1000;
    lb = double(lb); ub = double(ub);
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

% ------------- END reach_copepod_NODE_RA_new_unified.m ----------------