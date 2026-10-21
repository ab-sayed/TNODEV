% reach_copepod_NODE_RA_new.m — V2.0 (work-stealing, case 19 nonlinear)
%
% Copepod Neural ODE robustness verification via TNODEV
% Adapted from reach_cnn_tiny_NODE_RA_new.m (MNIST V2.2.0)
%
% Network:
%   Pre-ODE (1-7):  Conv(3→2,s=1)→BN→ReLU→Conv(2→2,s=2)→BN→ReLU→Flatten
%   ODE (8):        dx/dt = W2*ReLU(W1*x+b1)+b2  [32-dim, 8-neuron bottleneck]
%   Post-ODE (9-11): FC(32→64)→ReLU→FC(64→7)
%
% Images: 8×8×3, 7 classes

function reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, perturbation, splitting_heuristic)

    %% Suppress harmless ode45 warnings
    warning('off', 'all');

    %% 0) Start parallel pool
    % TIRA is single-threaded per worker (confirmed on MNIST)
    MAX_WORKERS = 48;
    num_workers = min(feature('numcores') - 1, MAX_WORKERS);
    num_workers = max(num_workers, 1);

    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('local', num_workers);
        fprintf('Started parpool with %d workers on %d cores (cap=%d).\n', ...
                pool.NumWorkers, feature('numcores'), MAX_WORKERS);
    else
        num_workers = pool.NumWorkers;
        fprintf('Using existing parpool with %d workers.\n', num_workers);
    end

    %% 1) Load and construct the NeuralODE

    file_path = 'odenet_tiny_copepod_weights.mat';
    load(file_path);

    % --- 1.1 Pre-ODE Layers (1-7) ---

    % Layer 1: Conv2d(3→2, 3×3, stride=1, padding=1)
    w1 = pre_conv1_W;  % [3, 3, 3, 2]
    b1 = reshape(pre_conv1_b, [1, 1, 2]);
    layer1 = Conv2DLayer(w1, b1, [1,1,1,1], [1,1], [1,1]);

    % Layer 2: BatchNorm2d(2)
    w2 = reshape(pre_bn1_weight, [1 1 2]);
    b2 = reshape(pre_bn1_bias, [1 1 2]);
    m2 = reshape(pre_bn1_running_mean, [1 1 2]);
    v2 = reshape(pre_bn1_running_var, [1 1 2]);
    layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2, ...
             'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',2,'Epsilon',2);

    % Layer 3: ReLU
    layer3 = ReluLayer;

    % Layer 4: Conv2d(2→2, 3×3, stride=2, padding=1)
    w4 = pre_conv2_W;  % [3, 3, 2, 2]
    b4 = reshape(pre_conv2_b, [1, 1, 2]);
    layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);

    % Layer 5: BatchNorm2d(2)
    w5 = reshape(pre_bn2_weight, [1 1 2]);
    b5 = reshape(pre_bn2_bias, [1 1 2]);
    m5 = reshape(pre_bn2_running_mean, [1 1 2]);
    v5 = reshape(pre_bn2_running_var, [1 1 2]);
    layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5, ...
             'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',2,'Epsilon',2);

    % Layer 6: ReLU
    layer6 = ReluLayer;

    % Layer 7: Flatten (8×8×2 → 32 after stride-2 conv on 8×8 → 4×4×2 = 32)
    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
    net_pre = NN(neuralLayers_Pre);

    % --- 1.2 ODE Layer (Layer 8) ---
    % Nonlinear: dx/dt = W2 * ReLU(W1*x + b1) + b2 (case 19)

    global W1_ode b1_ode W2_ode b2_ode
    W1_ode = ode_W1;          % 8 × 32
    b1_ode = ode_b1';         % 8 × 1
    W2_ode = ode_W2;          % 32 × 8
    b2_ode = ode_b2';         % 32 × 1

    ode_params = struct();
    ode_params.system_choice = 19;
    ode_params.t_final = 1;
    ode_params.timeStep = 20;
    ode_params.W1 = W1_ode;
    ode_params.b1 = b1_ode;
    ode_params.W2 = W2_ode;
    ode_params.b2 = b2_ode;

    odelayer_NODE_RA = TNODEVODELayer(ode_params);

    % --- 1.3 Post-ODE Layers (9-11) ---
    % FC(32→64) → ReLU → FC(64→7)
    layerout_fc1  = FullyConnectedLayer(post_fc1_W, post_fc1_b');
    layerout_relu = ReluLayer;
    layerout_fc2  = FullyConnectedLayer(post_fc2_W, post_fc2_b');

    % --- 1.4 Full Network for Falsification ---
    Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2};

    % --- 1.5 Print ODE dynamics info ---
    Aout = W2_ode * W1_ode;  % 32×32 rank-8 (for diagnosis only)
    fprintf('\n=== ODE Layer: NONLINEAR (case 19) ===\n');
    fprintf('  True dynamics: dx/dt = W2 * ReLU(W1*x + b1) + b2\n');
    fprintf('  W1: %dx%d | W2: %dx%d | Bottleneck: %d neurons\n', ...
            size(W1_ode,1), size(W1_ode,2), size(W2_ode,1), size(W2_ode,2), size(W1_ode,1));
    fprintf('  ODE state dim: %d | Collapsed A rank: %d\n', size(W2_ode,1), rank(Aout));
    fprintf('========================================\n\n');

    %% 2) Run experiments

    noise = noise * 255;
    n_class = 7;
    total_pixels = 8 * 8 * 3;  % 192
    pixels_attack = randperm(total_pixels, min(pix, total_pixels));

    time_ode = zeros(numT, 1);
    rob_ode  = zeros(numT, 1);

    % Wall-clock timeout
    slurm_time_str = getenv('SLURM_JOB_TIME_LIMIT');
    if ~isempty(slurm_time_str)
        parts = str2double(strsplit(slurm_time_str, ':'));
        if length(parts) == 3
            wall_limit_sec = parts(1)*3600 + parts(2)*60 + parts(3);
        else
            wall_limit_sec = 3600;
        end
    else
        wall_limit_sec = 600;
    end
    WALL_TIMEOUT = wall_limit_sec - 90;
    job_start_time = tic;

    for i = 1:numT
        fprintf('\n========== Image %d / %d ==========\n', i, numT);

        % A. CREATE ADVERSARIAL ATTACK
        img = XTest(:,:,:,i);  % [8, 8, 3]
        lb = img;
        ub = img;

        if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
            for j = pixels_attack
                ub(j) = min(255, ub(j) + noise);
                lb(j) = max(0, lb(j) - noise);
            end
        else
            error('Wrong perturbation type');
        end

        lb = lb ./ 255;
        ub = ub ./ 255;
        target_label = YTest(i);

        input_radii = (ub(:) - lb(:)) / 2;
        num_perturbed = sum(input_radii > 0);
        fprintf('  Input: %d perturbed pixels, max radius = %.6f\n', num_perturbed, max(input_radii));

        % B. GLOBAL FALSIFICATION
        fprintf('  Running Global Falsification (1000 samples)...\n');
        w_state = warning('off', 'all');
        is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);
        warning(w_state);

        if is_falsified
            fprintf('  >>> FALSIFIED! Counter-example found.\n');
            rob_ode(i) = 2;
            time_ode(i) = 0;
            continue;
        end

        fprintf('  No counter-example found. Starting formal verification...\n');

        % C. CONTINUOUS WORK-STEALING VERIFICATION
        t = tic;
        queue = {{lb, ub}};
        image_is_robust = true;
        image_is_unknown = false;
        total_verified = 0;
        total_split = 0;
        total_falsified = 0;

        vol8_history = [];
        vol11_history = [];

        % === COMPUTE HEURISTIC SCORES ===
        heuristic_scores = zeros(numel(lb), 1);

        if strcmp(splitting_heuristic, 'most-sensitive-input-radius') || ...
           strcmp(splitting_heuristic, 'interval-norm-gradient')

            fprintf('  Computing %s scores...', splitting_heuristic);
            sens_tic = tic;
            center_flat = reshape(double((lb + ub) / 2), [], 1);
            radii_flat = reshape((ub - lb) / 2, [], 1);
            input_shape = size(lb);
            perturbed_dims = find(radii_flat > 0);

            w_s = warning('off', 'all');
            x_base = reshape(center_flat, input_shape);
            for c_idx = 1:length(Full_Network_Components)
                x_base = Full_Network_Components{c_idx}.evaluate(x_base);
            end
            base_out = double(x_base(:));

            delta = 1e-6;

            if strcmp(splitting_heuristic, 'most-sensitive-input-radius')
                for idx = 1:length(perturbed_dims)
                    j = perturbed_dims(idx);
                    x_pert = center_flat;
                    x_pert(j) = x_pert(j) + delta;
                    x_curr = reshape(x_pert, input_shape);
                    for c_idx = 1:length(Full_Network_Components)
                        x_curr = Full_Network_Components{c_idx}.evaluate(x_curr);
                    end
                    sens_j = norm(double(x_curr(:)) - base_out) / delta;
                    heuristic_scores(j) = radii_flat(j) * sens_j;
                end

            elseif strcmp(splitting_heuristic, 'interval-norm-gradient')
                base_range = max(base_out) - min(base_out);
                for idx = 1:length(perturbed_dims)
                    j = perturbed_dims(idx);
                    x_fwd = center_flat; x_fwd(j) = x_fwd(j) + delta;
                    x_curr = reshape(x_fwd, input_shape);
                    for c_idx = 1:length(Full_Network_Components)
                        x_curr = Full_Network_Components{c_idx}.evaluate(x_curr);
                    end
                    out_fwd = double(x_curr(:));

                    x_bwd = center_flat; x_bwd(j) = x_bwd(j) - delta;
                    x_curr = reshape(x_bwd, input_shape);
                    for c_idx = 1:length(Full_Network_Components)
                        x_curr = Full_Network_Components{c_idx}.evaluate(x_curr);
                    end
                    out_bwd = double(x_curr(:));

                    range_fwd = max(out_fwd) - min(out_fwd);
                    range_bwd = max(out_bwd) - min(out_bwd);
                    heuristic_scores(j) = abs(range_fwd - range_bwd) / (2 * delta);
                end
            end
            warning(w_s);

            [~, top_idx] = sort(heuristic_scores, 'descend');
            top_n = min(5, length(perturbed_dims));
            fprintf(' done in %.1fs (%d dims, top-%d: ', toc(sens_tic), length(perturbed_dims), top_n);
            for si = 1:top_n
                fprintf('dim%d=%.2e ', top_idx(si), heuristic_scores(top_idx(si)));
            end
            fprintf(')\n');
        end

        % === SEED SPLITTING ===
        seed_target = num_workers * 2;
        fprintf('  Seed splitting: filling queue from 1 to %d items...', seed_target);
        seed_tic = tic;
        while length(queue) < seed_target
            curr = queue{1};
            queue(1) = [];
            [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, heuristic_scores);
            queue{end+1} = s1;
            queue{end+1} = s2;
        end
        fprintf(' done in %.2fs (queue: %d items)\n', toc(seed_tic), length(queue));

        % === CONTINUOUS WORK-STEALING ===
        active_inputs = cell(1, num_workers);
        futures = parallel.FevalFuture.empty(0, num_workers);

        n_initial = min(length(queue), num_workers);
        for w = 1:n_initial
            curr = queue{1};
            queue(1) = [];
            active_inputs{w} = curr;
            futures(w) = parfeval(pool, @verify_single_interval, 1, ...
                curr{1}, curr{2}, ...
                net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2, ...
                target_label, n_class, ...
                W1_ode, b1_ode, W2_ode, b2_ode, ...
                Full_Network_Components);
        end
        n_active = n_initial;
        tasks_completed = 0;
        last_report_time = tic;
        report_interval = 30;

        fprintf('  Filled %d workers. Starting continuous processing...\n', n_active);

        while n_active > 0
            if toc(job_start_time) > WALL_TIMEOUT
                fprintf('  [WALL TIMEOUT] Cancelling %d active tasks...\n', n_active);
                cancel(futures);
                image_is_unknown = true;
                break;
            end

            [completedIdx, result] = fetchNext(futures);
            tasks_completed = tasks_completed + 1;

            if ~isnan(result.vol_8),  vol8_history(end+1)  = result.vol_8;  end
            if ~isnan(result.vol_11), vol11_history(end+1) = result.vol_11; end

            if result.status == 0
                total_verified = total_verified + 1;
            elseif result.status == 2
                total_falsified = total_falsified + 1;
                image_is_robust = false;
            else
                total_split = total_split + 1;
                curr = active_inputs{completedIdx};
                [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, heuristic_scores);
                queue{end+1} = s1;
                queue{end+1} = s2;
            end

            % Immediately refill this worker slot
            if ~isempty(queue) && image_is_robust
                curr = queue{1};
                queue(1) = [];
                active_inputs{completedIdx} = curr;
                futures(completedIdx) = parfeval(pool, @verify_single_interval, 1, ...
                    curr{1}, curr{2}, ...
                    net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2, ...
                    target_label, n_class, ...
                    W1_ode, b1_ode, W2_ode, b2_ode, ...
                    Full_Network_Components);
            else
                n_active = n_active - 1;
            end

            % Periodic status
            if toc(last_report_time) > report_interval || ~image_is_robust
                fprintf('  [t=%.0fs] tasks=%d | safe=%d | split=%d | falsified=%d | queue=%d | vol8=%.1f | vol11=%.1f | radius=%.2e\n', ...
                    toc(t), tasks_completed, total_verified, total_split, total_falsified, ...
                    length(queue), result.vol_8, result.vol_11, result.input_radius);
                last_report_time = tic;
            end

            if ~image_is_robust
                fprintf('  Image FALSIFIED! Cancelling remaining tasks...\n');
                cancel(futures);
                break;
            end
        end

        % Final status
        fprintf('\n  === Verification complete ===\n');
        fprintf('  Total tasks: %d | Safe: %d | Split: %d | Falsified: %d\n', ...
                tasks_completed, total_verified, total_split, total_falsified);
        fprintf('  Queue remaining: %d\n', length(queue));
        if ~isempty(vol11_history)
            fprintf('  vol11 stats: mean=%.2f, min=%.2f, max=%.2f, n=%d\n', ...
                    mean(vol11_history), min(vol11_history), max(vol11_history), length(vol11_history));
        end

        clear futures active_inputs;

        time_ode(i) = toc(t);

        if ~image_is_robust
            rob_ode(i) = 2;
            fprintf('Final Result: NOT ROBUST (%.2fs)\n', time_ode(i));
        elseif image_is_unknown
            rob_ode(i) = 0;
            fprintf('Final Result: UNKNOWN - Timeout (%.2fs)\n', time_ode(i));
        else
            rob_ode(i) = 1;
            fprintf('Final Result: ROBUST (%.2fs)\n', time_ode(i));
        end
    end

    %% 3) Summary and Save
    rob  = sum(rob_ode == 1);
    unk  = sum(rob_ode == 0);
    notr = sum(rob_ode == 2);
    timeT = sum(time_ode);

    fprintf('\n=== FINAL SUMMARY ===\n');
    fprintf('Robust images:     %d\n', rob);
    fprintf('Unknown images:    %d\n', unk);
    fprintf('Not robust images: %d\n', notr);
    fprintf('Total time =       %.2f s\n', timeT);
    fprintf('Wall time used =   %.2f s / %.2f s limit\n', toc(job_start_time), wall_limit_sec);

    save("copepod_NODE_RA_NEW_nonlinear_" + string(perturbation) + "_" + string(noise) + ".mat", ...
         'rob', 'rob_ode', 'timeT', 'pix', 'numT', 'noise', 'vol8_history', 'vol11_history');

    delete(pool);
end


% =========================================================================
% WORKER FUNCTION
% =========================================================================
function result = verify_single_interval(c_lb, c_ub, ...
    net_pre, odelayer_NODE_RA, layerout_fc1, layerout_relu, layerout_fc2, ...
    target_label, n_class, ...
    W1_param, b1_param, W2_param, b2_param, ...
    Full_Network_Components)

    task_timer = tic;
    epsilon = 1e-20;

    result = struct();
    result.status = 1;          % 0=Safe, 1=Unknown, 2=Falsified
    result.status_str = 'UNKNOWN';
    result.vol_8 = NaN;
    result.vol_11 = NaN;
    result.input_radius = max(abs(c_ub(:) - c_lb(:))) / 2;
    result.elapsed = 0;

    % Set globals for TIRA
    global W1_ode b1_ode W2_ode b2_ode
    W1_ode = W1_param;
    b1_ode = b1_param;
    W2_ode = W2_param;
    b2_ode = b2_param;

    try
        % === STEP 0: Quick sub-interval falsification (50 samples) ===
        w_state = warning('off', 'all');
        input_shape = size(c_lb);
        lb_flat = reshape(double(c_lb), [], 1);
        ub_flat = reshape(double(c_ub), [], 1);
        n_quick = 50;
        X_quick = lb_flat + (ub_flat - lb_flat) .* rand(length(lb_flat), n_quick);

        for k = 1:n_quick
            x_curr = reshape(X_quick(:,k), input_shape);
            for c = 1:length(Full_Network_Components)
                x_curr = Full_Network_Components{c}.evaluate(x_curr);
            end
            [~, pred_idx] = max(x_curr);
            if pred_idx ~= target_label
                result.status = 2;
                result.status_str = 'FALSIFIED';
                result.elapsed = toc(task_timer);
                warning(w_state);
                return;
            end
        end
        warning(w_state);

        % === STEP 1: Pre-ODE Layers (1-7) via NNV ===
        inpS = ImageStar(c_lb, c_ub);
        R_pre = net_pre.reach(inpS);

        % === STEP 2: ODE Layer (8) via TNODEV ===
        S_ode_out = odelayer_NODE_RA.reach(R_pre);

        % --- Volume Check Layer 8 ---
        lb_all_8 = []; ub_all_8 = [];
        for s_idx = 1:length(S_ode_out)
            B8 = S_ode_out(s_idx).getBoxFast();
            lb_all_8(:, s_idx) = B8.lb;
            ub_all_8(:, s_idx) = B8.ub;
        end
        lb_8 = min(lb_all_8, [], 2);
        ub_8 = max(ub_all_8, [], 2);
        widths_8 = ub_8 - lb_8;
        widths_8(isnan(widths_8) | isinf(widths_8) | widths_8 < 0) = 0;
        result.vol_8 = sum(log(widths_8 + epsilon));

        % === STEP 3: Post-ODE Layers (9-11) via NNV ===
        S_fc1  = layerout_fc1.reach(S_ode_out, 'exact-star');
        S_relu = layerout_relu.reach(S_fc1, 'exact-star');
        S_final = layerout_fc2.reach(S_relu, 'exact-star');

        % --- Volume Check Layer 11 ---
        lb_all_11 = []; ub_all_11 = [];
        for s_idx = 1:length(S_final)
            B11 = S_final(s_idx).getBoxFast();
            lb_all_11(:, s_idx) = B11.lb;
            ub_all_11(:, s_idx) = B11.ub;
        end
        lb_11 = min(lb_all_11, [], 2);
        ub_11 = max(ub_all_11, [], 2);
        widths_11 = ub_11 - lb_11;
        widths_11(isnan(widths_11) | isinf(widths_11) | widths_11 < 0) = 0;
        result.vol_11 = sum(log(widths_11 + epsilon));

        % === STEP 4: Robustness Check ===
        robust = check_robustness_worker(S_final, target_label, n_class);

        if robust == 0
            result.status = 0;
            result.status_str = 'SAFE';
        else
            result.status = 1;
            result.status_str = 'UNKNOWN';
        end

    catch ME
        fprintf('    [WORKER ERROR] %s\n', ME.message);
        result.status = 1;
        result.status_str = sprintf('ERROR:%s', ME.message(1:min(50,end)));
    end

    result.elapsed = toc(task_timer);
end


% =========================================================================
% Helper: Robustness check (7 classes)
% =========================================================================
function robust = check_robustness_worker(S, target, n_class)
    robust = 0;

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
                    robust = 1;
                    return;
                end
            catch
                robust = 1;
                return;
            end
        end
    end
end


% =========================================================================
% Helper: Splitting Heuristic (3D image arrays)
% =========================================================================
function [s1, s2] = split_interval_heuristic(lb, ub, method, heuristic_scores)
    radii = (ub - lb) / 2;

    switch method
        case 'naive'
            [~, split_dim] = max(radii(:));

        case 'most-sensitive-input-radius'
            max_r = max(radii(:));
            candidates = find(radii(:) >= max_r - eps(max_r));
            if ~isempty(heuristic_scores) && any(heuristic_scores(candidates) > 0)
                [~, best_idx] = max(heuristic_scores(candidates));
                split_dim = candidates(best_idx);
            else
                split_dim = candidates(randi(length(candidates)));
            end

        case 'interval-norm-gradient'
            max_r = max(radii(:));
            candidates = find(radii(:) >= max_r - eps(max_r));
            if ~isempty(heuristic_scores) && any(heuristic_scores(candidates) > 0)
                [~, best_idx] = max(heuristic_scores(candidates));
                split_dim = candidates(best_idx);
            else
                split_dim = candidates(randi(length(candidates)));
            end

        otherwise
            warning('Unknown method "%s". Defaulting to naive.', method);
            [~, split_dim] = max(radii(:));
    end

    % Convert linear index to subscripts for 3D image array
    [row, col, ch] = ind2sub(size(lb), split_dim);
    mid = (lb(row, col, ch) + ub(row, col, ch)) / 2;

    lb1 = lb; ub1 = ub; ub1(row, col, ch) = mid;
    lb2 = lb; ub2 = ub; lb2(row, col, ch) = mid;
    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end


% =========================================================================
% Helper: Global Falsification
% =========================================================================
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
            x_curr = network_components{c}.evaluate(x_curr);
        end
        [~, pred_idx] = max(x_curr);
        if pred_idx ~= target_label
            falsified = true;
            return;
        end
    end
end
