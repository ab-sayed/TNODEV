function reach_cnn_tiny_NODE_RA_smoketest(pix, numT, noise, XTest, YTest, perturbation)
% reach_cnn_tiny_NODE_RA_smoketest - Single-pass full-image L_inf
% verification for MNIST tiny ODE-CNN.
%
% Differences vs reach_cnn_tiny_NODE_RA_new.m:
%   - NO falsification step
%   - NO iterative refinement / input-space splitting
%   - NO parfor (single image, single pass)
%   - Per-stage interval-width diagnostics (see exactly where bounds blow up)
%   - Saves ODE-input box to .mat for inspection
%
% Pipeline:
%   ImageStar  ->  NNV Pre-ODE (1-7)  ->  TNODEV ODE (8)
%               ->  NNV final FC (9)  ->  robustness check
%
% Outputs printed to stdout; results saved to:
%   smoketest_S_ode_in_img<i>.mat   (ODE-input box per image, for PJ)
%   smoketest_results.mat           (verdict + timings)
%
% Inputs:
%   pix         : number of perturbed pixels (use 784 for full-image)
%   numT        : number of images to test
%   noise       : L_inf radius in normalized [0,1] units (will be scaled by 255 internally)
%   XTest       : 28 x 28 x 1 x numT image tensor (single)
%   YTest       : numT x 1 label vector
%   perturbation: 'inf' (or 'random' - same logic for selected pixels)
%
% Author: Abdelrahman Sayed Sayed | Date: May 2026

    %% 1) Construct the full network (mirrors reach_cnn_tiny_NODE_RA_new)

    file_path = 'odecnn_mnist_tiny.mat';
    load(file_path); %#ok<LOAD>

    % --- Pre-ODE layers (1-7) ---
    w1 = permute(Wb{1},[4 3 2 1]);                                       %#ok<USENS>
    b1 = reshape(Wb{2}, [1, 1, 4]);
    layer1 = Conv2DLayer(w1, b1, [0,0,0,0], [1,1], [1,1]);

    w2 = reshape(Wb{3}, [1 1 4]);   b2 = reshape(Wb{4},  [1 1 4]);
    m2 = reshape(Wb{15},[1 1 4]);   v2 = reshape(Wb{16}, [1 1 4]);
    layer2 = BatchNormalizationLayer('Offset', b2, 'Scale', w2, ...
        'TrainedMean', m2, 'TrainedVariance', v2, 'NumChannels', 4, 'Epsilon', 4);

    layer3 = ReluLayer;

    w4 = permute(Wb{5}, [4 3 2 1]);
    b4 = reshape(Wb{6}, [1, 1, 4]);
    layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);

    w5 = reshape(Wb{7}, [1 1 4]);   b5 = reshape(Wb{8},  [1 1 4]);
    m5 = reshape(Wb{18},[1 1 4]);   v5 = reshape(Wb{19}, [1 1 4]);
    layer5 = BatchNormalizationLayer('Offset', b5, 'Scale', w5, ...
        'TrainedMean', m5, 'TrainedVariance', v5, 'NumChannels', 4, 'Epsilon', 4);

    layer6 = ReluLayer;
    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});

    % --- ODE layer (8): linear dynamics dx/dt = Aout*x + Bout ---
    w_ode1 = Wb{9};   b_ode1 = Wb{10}';
    w_ode2 = Wb{11};  b_ode2 = Wb{12}';
    Aout = w_ode2 * w_ode1;
    Bout = w_ode2 * b_ode1 + b_ode2;

    ode_params = struct();
    ode_params.system_choice = 16;     % MNIST linear NODE in System_description.m
    ode_params.t_final       = 1;       % paper's time horizon
    ode_params.timeStep      = 20;      % 20 sub-steps of 0.05
    ode_params.A             = Aout;
    ode_params.B             = Bout;
    odelayer_NODE_RA = TNODEVODELayer(ode_params);

    % --- Final FC (layer 9) ---
    layerout = FullyConnectedLayer(Wb{13}, Wb{14}');

    % --- Diagnostics about Aout (one-time, important for understanding bounds) ---
    fprintf('\n========== TNODEV MNIST SMOKE TEST ==========\n');
    fprintf('Settings:  pix=%d (%s) | numT=%d | noise=%.6f (= %.4f/255)\n', ...
            pix, ternary(pix==784, 'FULL IMAGE', 'partial'), numT, noise, noise*255);
    fprintf('ODE: t_final=%g, timeStep=%d (%d sub-steps of %g)\n', ...
            ode_params.t_final, ode_params.timeStep, ode_params.timeStep, ...
            ode_params.t_final/ode_params.timeStep);
    eigs_A    = eig(Aout);
    max_real  = max(real(eigs_A));
    fprintf('Aout: dim %dx%d | max Re(eig) = %+.4f | ||Aout||_inf = %.4f\n', ...
            size(Aout,1), size(Aout,2), max_real, norm(Aout, 'inf'));
    fprintf('Bout: dim %dx1   | ||Bout||_inf = %.4f\n', size(Bout,1), norm(Bout, 'inf'));
    if max_real > 0
        fprintf('[!] Aout has unstable modes (max Re(eig) > 0). Expect rapid wrapping.\n');
    end
    fprintf('=============================================\n\n');

    %% 2) Prepare attack and storage

    noise_abs    = noise * 255;             % absolute on [0,255] scale
    pixels_attack = randperm(784, pix);     % which pixels to perturb

    rob       = zeros(numT, 1);    % 1=Robust, 0=Unknown, 2=NotRobust, -1=NumericalFail
    time_tot  = zeros(numT, 1);
    stage_t   = zeros(numT, 4);    % [pre, ode, fc, check]

    %% 3) Loop over images (no refinement, no falsification)

    for i = 1:numT
        fprintf('\n--- Image %d / %d (label = %d) ---\n', i, numT, YTest(i));
        t_total = tic;

        % ------ Build L_inf box (in normalized [0,1] units) ------
        img_flat = XTest(:,:,:,i)';      % 28x28 (flattened layout used by NNV ImageStar)
        lb = img_flat;  ub = img_flat;
        if ~ismember(perturbation, {'inf', 'random'})
            error('Wrong perturbation type: %s', perturbation);
        end
        for j = pixels_attack
            ub(j) = min(255, ub(j) + noise_abs);
            lb(j) = max(0,   lb(j) - noise_abs);
        end
        lb = lb ./ 255;   ub = ub ./ 255;
        target_label = YTest(i);

        w_in = ub - lb;
        fprintf('Input box: max width = %.5f, mean = %.5f, perturbed dims = %d/%d\n', ...
                max(w_in(:)), mean(w_in(w_in > 0)), nnz(w_in), numel(w_in));

        % ------ Sanity: classify center point through full network ------
        try
            x = double((lb + ub) / 2);
            x = net_pre.evaluate(x);
            x = odelayer_NODE_RA.evaluate(x);
            x = layerout.evaluate(x);
            [~, pred_idx] = max(x);
            if pred_idx == double(target_label)
                fprintf('Center-point check: predicts %d  =  target. OK.\n', pred_idx);
            else
                fprintf('[!] Center-point predicts %d  !=  target %d. Verification will fail by definition.\n', ...
                        pred_idx, target_label);
            end
        catch ME
            fprintf('[!] Center-point evaluation failed: %s\n', ME.message);
        end

        % ============================================================
        % STAGE 1: NNV Pre-ODE (layers 1-7)
        % ============================================================
        fprintf('\n[1/3] NNV Pre-ODE reach (layers 1-7)...\n');
        t_pre = tic;
        try
            inpS  = ImageStar(lb, ub);
            R_pre = net_pre.reach(inpS);
        catch ME
            fprintf('  FAILED in Pre-ODE reach: %s\n', ME.message);
            rob(i) = -1;  time_tot(i) = toc(t_total);  continue;
        end
        stage_t(i,1) = toc(t_pre);
        [lb_pre, ub_pre] = star_array_box_hull(R_pre);
        w_pre = ub_pre - lb_pre;
        fprintf('  time: %.2f s | output Stars in array: %d\n', stage_t(i,1), numel(R_pre));
        fprintf('  ODE-input box (676 dims):  max=%.4g, mean=%.4g, sum=%.4g\n', ...
                max(w_pre), mean(w_pre), sum(w_pre));

        % Save first image's ODE-input box for inspection (PJ request)
        if i == 1
            S_ode_in = Star(lb_pre, ub_pre);  %#ok<NASGU>
            fname = sprintf('smoketest_S_ode_in_img%d.mat', i);
            save(fname, 'S_ode_in', 'lb_pre', 'ub_pre');
            fprintf('  Saved ODE-input box to %s\n', fname);
        end

        % ============================================================
        % STAGE 2: TNODEV ODE step (layer 8)
        % ============================================================
        fprintf('\n[2/3] TNODEV ODE reach (layer 8): t=[0,%.2f], %d sub-steps...\n', ...
                ode_params.t_final, ode_params.timeStep);
        t_ode = tic;
        try
            S_ode_out = odelayer_NODE_RA.reach(R_pre);
        catch ME
            fprintf('  FAILED in ODE reach: %s\n', ME.message);
            rob(i) = -1;  time_tot(i) = toc(t_total);  continue;
        end
        stage_t(i,2) = toc(t_ode);
        [lb_ode, ub_ode] = star_array_box_hull(S_ode_out);
        w_ode = ub_ode - lb_ode;

        % Avoid division by zero in inflation factors
        denom = max(w_pre, eps);
        infl  = (w_ode + eps) ./ denom;
        fprintf('  time: %.2f s | output Stars: %d\n', stage_t(i,2), numel(S_ode_out));
        fprintf('  ODE-output box (676 dims): max=%.4g, mean=%.4g, sum=%.4g\n', ...
                max(w_ode), mean(w_ode), sum(w_ode));
        fprintf('  Width inflation Pre->ODE:  median %.2gx, p95 %.2gx, max %.2gx\n', ...
                median(infl), prctile(infl, 95), max(infl));

        % ============================================================
        % STAGE 3: NNV final FC (layer 9)
        % ============================================================
        fprintf('\n[3/3] NNV final FC reach (layer 9)...\n');
        t_fc = tic;
        try
            S_final = layerout.reach(S_ode_out, 'exact-star');
        catch ME
            fprintf('  FAILED in final FC reach: %s\n', ME.message);
            rob(i) = -1;  time_tot(i) = toc(t_total);  continue;
        end
        stage_t(i,3) = toc(t_fc);
        [lb_fin, ub_fin] = star_array_box_hull(S_final);
        fprintf('  time: %.2f s | output Stars: %d\n', stage_t(i,3), numel(S_final));
        fprintf('  Output logit ranges (10 classes):\n');
        for c = 1:10
            tag = '';  if c == double(target_label),  tag = '   <-- target';  end
            fprintf('    class %2d: [%+10.4f, %+10.4f]   width=%.4f%s\n', ...
                    c-1, lb_fin(c), ub_fin(c), ub_fin(c)-lb_fin(c), tag);
        end

        % ============================================================
        % STAGE 4: Robustness check
        % ============================================================
        fprintf('\nRobustness check (target dominates all others)...\n');
        t_chk = tic;
        try
            verdict = check_robustness_strict(S_final, target_label);
        catch ME
            fprintf('  Linprog FAILED: %s\n', ME.message);
            verdict = -1;
        end
        stage_t(i,4) = toc(t_chk);

        rob(i)      = verdict;
        time_tot(i) = toc(t_total);

        switch verdict
            case  1, fprintf('=> ROBUST       (check time: %.2f s)\n', stage_t(i,4));
            case  0, fprintf('=> UNKNOWN      (check time: %.2f s) -- target does not dominate\n', stage_t(i,4));
            case -1, fprintf('=> NUMERICAL FAILURE (check time: %.2f s) -- bounds too wide for linprog\n', stage_t(i,4));
        end

        fprintf('\n--- Image %d done. Total time: %.2f s ---\n', i, time_tot(i));
        fprintf('    Stage breakdown: Pre=%.2fs | ODE=%.2fs | FC=%.2fs | Check=%.2fs\n', ...
                stage_t(i,1), stage_t(i,2), stage_t(i,3), stage_t(i,4));
    end

    %% 4) Summary + save

    fprintf('\n\n========== SUMMARY ==========\n');
    fprintf('Robust:            %d / %d\n', sum(rob == 1),  numT);
    fprintf('Unknown:           %d / %d\n', sum(rob == 0),  numT);
    fprintf('NumericalFail:     %d / %d\n', sum(rob == -1), numT);
    fprintf('Total time:        %.2f s\n',  sum(time_tot));
    fprintf('Mean time / image: %.2f s\n',  mean(time_tot));
    fprintf('=============================\n');

    save('smoketest_results.mat', 'rob', 'time_tot', 'stage_t', ...
         'pix', 'numT', 'noise', 'perturbation');
    fprintf('Results saved to smoketest_results.mat\n');
end


% % =========================================================================
% %  Helper: take an array of Star/ImageStar and return the interval hull
% % =========================================================================
% function [lb, ub] = star_array_box_hull(S)
%     n_sets = numel(S);
%     if n_sets == 0
%         error('star_array_box_hull: empty Star array.');
%     end
%     % First box decides the dimension
%     B = S(1).getBox();
%     n_dim = numel(B.lb);
%     lb_all = zeros(n_dim, n_sets);
%     ub_all = zeros(n_dim, n_sets);
%     lb_all(:,1) = reshape(B.lb, [], 1);
%     ub_all(:,1) = reshape(B.ub, [], 1);
%     for k = 2:n_sets
%         Bk = S(k).getBox();
%         lb_all(:,k) = reshape(Bk.lb, [], 1);
%         ub_all(:,k) = reshape(Bk.ub, [], 1);
%     end
%     lb = min(lb_all, [], 2);
%     ub = max(ub_all, [], 2);
% end


% =========================================================================
%  Helper: take an array of Star/ImageStar and return the interval hull
%  Dispatches on class because:
%    - ImageStar provides getRanges() returning (lb, ub) tensors
%    - Star provides getBoxFast() returning a Box with .lb / .ub fields
% =========================================================================
function [lb, ub] = star_array_box_hull(S)
    n_sets = numel(S);
    if n_sets == 0
        error('star_array_box_hull: empty set array.');
    end

    is_imagestar = isa(S(1), 'ImageStar');

    % First set determines dimension
    [l1, u1] = get_one_box(S(1), is_imagestar);
    n_dim    = numel(l1);

    lb_all = zeros(n_dim, n_sets);
    ub_all = zeros(n_dim, n_sets);
    lb_all(:,1) = l1;
    ub_all(:,1) = u1;

    for k = 2:n_sets
        [lk, uk] = get_one_box(S(k), is_imagestar);
        lb_all(:,k) = lk;
        ub_all(:,k) = uk;
    end

    lb = min(lb_all, [], 2);
    ub = max(ub_all, [], 2);
end


% -------------------------------------------------------------------------
%  Polymorphic single-set range extraction
% -------------------------------------------------------------------------
function [lb_vec, ub_vec] = get_one_box(S, is_imagestar)
    if is_imagestar
        % ImageStar: getRanges() returns tensor-shaped (e.g. 13x13x4) bounds
        [lb_t, ub_t] = S.getRanges();
        lb_vec = reshape(lb_t, [], 1);
        ub_vec = reshape(ub_t, [], 1);
    else
        % Star: prefer getBoxFast; fall back to getBox if not available
        if ismethod(S, 'getBoxFast')
            B = S.getBoxFast();
        else
            B = S.getBox();
        end
        lb_vec = reshape(B.lb, [], 1);
        ub_vec = reshape(B.ub, [], 1);
    end
end


% =========================================================================
%  Helper: strict robustness check
%   Returns  1 if target STRICTLY dominates every other class on every Star
%            0 if some adversarial halfspace is non-empty (potential violation)
%           -1 if linprog crashed (bounds too wide / numerically unstable)
% =========================================================================
function verdict = check_robustness_strict(S, target)
    verdict = 1;
    n_class = 10;
    target_d = double(target);
    for i = 1:numel(S)
        sset = S(i);
        if isempty(sset),  continue;  end
        for adv = 1:n_class
            if adv == target_d,  continue;  end
            c = zeros(1, n_class);
            c(target_d) = 1;
            c(adv)      = -1;
            try
                s_unsafe = sset.intersectHalfSpace(c, 0);
            catch ME
                % linprog crashed -> numerically unsafe
                rethrow(ME);
            end
            if ~isempty(s_unsafe)
                verdict = 0;
                return;
            end
        end
    end
end


% =========================================================================
%  Tiny ternary helper (MATLAB lacks one)
% =========================================================================
function r = ternary(cond, a, b)
    if cond, r = a; else, r = b; end
end