% reach_cnn_medium_NODE_RA_expm.m
% =========================================================================
% L_inf verification for MNIST MEDIUM ODE-CNN with closed-form
% matrix-exponential interval reach for the linear ODE block.
%
% Pipeline:
%   ImageStar -> NNV Pre-ODE (1-7) 
%             -> expm interval reach (layer 8)
%             -> NNV final FC (9)  -> robustness check
%
% Mirrors the tiny version structurally; differences are channel count (10
% vs 4), ODE state dimension (1690 vs 676), and network file.
%
% Image 1 prints full diagnostics; images 2..N print one line each.
%
% fast_bounds (bool):
%   false  - getRanges() on R_pre (LP-based; tight; slow)
%   true   - ImageStar -> Star -> getBoxFast() (faster, looser; SOUND).
%            Falls back to LP path automatically on conversion failure.
%
% Outputs:
%   smoketest_expm_medium_S_ode_in_img1.mat
%   smoketest_expm_medium_results.mat
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

function reach_cnn_medium_NODE_RA_expm( ...
        pix, numT, noise, XTest, YTest, perturbation, fast_bounds)

    if nargin < 7, fast_bounds = false; end

    %% 1) Construct network

    file_path = 'odecnn_mnist_mid2.mat';
    load(file_path); 

    w1 = permute(Wb{1},[4 3 2 1]);                                       
    b1 = reshape(Wb{2}, [1, 1, 10]);
    layer1 = Conv2DLayer(w1, b1, [0,0,0,0], [1,1], [1,1]);

    w2 = reshape(Wb{3}, [1 1 10]);   b2 = reshape(Wb{4},  [1 1 10]);
    m2 = reshape(Wb{15},[1 1 10]);   v2 = reshape(Wb{16}, [1 1 10]);
    layer2 = BatchNormalizationLayer('Offset', b2, 'Scale', w2, ...
        'TrainedMean', m2, 'TrainedVariance', v2, 'NumChannels', 10, 'Epsilon', 10);

    layer3 = ReluLayer;

    w4 = permute(Wb{5}, [4 3 2 1]);
    b4 = reshape(Wb{6}, [1, 1, 10]);
    layer4 = Conv2DLayer(w4, b4, [1,1,1,1], [2,2], [1,1]);

    w5 = reshape(Wb{7}, [1 1 10]);   b5 = reshape(Wb{8},  [1 1 10]);
    m5 = reshape(Wb{18},[1 1 10]);   v5 = reshape(Wb{19}, [1 1 10]);
    layer5 = BatchNormalizationLayer('Offset', b5, 'Scale', w5, ...
        'TrainedMean', m5, 'TrainedVariance', v5, 'NumChannels', 10, 'Epsilon', 10);

    layer6 = ReluLayer;
    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});

    w_ode1 = Wb{9};   b_ode1 = Wb{10}';
    w_ode2 = Wb{11};  b_ode2 = Wb{12}';
    Aout = w_ode2 * w_ode1;            % 1690 x 1690
    Bout = w_ode2 * b_ode1 + b_ode2;   % 1690 x 1

    t_final = 1;

    layerout = FullyConnectedLayer(Wb{13}, Wb{14}');

    %% 2) Precompute the propagator (one-time)

    fprintf('\n========== TNODEV MNIST SMOKE TEST (expm linear reach, MEDIUM) ==========\n');
    fprintf('Settings:  pix=%d (%s) | numT=%d | noise=%.6f (= %.4f/255)\n', ...
            pix, ternary(pix==784, 'FULL IMAGE', 'partial'), numT, noise, noise*255);
    fprintf('ODE: linear, dim=%d, t_final=%g (single expm step)\n', size(Aout,1), t_final);
    fprintf('Bound extraction at ODE input: %s\n', ...
            ternary(fast_bounds, 'getBoxFast() [fast, looser]', 'getRanges() [LP, tight]'));

    eigs_A   = eig(Aout);
    max_real = max(real(eigs_A));
    fprintf('Aout: dim %dx%d | max Re(eig) = %+.4f | ||Aout||_inf = %.4f\n', ...
            size(Aout,1), size(Aout,2), max_real, norm(Aout, 'inf'));
    fprintf('Bout: dim %dx1   | ||Bout||_inf = %.4f\n', size(Bout,1), norm(Bout, 'inf'));

    n = size(Aout, 1);
    A_aug = [Aout, Bout; zeros(1, n+1)];

    fprintf('Building propagator: expm(A_aug * %g), A_aug is %dx%d ...\n', t_final, n+1, n+1);
    t_expm = tic;
    E_aug = expm(A_aug * t_final);
    expm_time = toc(t_expm);

    if any(~isfinite(E_aug(:)))
        error('expm produced non-finite entries; matrix is too ill-conditioned.');
    end

    M = E_aug(1:n, 1:n);
    c = E_aug(1:n, n+1);
    M_pos = max(M, 0);
    M_neg = min(M, 0);

    fprintf('  expm time: %.3f s\n', expm_time);
    fprintf('  Propagator M:  ||M||_inf = %.4f, max|M_ij| = %.4g, range = [%.4g, %.4g]\n', ...
            norm(M, 'inf'), max(abs(M(:))), min(M(:)), max(M(:)));
    fprintf('  Drift c:       ||c||_inf = %.4f, ||c||_2 = %.4f\n', ...
            norm(c, 'inf'), norm(c, 2));
    fprintf('=========================================================================\n');

    %% 3) Prepare attack and storage

    noise_abs    = noise * 255;
    pixels_attack = randperm(784, pix);

    rob       = zeros(numT, 1);
    time_tot  = zeros(numT, 1);
    stage_t   = zeros(numT, 4);
    bound_t   = zeros(numT, 1);

    %% 4) Main loop

    for i = 1:numT
        is_diag = (i == 1);

        if is_diag
            fprintf('\n--- Image %d / %d (label = %d) ---\n', i, numT, YTest(i));
        end
        t_total = tic;

        img_flat = XTest(:,:,:,i)';
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

        if is_diag
            w_in = ub - lb;
            fprintf('Input box: max width = %.5f, mean = %.5f, perturbed dims = %d/%d\n', ...
                    max(w_in(:)), mean(w_in(w_in > 0)), nnz(w_in), numel(w_in));
            try
                x = double((lb + ub) / 2);
                x_pre_eval = double(net_pre.evaluate(x));
                x_pre_flat = reshape(x_pre_eval, [], 1);
                x_ode_expm = M * x_pre_flat + c;
                opts = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);
                [~, x_traj] = ode45(@(tt, xx) Aout * xx + Bout, ...
                                    [0, t_final], x_pre_flat, opts);
                x_ode_ref = x_traj(end, :)';
                center_err = norm(x_ode_expm - x_ode_ref, inf);
                fprintf('Center-point validation: ||M*x+c - ode45(.)||_inf = %.3e\n', center_err);
                x_final = layerout.evaluate(x_ode_expm);
                [~, pred_idx] = max(x_final);
                if pred_idx == double(target_label)
                    fprintf('Center-point classification: predicts %d  =  target. OK.\n', pred_idx);
                else
                    fprintf('[!] Center predicts %d != target %d.\n', pred_idx, target_label);
                end
            catch ME
                fprintf('[!] Center-point evaluation failed: %s\n', ME.message);
            end
        end

        % ============================================================
        % STAGE 1: NNV Pre-ODE reach
        % ============================================================
        if is_diag, fprintf('\n[1/3] NNV Pre-ODE reach (layers 1-7)...\n'); end
        t_pre = tic;
        try
            inpS  = ImageStar(lb, ub);
            R_pre = net_pre.reach(inpS);
        catch ME
            fprintf('  FAILED in Pre-ODE reach: %s\n', ME.message);
            rob(i) = -1;  time_tot(i) = toc(t_total);  continue;
        end
        stage_t(i,1) = toc(t_pre);

        t_bnd = tic;
        [lb_pre, ub_pre, used_path] = extract_pre_ode_bounds(R_pre, fast_bounds, is_diag);
        bound_t(i) = toc(t_bnd);

        if is_diag
            w_pre = ub_pre - lb_pre;
            fprintf('  reach time: %.2f s | bound-extract time: %.2f s (%s)\n', ...
                    stage_t(i,1), bound_t(i), used_path);
            fprintf('  output Stars in array: %d\n', numel(R_pre));
            fprintf('  ODE-input box (%d dims):  max=%.4g, mean=%.4g, sum=%.4g\n', ...
                    numel(w_pre), max(w_pre), mean(w_pre), sum(w_pre));

            S_ode_in = Star(lb_pre, ub_pre); 
            fname = sprintf('smoketest_expm_medium_S_ode_in_img%d.mat', i);
            save(fname, 'S_ode_in', 'lb_pre', 'ub_pre');
            fprintf('  Saved ODE-input box to %s\n', fname);
        end

        % ============================================================
        % STAGE 2: ODE reach via expm
        % ============================================================
        if is_diag, fprintf('\n[2/3] ODE reach via expm (closed-form linear)...\n'); end
        t_ode = tic;
        lb_ode = M_pos * lb_pre + M_neg * ub_pre + c;
        ub_ode = M_pos * ub_pre + M_neg * lb_pre + c;
        stage_t(i,2) = toc(t_ode);

        if any(lb_ode > ub_ode)
            mid = (lb_ode + ub_ode) / 2;
            lb_ode = min(lb_ode, mid);
            ub_ode = max(ub_ode, mid);
        end

        if is_diag
            w_ode = ub_ode - lb_ode;
            w_pre_diag = ub_pre - lb_pre;
            fprintf('  time: %.4f s\n', stage_t(i,2));
            fprintf('  ODE-output box (%d dims): max=%.4g, mean=%.4g, sum=%.4g\n', ...
                    numel(w_ode), max(w_ode), mean(w_ode), sum(w_ode));
            denom_mask = w_pre_diag > 1e-6;
            if any(denom_mask)
                infl = w_ode(denom_mask) ./ w_pre_diag(denom_mask);
                fprintf('  Width inflation (over %d dims with w_pre > 1e-6):  median %.2gx, p95 %.2gx, max %.2gx\n', ...
                        nnz(denom_mask), median(infl), prctile(infl, 95), max(infl));
            end
            if sum(w_pre_diag) > 0
                fprintf('  Aggregate expansion:  sum(w_ode)/sum(w_pre) = %.2fx,  max/max = %.2fx\n', ...
                        sum(w_ode)/sum(w_pre_diag), max(w_ode)/max(w_pre_diag));
            end
        end

        S_ode_out = Star(lb_ode, ub_ode);

        % ============================================================
        % STAGE 3: NNV final FC reach
        % ============================================================
        if is_diag, fprintf('\n[3/3] NNV final FC reach (layer 9)...\n'); end
        t_fc = tic;
        try
            S_final = layerout.reach(S_ode_out, 'exact-star');
        catch ME
            fprintf('  FAILED in final FC reach: %s\n', ME.message);
            rob(i) = -1;  time_tot(i) = toc(t_total);  continue;
        end
        stage_t(i,3) = toc(t_fc);

        [lb_fin, ub_fin] = star_array_box_hull(S_final);

        target_d   = double(target_label);
        others     = setdiff(1:10, target_d);
        margin_box = lb_fin(target_d) - max(ub_fin(others));

        if is_diag
            fprintf('  time: %.2f s | output Stars: %d\n', stage_t(i,3), numel(S_final));
            fprintf('  Output logit ranges (10 classes):\n');
            for cls = 1:10
                tag = '';  if cls == target_d,  tag = '   <-- target';  end
                fprintf('    class %2d: [%+10.4f, %+10.4f]   width=%.4f%s\n', ...
                        cls-1, lb_fin(cls), ub_fin(cls), ub_fin(cls)-lb_fin(cls), tag);
            end
            fprintf('  Box-hull margin (target_lb - max(other_ub)) = %+.4f%s\n', ...
                    margin_box, ternary(margin_box > 0, '   (already dominates)', ''));
        end

        % ============================================================
        % STAGE 4: Robustness check
        % ============================================================
        if is_diag, fprintf('\nRobustness check (target dominates all others, full Star)...\n'); end
        t_chk = tic;
        if margin_box > 0
            verdict = 1;
        else
            try
                verdict = check_robustness_strict(S_final, target_label);
            catch ME
                if is_diag, fprintf('  Linprog failed: %s\n', ME.message); end
                verdict = -1;
            end
        end
        stage_t(i,4) = toc(t_chk);

        rob(i)      = verdict;
        time_tot(i) = toc(t_total);

        if is_diag
            switch verdict
                case  1, fprintf('=> ROBUST       (check time: %.2f s)\n', stage_t(i,4));
                case  0, fprintf('=> UNKNOWN      (check time: %.2f s) -- target does not dominate\n', stage_t(i,4));
                case -1, fprintf('=> NUMERICAL FAILURE (check time: %.2f s)\n', stage_t(i,4));
            end
            fprintf('\n--- Image %d done. Total time: %.2f s ---\n', i, time_tot(i));
            fprintf('    Stage breakdown: Pre=%.2fs | BoundExtract=%.2fs | ODE=%.4fs | FC=%.2fs | Check=%.2fs\n', ...
                    stage_t(i,1), bound_t(i), stage_t(i,2), stage_t(i,3), stage_t(i,4));
        else
            switch verdict
                case  1, label_str = 'ROBUST   ';
                case  0, label_str = 'UNKNOWN  ';
                case -1, label_str = 'NUM-FAIL ';
            end
            fprintf('Image %3d/%d (label=%2d): %s  (%.2f s)\n', ...
                    i, numT, YTest(i), label_str, time_tot(i));
        end
    end

    %% 5) Summary + save

    fprintf('\n\n========== SUMMARY (expm, MEDIUM, fast_bounds=%d) ==========\n', fast_bounds);
    fprintf('Robust:            %d / %d\n', sum(rob == 1),  numT);
    fprintf('Unknown:           %d / %d\n', sum(rob == 0),  numT);
    fprintf('NumericalFail:     %d / %d\n', sum(rob == -1), numT);
    fprintf('Total time:        %.2f s\n',  sum(time_tot));
    fprintf('Mean time / image: %.2f s\n',  mean(time_tot));
    fprintf('  of which bound-extract: mean %.2f s\n', mean(bound_t));
    fprintf('expm precompute:   %.2f s (one-time)\n', expm_time);
    fprintf('============================================================\n');

    save('smoketest_expm_medium_results.mat', 'rob', 'time_tot', 'stage_t', 'bound_t', ...
         'pix', 'numT', 'noise', 'perturbation', 't_final', 'expm_time', 'fast_bounds');
    fprintf('Results saved to smoketest_expm_medium_results.mat\n');
end


% =========================================================================
%  Helper: extract [lb, ub] of R_pre using requested method
% =========================================================================
function [lb, ub, used_path] = extract_pre_ode_bounds(R_pre, fast_bounds, verbose)
    if fast_bounds
        try
            [lb, ub] = star_array_box_hull_fast(R_pre);
            used_path = 'getBoxFast';
            return;
        catch ME
            if verbose
                fprintf('  [!] fast path failed (%s); falling back to LP getRanges()\n', ...
                        ME.message);
            else
                warning('fast_bounds path failed: %s. Falling back to LP getRanges().', ME.message);
            end
        end
    end
    [lb, ub] = star_array_box_hull(R_pre);
    used_path = 'getRanges (LP)';
end


function [lb, ub] = star_array_box_hull_fast(S)
    n_sets = numel(S);
    if n_sets == 0
        error('star_array_box_hull_fast: empty set array.');
    end

    lb_all = [];   ub_all = [];
    for k = 1:n_sets
        Sk = S(k);
        if isa(Sk, 'ImageStar')
            Sk = imagestar_to_star(Sk);
        end
        if ~ismethod(Sk, 'getBoxFast')
            error('Star object lacks getBoxFast() method.');
        end
        B = Sk.getBoxFast();
        lk = reshape(B.lb, [], 1);
        uk = reshape(B.ub, [], 1);
        if k == 1
            n_dim  = numel(lk);
            lb_all = zeros(n_dim, n_sets);
            ub_all = zeros(n_dim, n_sets);
        end
        lb_all(:,k) = lk;
        ub_all(:,k) = uk;
    end
    lb = min(lb_all, [], 2);
    ub = max(ub_all, [], 2);
end


function S = imagestar_to_star(IS)
    if ismethod(IS, 'toStar')
        S = IS.toStar();
        return;
    end
    V = IS.V;
    n_basis = size(V, ndims(V));
    V_flat  = reshape(V, [], n_basis);
    if isprop(IS, 'C') && isprop(IS, 'd') ...
            && isprop(IS, 'pred_lb') && isprop(IS, 'pred_ub')
        S = Star(V_flat, IS.C, IS.d, IS.pred_lb, IS.pred_ub);
    else
        S = Star(V_flat, IS.C, IS.d);
    end
end


% =========================================================================
%  Helper: LP-based interval hull (tight)
% =========================================================================
function [lb, ub] = star_array_box_hull(S)
    n_sets = numel(S);
    if n_sets == 0
        error('star_array_box_hull: empty set array.');
    end

    is_imagestar = isa(S(1), 'ImageStar');

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


function [lb_vec, ub_vec] = get_one_box(S, is_imagestar)
    if is_imagestar
        [lb_t, ub_t] = S.getRanges();
        lb_vec = reshape(lb_t, [], 1);
        ub_vec = reshape(ub_t, [], 1);
    else
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
%  Helper: strict robustness check on a Star array
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
            cvec = zeros(1, n_class);
            cvec(target_d) = 1;
            cvec(adv)      = -1;
            s_unsafe = sset.intersectHalfSpace(cvec, 0);
            if ~isempty(s_unsafe)
                verdict = 0;
                return;
            end
        end
    end
end


% =========================================================================
%  Tiny ternary helper
% =========================================================================
function r = ternary(cond, a, b)
    if cond, r = a; else, r = b; end
end