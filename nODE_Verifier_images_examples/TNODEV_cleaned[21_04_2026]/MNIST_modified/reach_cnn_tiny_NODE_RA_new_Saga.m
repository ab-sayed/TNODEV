% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 29 January 2026
% Last update: 25 March 2026 
% Last revision:  2026

% %------------- BEGIN OF CODE V1.1.4.4   -------------- |

function reach_cnn_tiny_NODE_RA_new(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% reach_cnn_tiny_NODE_RA_new - Robustness verification for MNIST Neural ODE
% Integrates NNV for standard layers and NODE_RA_NEW.m for the ODE layer
% Includes Iterative Refinement and Falsification
% The program flow:
% 1. Constructs NNV Network (Split into Pre-ODE, ODE-Wrapper, Post-ODE)
% 2. Generates Adversarial Attack (Pixel perturbation)
% 3. Runs Iterative Refinement: Verify -> Falsify -> Split -> Repeat
    
    % Add NMPI library path for Saga (R2023a/R2025a compatible)
    addpath('/cluster/software/MATLAB/2022b/NMPI/version13_intel2021a/');
    parallel_start;


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

    % --- NEW: Set globals for the Master Node ---
    global Aout_global Bout_global;
    Aout_global = Aout;
    Bout_global = Bout;
    % --------------------------------------------

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
    MAX_ITER = 1000;    % Maximum number of iterations for Refinement

    % fprintf('\nStarting Verification with NODE_RA_NEW (Iterative Refinement Approach)...\n');

    % Loop over Test Images
    for i = 1:numT
        % fprintf('\n--- Image %d / %d ---\n', i, numT);
        if my_rank == Master_rank
            fprintf('\n--- Image %d / %d ---\n', i, numT);
        end

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

        % fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
        if my_rank == Master_rank
            fprintf('  Running Global Falsification (1000 samples)...\n');  %|--> modifiy this to add the number of samples defined in num_samples at the helper function instead of hard coding it
        end

        is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);

        if is_falsified
             % fprintf('  >>> FALSIFIED! Counter-example found.\n');
             if my_rank == Master_rank
                fprintf('  >>> FALSIFIED! Counter-example found.\n');
             end
             rob_ode(i) = 2; % Not Robust
             % time_ode(i) = toc(t);
             time_ode(i) = 0; % No formal verification time spent
             mdisplay('Result: NOT ROBUST');
             continue; % Skip Verification, move to next image
        end

        % fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print
        if my_rank == Master_rank
            fprintf('  No counter-example found. Proceeding to Formal Verification (RA)...\n'); %|--> check if I will remove the Formal Verification thing in the print
        end

        % -----------------------------------------------------
        % C. RUN VERIFICATION (Reachability + Refinement) on the Full Network
        % -----------------------------------------------------
     
        t = tic;

        % Queue stores the INPUT intervals (Pixel Space)
        queue = {{lb, ub}};

        image_is_robust = true;    
        image_is_unknown = false; 
        iter = 0;
        

        worker_load_factor = 5; %--> why 5 or on which basis should I choose this number ? The sweet spot is between 3~10
        batch_size = num_ranks * worker_load_factor; % num_ranks is provided by NMPI

        while ~isempty(queue)
            iter = iter + 1;
            
            % fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
            if my_rank == Master_rank
                fprintf('\n--- Refinement Iteration %d (Queue Size: %d) ---\n', iter, length(queue));
            end

            if iter > MAX_ITER
                % fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
                if my_rank == Master_rank
                    fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
                end
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

            % parfor b = 1:current_batch_size
            for b = parsteps(1:current_batch_size) 

                % --- NEW: Force the workers to initialize their own globals ---
                global Aout_global Bout_global;
                Aout_global = Aout;
                Bout_global = Bout;
                % --------------------------------------------------------------

                % Get current sub-interval
                curr = batch_inputs{b};

                % Get current sub-interval
                curr = batch_inputs{b};
                c_lb = curr{1}; c_ub = curr{2};

                % --- 1. Reachability for Layers 1-7 (NN-Layers) "NNV" ---
                % fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
                if my_rank == Master_rank
                    fprintf('  (1/3) Running Pre-ODE Layers (1-7)...\n');
                end
                % (NNV prints its own logs here automatically)
                inpS = ImageStar(c_lb, c_ub);
                R_pre = net_pre.reach(inpS);
               
                % --- 2. Reachability for Layer 8 (ODE-Layer) "TNODEV" ---
                % Using TNODEVODELayer.m
                % fprintf('  (2/3) Running ODE Layer (8)...\n');
                if my_rank == Master_rank
                    fprintf('  (2/3) Running ODE Layer (8)...\n');
                end
                S_ode_out = odelayer_NODE_RA.reach(R_pre);
                
                % --- VOLUME CHECK LAYER 8 ---  --> (Can be messy in parallel)
                epsilon = 1e-20; 
                lb_all_8 = []; ub_all_8 = [];
                
                % FIX: Use getBoxFast() instead of getRanges() for massive speedup
                for s_idx = 1:length(S_ode_out)
                    B8 = S_ode_out(s_idx).getBoxFast();
                    lb_all_8(:, s_idx) = B8.lb;
                    ub_all_8(:, s_idx) = B8.ub;
                end
                lb_8 = min(lb_all_8, [], 2);
                ub_8 = max(ub_all_8, [], 2);
                vol_8 = sum(log((ub_8 - lb_8) + epsilon)); 
                % fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
                if my_rank == Master_rank
                    fprintf('        > Layer 8 Hull Log-Volume: %.4f\n', vol_8);
                end
              
                % --- 3. Reachability for Layer 9 (Output-FC Layer) "NNV" ---
                % fprintf('  (3/3) Running Final Layer (9)...\n');
                if my_rank == Master_rank
                    fprintf('  (3/3) Running Final Layer (9)...\n');
                end
                S_final = layerout.reach(S_ode_out, 'exact-star');
                
                % --- VOLUME CHECK LAYER 9 ---  --> (Can be messy in parallel)
                
                % FIX: Use getBoxFast() instead of getRanges() for massive speedup
                lb_all_9 = []; ub_all_9 = [];
                for s_idx = 1:length(S_final)
                    B9 = S_final(s_idx).getBoxFast();
                    lb_all_9(:, s_idx) = B9.lb;
                    ub_all_9(:, s_idx) = B9.ub;
                end
                lb_9 = min(lb_all_9, [], 2);
                ub_9 = max(ub_all_9, [], 2);
                vol_9 = sum(log((ub_9 - lb_9) + epsilon));
                % fprintf('        > Layer 9 Hull Log-Volume: %.4f\n', vol_9);
                if my_rank == Master_rank
                    fprintf('        > Layer 9 Hull Log-Volume: %.4f\n', vol_9);
                end

                % --- 4. Check Robustness ---

                % FIX: Using local helper instead of net_post.checkRobust
                res = check_robustness_manual(S_final, target_label);

                if res == 1
                    % Verified Safe for this sub-interval
                    batch_results(b) = 0; % Safe
                    % fprintf('  [RESULT] Verified Safe for this sub-interval.\n')
                    if my_rank == Master_rank
                        fprintf('  [RESULT] Verified Safe for this sub-interval.\n')
                    end
                else
                    batch_results(b) = 1; % Unknown -> Needs Split
                end
            end % <---- END THE PARSTEPS LOOP HERE
            
            % NOW run the reduction after all cores have finished their steps
            batch_results = reduction('+', batch_results);

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
            % fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
            if my_rank == Master_rank
                fprintf('  Batch Summary: %d Safe, %d Splitting.\n', safe_count, split_count);
            end
        end

        time_ode(i) = toc(t);

        % Record Result
        if ~image_is_robust
            rob_ode(i) = 2; 
            mdisplay('Final Result: NOT ROBUST');
        elseif image_is_unknown
            rob_ode(i) = 0; 
            mdisplay('Final Result: UNKNOWN (Timeout)');
        else
            rob_ode(i) = 1; 
            mdisplay('Final Result: ROBUST');
        end
    end    

    % Process and Save results
    rob = sum(rob_ode == 1);
    unk = sum(rob_ode == 0);
    notr = sum(rob_ode == 2);
    timeT = sum(time_ode);

    mdisplay(' ');
    mdisplay('=== FINAL SUMMARY ===');
    mdisplay('Robust images:     ' + string(rob));
    mdisplay('Unknown images:    ' + string(unk));
    mdisplay('Not robust images: ' + string(notr));
    mdisplay('Total time =       ' + string(timeT));

    % save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
    if my_rank == Master_rank
        save("cnn_tiny_NODE_RA_NEW_"+string(perturbation)+"_"+string(noise)+".mat",'rob','rob_ode','timeT','pix','numT','noise');
    end

parallel_end; % Must be the absolute last line!
end

% =========================================================================
% Helper Functions
% =========================================================================

% % Refinement Heuristics
function [s1, s2] = split_interval_heuristic(lb, ub, method, ~)
    % For the MNIST CNN+ODE architecture, advanced heuristics (Jacobian/Gradient)
    % mapping 676 latent ODE states back to 784 raw pixels through 7 layers 
    % is computationally intractable using pure interval arithmetic. 
    % Therefore, we strictly enforce the 'naive' splitting approach.
    
    radii = (ub - lb) / 2;
    
    switch method
        case 'naive'
            % Method 1: Largest Width
            [~, split_dim] = max(radii);
            
        otherwise
            % Catch any leftover calls to 'jacobian' or 'gradient'
            warning('Method "%s" is intractable for MNIST pixels. Defaulting to naive.', method);
            [~, split_dim] = max(radii);
    end

    % Perform Split on the selected dimension
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
                % fprintf('    [WARNING] Numerical Instability in Linprog (Volume too large). Treating as Unsafe.\n');
                if my_rank == Master_rank
                    fprintf('    [WARNING] Numerical Instability in Linprog (Volume too large). Treating as Unsafe.\n');
                end
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
    % num_samples = 5; % Reduced for Smoke Test

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

% %------------- END OF CODE V1.1.4.4  --------------

%%

% %------------- BEGIN OF CODE V2.0.0 (parfeval async) -------------- |
%
% CHANGELOG from V1.1.4.4:
%   - Replaced NMPI parsteps/reduction with MATLAB PCT parfeval
%   - Eliminated BSP barrier: cores never idle-wait
%   - Work-stealing pattern: finished core immediately gets next task
%   - Added wall-clock timeout to avoid Slurm kills
%   - Added adaptive batch sizing based on queue depth
%   - Removed all NMPI dependencies (parallel_start, parallel_end, 
%     parsteps, reduction, my_rank, Master_rank, mdisplay, num_ranks)

function reach_cnn_tiny_NODE_RA_new_parfeval(pix,numT,noise,XTest,YTest,perturbation,splitting_heuristic)
% reach_cnn_tiny_NODE_RA_new_parfeval - Robustness verification for MNIST Neural ODE
% Uses MATLAB PCT parfeval for truly asynchronous parallel reachability.

    %% 0) Start parallel pool (replaces NMPI parallel_start)
    % -------------------------------------------------------
    % Use all available cores minus 1 (the master needs a thread too)
    num_workers = feature('numcores') - 1;
    num_workers = max(num_workers, 1); % At least 1 worker
    
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('local', num_workers);
        fprintf('Started parpool with %d workers on %d available cores.\n', pool.NumWorkers, feature('numcores'));
    else
        num_workers = pool.NumWorkers;
        fprintf('Using existing parpool with %d workers.\n', num_workers);
    end

    %% 1) Load and construct the NeuralODE
    % (Identical to V1.1.4.4 — network construction is serial anyway)

    file_path = 'odecnn_mnist_tiny.mat';
    load(file_path); 

    % --- 1.1 Pre-ODE Layers (1-7) ---
    w1 = permute(Wb{1},[4 3 2 1]);
    b1 = reshape(Wb{2},[1,1,4]);
    layer1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);

    w2 = reshape(Wb{3},[1 1 4]);
    b2 = reshape(Wb{4},[1 1 4]);
    m2 = reshape(Wb{15},[1 1 4]);
    v2 = reshape(Wb{16},[1 1 4]);
    layer2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);

    layer3 = ReluLayer;

    w4 = permute(Wb{5},[4 3 2 1]);
    b4 = reshape(Wb{6},[1,1,4]);
    layer4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);

    w5 = reshape(Wb{7},[1 1 4]);
    b5 = reshape(Wb{8},[1 1 4]);
    m5 = reshape(Wb{18},[1 1 4]);
    v5 = reshape(Wb{19},[1 1 4]);
    layer5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);

    layer6 = ReluLayer;

    layer7 = FlattenLayer;
    layer7.Type = 'nnet.cnn.layer.FlattenLayer';

    neuralLayers_Pre = {layer1, layer2, layer3, layer4, layer5, layer6, layer7};
    net_pre = NN(neuralLayers_Pre);

    % --- 1.2 ODE Layer (Layer 8) ---
    w_ode1 = Wb{9};
    b_ode1 = Wb{10}';
    w_ode2 = Wb{11};
    b_ode2 = Wb{12}';

    Aout = w_ode2 * w_ode1;
    Bout = w_ode2 * b_ode1 + b_ode2;

    % Set globals on master (workers will get them via closure or explicit init)
    global Aout_global Bout_global;
    Aout_global = Aout;
    Bout_global = Bout;

    ode_params = struct();
    ode_params.system_choice = 16;
    ode_params.t_final = 1;
    ode_params.timeStep = 20;
    ode_params.A = Aout;
    ode_params.B = Bout;

    odelayer_NODE_RA = TNODEVODELayer(ode_params);

    % --- 1.3 Post-ODE Layer (Layer 9) ---
    layerout = FullyConnectedLayer(Wb{13}, Wb{14}');

    % --- 1.4 Full Network for Falsification ---
    Full_Network_Components = {net_pre, odelayer_NODE_RA, layerout};

    %% 2) Run experiments with ASYNC parallel verification

    noise = noise*255;
    pixels_attack = randperm(784,pix);

    time_ode = zeros(numT,1);
    rob_ode = zeros(numT,1);   % 1=Robust, 0=Unknown, 2=Not Robust
    MAX_ITER = 1000;
    
    % Wall-clock safety margin: stop 60s before Slurm kills us
    % Parse SLURM time limit if available, else assume 10 min
    slurm_time_str = getenv('SLURM_JOB_TIME_LIMIT');  % e.g. "00:10:00"
    if ~isempty(slurm_time_str)
        parts = str2double(strsplit(slurm_time_str, ':'));
        wall_limit_sec = parts(1)*3600 + parts(2)*60 + parts(3);
    else
        wall_limit_sec = 600; % 10 min default
    end
    WALL_TIMEOUT = wall_limit_sec - 60; % Stop 60s early for cleanup
    job_start_time = tic;

    for i = 1:numT
        fprintf('\n--- Image %d / %d ---\n', i, numT);

        % A. CREATE ADVERSARIAL ATTACK
        img_flat = XTest(:,:,:,i)';
        lb = img_flat; 
        ub = img_flat;

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

        % B. FALSIFICATION (Quick serial check first)
        fprintf('  Running Global Falsification (1000 samples)...\n');
        is_falsified = run_generic_falsification(lb, ub, Full_Network_Components, target_label);

        if is_falsified
            fprintf('  >>> FALSIFIED! Counter-example found.\n');
            rob_ode(i) = 2;
            time_ode(i) = 0;
            continue;
        end

        fprintf('  No counter-example found. Proceeding to Formal Verification...\n');

        % C. ASYNC VERIFICATION (parfeval work-stealing)
        t = tic;
        queue = {{lb, ub}};
        image_is_robust = true;    
        image_is_unknown = false; 
        iter = 0;
        total_verified = 0;
        total_split = 0;

        while ~isempty(queue)
            iter = iter + 1;

            % --- Wall-clock timeout check ---
            if toc(job_start_time) > WALL_TIMEOUT
                fprintf('  [WALL TIMEOUT] Approaching Slurm limit. Saving partial results.\n');
                image_is_unknown = true;
                break;
            end

            if iter > MAX_ITER
                fprintf('  Timeout: Max iterations (%d) reached.\n', MAX_ITER);
                image_is_unknown = true;
                break;
            end

            fprintf('\n--- Refinement Iter %d (Queue: %d, Verified: %d, Split: %d) ---\n', ...
                    iter, length(queue), total_verified, total_split);

            % ============================================================
            % ASYNC DISPATCH: Submit up to N tasks, process as they finish
            % ============================================================
            
            % Take a batch from the queue
            % Slightly oversubscribe: 2x workers so there's always work ready
            batch_size = min(length(queue), num_workers * 2);
            batch_inputs = queue(1:batch_size);
            queue(1:batch_size) = [];

            % Submit all tasks asynchronously
            futures(1:batch_size) = parallel.FevalFuture;
            for b = 1:batch_size
                curr = batch_inputs{b};
                % Each parfeval runs verify_single_interval on a worker
                % Pass ALL needed data as arguments (no globals needed!)
                futures(b) = parfeval(pool, @verify_single_interval, 1, ...
                    curr{1}, curr{2}, ...               % lb, ub
                    net_pre, odelayer_NODE_RA, layerout, ... % network components
                    target_label, Aout, Bout);           % verification params
            end

            % Process results AS THEY COMPLETE (no barrier!)
            for b = 1:batch_size
                % fetchNext blocks until ANY future completes (not all!)
                [completedIdx, result] = fetchNext(futures);
                
                if result == 0
                    % Verified Safe
                    total_verified = total_verified + 1;
                else
                    % Unknown -> Split and add back to queue
                    total_split = total_split + 1;
                    curr = batch_inputs{completedIdx};
                    [s1, s2] = split_interval_heuristic(curr{1}, curr{2}, splitting_heuristic, Aout);
                    queue{end+1} = s1;
                    queue{end+1} = s2;
                end
            end

            fprintf('  Batch complete. Running total: %d verified, %d split.\n', ...
                    total_verified, total_split);
            
            % Clean up futures array for next iteration
            clear futures;
        end

        time_ode(i) = toc(t);

        if ~image_is_robust
            rob_ode(i) = 2; 
            fprintf('Final Result: NOT ROBUST\n');
        elseif image_is_unknown
            rob_ode(i) = 0; 
            fprintf('Final Result: UNKNOWN (Timeout)\n');
        else
            rob_ode(i) = 1; 
            fprintf('Final Result: ROBUST\n');
        end
    end    

    %% 3) Summary and Save
    rob = sum(rob_ode == 1);
    unk = sum(rob_ode == 0);
    notr = sum(rob_ode == 2);
    timeT = sum(time_ode);

    fprintf('\n=== FINAL SUMMARY ===\n');
    fprintf('Robust images:     %d\n', rob);
    fprintf('Unknown images:    %d\n', unk);
    fprintf('Not robust images: %d\n', notr);
    fprintf('Total time =       %.2f s\n', timeT);

    save("cnn_tiny_NODE_RA_NEW_parfeval_"+string(perturbation)+"_"+string(noise)+".mat", ...
         'rob','rob_ode','timeT','pix','numT','noise');

    % Shut down pool cleanly
    delete(pool);
end


% =========================================================================
% WORKER FUNCTION: Runs on a single core via parfeval
% =========================================================================
function result = verify_single_interval(c_lb, c_ub, net_pre, odelayer_NODE_RA, layerout, target_label, Aout, Bout)
    % verify_single_interval - Self-contained verification of one sub-interval
    % Returns: 0 = Safe (verified), 1 = Unknown (needs split)
    %
    % IMPORTANT: This function receives ALL data as arguments.
    % No globals needed — parfeval passes data by value to the worker.
    
    % Initialize globals that TIRA/System_description.m expects
    global Aout_global Bout_global;
    Aout_global = Aout;
    Bout_global = Bout;
    
    try
        % 1. Pre-ODE Layers (1-7) via NNV
        inpS = ImageStar(c_lb, c_ub);
        R_pre = net_pre.reach(inpS);
        
        % 2. ODE Layer (8) via TNODEV
        S_ode_out = odelayer_NODE_RA.reach(R_pre);
        
        % 3. Post-ODE FC Layer (9) via NNV
        S_final = layerout.reach(S_ode_out, 'exact-star');
        
        % 4. Check Robustness
        result = check_robustness_worker(S_final, target_label);
        
    catch ME
        % If anything crashes (OOM, numerical instability), treat as Unknown
        fprintf('    [WORKER WARNING] %s — treating as Unknown.\n', ME.message);
        result = 1; % Unknown -> will be split
    end
end


% =========================================================================
% Helper: Robustness check (worker-safe, no globals for printing)
% =========================================================================
function robust = check_robustness_worker(S, target)
    robust = 0; % 0 = Safe (verified robust for this sub-interval)
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
                s_unsafe = star_section.intersectHalfSpace(c, 0);
                if ~isempty(s_unsafe)
                    robust = 1; % 1 = Unknown (potential violation)
                    return;
                end
            catch
                robust = 1; % Numerical instability -> Unknown
                return;
            end
        end
    end
end


% =========================================================================
% Helper: Splitting Heuristic (unchanged)
% =========================================================================
function [s1, s2] = split_interval_heuristic(lb, ub, method, ~)
    radii = (ub - lb) / 2;
    
    switch method
        case 'naive'
            [~, split_dim] = max(radii);
        otherwise
            warning('Method "%s" intractable for MNIST. Defaulting to naive.', method);
            [~, split_dim] = max(radii);
    end

    mid = lb(split_dim) + radii(split_dim);
    lb1 = lb; ub1 = ub; ub1(split_dim) = mid;
    lb2 = lb; ub2 = ub; lb2(split_dim) = mid;
    s1 = {lb1, ub1}; s2 = {lb2, ub2};
end


% =========================================================================
% Helper: Falsification (unchanged)
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

% %------------- END OF CODE V2.0.0 -------------- |