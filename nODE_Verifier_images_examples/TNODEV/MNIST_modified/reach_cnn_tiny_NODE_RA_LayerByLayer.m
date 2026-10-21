function reach_cnn_tiny_NODE_RA_LayerByLayer(pix, numT, noise, XTest, YTest, perturbation)
% Reachability analysis of tiny MNIST Neural ODE (Layer-by-Layer approach)
% Compares against NNV's integrated "net.reach()" style.
%
% Flow:
%   1. NNV computes Layers 1-7 (Prefix)
%   2. NODE_RA computes Layer 8 (ODE) using Diagonalized TIRA
%   3. NNV computes Layer 9 (Output)

    %% 0) Add Paths
    verifier_root = '/home/ibrahima/Documents/nODE_Verifier/TNODEV';
    addpath(genpath(verifier_root));

    %% 1) Load Weights & Parameters
    file_path = 'odecnn_mnist_tiny.mat';
    load(file_path); 
    
    % Dynamics for Layer 8 (ODE)
    w1_ode = Wb{9};  b1_ode = Wb{10}';
    w2_ode = Wb{11}; b2_ode = Wb{12}';
    Aout = w2_ode * w1_ode;
    Bout = w2_ode * b1_ode + b2_ode;
    
    tfinal = 1.0; 
    timeStep = 0.1; % Step size for TIRA

    %% 2) PRE-COMPUTATION (Optimization)
    % We compute the Eigen-decomposition ONCE here to make the loop fast.
    % This mimics what we did inside the ODELayer_TIRA constructor.
    fprintf('Pre-computing Eigen-decomposition for NODE_RA...\n');
    [V, D] = eig(Aout);
    [V_real, D_real] = cdf2rdf(V, D);
    V_inv = inv(V_real);
    B_tilde = V_inv * Bout;

    % Set Globals required by NODE_RA internally
    global Aout_global Bout_global system_choice
    Aout_global = Aout;
    Bout_global = Bout;
    system_choice = 16;

    %% 3) Build Network Parts
    % --- Part A: Prefix Network (Layers 1-7) ---
    w1 = permute(Wb{1},[4 3 2 1]); b1 = reshape(Wb{2},[1,1,4]);
    L1 = Conv2DLayer(w1,b1,[0,0,0,0],[1,1],[1,1]);
    
    w2 = reshape(Wb{3},[1 1 4]); b2 = reshape(Wb{4},[1 1 4]);
    m2 = reshape(Wb{15},[1 1 4]); v2 = reshape(Wb{16},[1 1 4]);
    L2 = BatchNormalizationLayer('Offset',b2,'Scale',w2,'TrainedMean',m2,'TrainedVariance',v2,'NumChannels',4,'Epsilon',4);
    
    L3 = ReluLayer;
    
    w4 = permute(Wb{5},[4 3 2 1]); b4 = reshape(Wb{6},[1,1,4]);
    L4 = Conv2DLayer(w4,b4,[1,1,1,1],[2,2],[1,1]);
    
    w5 = reshape(Wb{7},[1 1 4]); b5 = reshape(Wb{8},[1 1 4]);
    m5 = reshape(Wb{18},[1 1 4]); v5 = reshape(Wb{19},[1 1 4]);
    L5 = BatchNormalizationLayer('Offset',b5,'Scale',w5,'TrainedMean',m5,'TrainedVariance',v5,'NumChannels',4,'Epsilon',4);
    
    L6 = ReluLayer;
    L7 = FlattenLayer;

    % --- FIX: Explicitly set the layer type ---
    L7.Type = 'nnet.cnn.layer.FlattenLayer';
    % ------------------------------------------

    net_prefix = NN({L1, L2, L3, L4, L5, L6, L7});
    
    % --- Part B: Output Network (Layer 9) ---
    L9 = FullyConnectedLayer(Wb{13}, Wb{14}');
    net_out = NN({L9});

    %% 4) Attack Setup
    pixels_attack = randperm(784, pix);
    time_total = zeros(numT,1);
    rob_results = zeros(numT,1);

    fprintf('Starting Layer-by-Layer Verification on %d images...\n', numT);

    %% 5) Main Verification Loop
    for i = 1:numT
        t_start = tic;
        
        % --- A. Prepare Input Star ---
        img_flat = XTest(:,:,:,i)';
        if max(img_flat) <= 1.0, imgMax = 1.0; noisePix = noise; 
        else, imgMax = 255.0; noisePix = noise * 255; end    

        lb = img_flat; ub = img_flat;
        if strcmp(perturbation,'random') || strcmp(perturbation,'inf')
            for j = pixels_attack
                ub(j) = min(imgMax, ub(j) + noisePix);
                lb(j) = max(0, lb(j) - noisePix);
            end
        else
            error('Wrong perturbation type');
        end

        if imgMax == 255.0, inpS = ImageStar(lb./255, ub./255);
        else, inpS = ImageStar(lb, ub); end
        
        % =========================================================
        % STEP 1: Prefix Reachability (NNV)
        % =========================================================
        % We call reach() on the prefix network.
        % This computes Layers 1 through 7 sequentially.
        try
            R_prefix = net_prefix.reach(inpS, 'approx-star');
        catch
            R_prefix = net_prefix.reach(inpS);
        end
        
        % =========================================================
        % STEP 2: NODE_RA (Your Tool)
        % =========================================================
        % Prepare Parameters
        [lb_node, ub_node] = R_prefix.getRanges();
        params.x0 = 0.5 * (lb_node + ub_node);
        params.Initial_radius = 0.5 * (ub_node - lb_node);
        params.t_final = tfinal;
        params.timeStep = timeStep;
        params.system_choice = 16;
        
        % Pass the Exact Star for Precision
        params.input_star = R_prefix;
        
        % Pass Pre-computed Matrices for Speed (Matches NODE_RA V2.8 logic)
        params.V_real = V_real;
        params.V_inv  = V_inv;
        params.D_real = D_real;
        params.B_tilde = B_tilde;
        
        % Call NODE_RA
        res = NODE_RA(params);
        
        if isfield(res, 'star_set')
            R_node = res.star_set;
        else
            R_node = Star(res.intersect_low, res.intersect_up);
        end
        
        % =========================================================
        % STEP 3: Output Reachability (NNV)
        % =========================================================
        R_final = net_out.reach(R_node);
        
        % =========================================================
        % STEP 4: Robustness Check
        % =========================================================
        rv = net_out.checkRobust(R_final, YTest(i));
        rob_results(i) = rv;
        time_total(i) = toc(t_start);
        
        fprintf('Img %d: Result=%d (Time: %.4fs)\n', i, rv, time_total(i));
    end

    %% 6) Summary
    rob  = sum(rob_results == 1);
    unk  = sum(rob_results == 0);
    notr = sum(rob_results == 2);
    total_t = sum(time_total);

    disp(' ');
    disp('=== LAYER-BY-LAYER SUMMARY ===');
    disp('Robust images: '    + string(rob));
    disp('Unknown images: '   + string(unk));
    disp('Not robust images: '+ string(notr));
    disp('Total time = '      + string(total_t) + ' sec');
end