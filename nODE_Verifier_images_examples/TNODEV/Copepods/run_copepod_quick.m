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
%%
% function run_copepod_quick()
% % Copepod Neural ODE verification using TNODEV
% % Adapted from run_mnist_quick.m
% %
% % Parameters
% pix = 1;          % number of pixels per image to attack (max: 768 = 16*16*3)
% numT = 5;         % number of copepod images to test
% noise = 0.5/255;  % L_inf_norm radius (will be scaled by 255 inside the verification function)
% rng(2022);        % Set random seed
% 
% % --- CHOOSE HEURISTIC HERE ---
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% splitting_heuristic = 'naive';
% % -----------------------------
% 
% % Load copepod test images
% load('copepod_test_images_tiny.mat');  % contains: images [16,16,3,20], labels [1,20]
% 
% % images is [H, W, C, N] = [16, 16, 3, 20]
% % labels is 1-indexed (copepod = 2)
% 
% % Build test set with numT images: shape 16x16x3xnumT
% % XTest = zeros(16, 16, 3, numT, 'single');
% % YTest = zeros(numT, 1, 'single');
% 
% % cast the images to double to fix the ode45 warnings
% XTest = zeros(16, 16, 3, numT, 'double');
% YTest = zeros(numT, 1, 'double');
% 
% for k = 1:numT
%     YTest(k) = labels(k);               % copepod label = 2
%     XTest(:,:,:,k) = images(:,:,:,k);
% end
% 
% fprintf('Data loaded. Running copepod NODE_RA verification on %d images...\n', numT);
% fprintf('  Pixels attacked: %d\n', pix);
% fprintf('  Noise: %.6f (pre-scaling)\n', noise);
% fprintf('  Heuristic: %s\n\n', splitting_heuristic);
% 
% % Reach computation using TNODEV
% reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);
% 
% end
%%

% function run_copepod_quick()
% % Copepod Neural ODE verification using TNODEV
% % Adapted from run_mnist_quick.m
% 
% 
% % === DIAGNOSTIC: Check if clean image classifies correctly ===
% load('odenet_tiny_copepod_weights.mat');
% load('copepod_test_images_tiny.mat');
% 
% img = double(images(:,:,:,1));  % first copepod image, clean, no perturbation
% fprintf('Image size: %s\n', mat2str(size(img)));
% fprintf('Image range: [%.4f, %.4f]\n', min(img(:)), max(img(:)));
% fprintf('True label: %d (copepod)\n', labels(1));
% 
% 
% % === DIAGNOSTIC: Run clean forward pass through full network ===
% fprintf('\n--- Clean Forward Pass Diagnostic ---\n');
% 
% % Build pre-ODE network (same as in reach function)
% w1 = pre_conv1_W;
% b1_l = reshape(pre_conv1_b, [1, 1, 2]);
% layer1 = Conv2DLayer(w1, b1_l, [1,1,1,1], [1,1], [1,1]);
% w2_l = reshape(pre_bn1_weight, [1 1 2]);
% b2_l = reshape(pre_bn1_bias, [1 1 2]);
% m2_l = reshape(pre_bn1_running_mean, [1 1 2]);
% v2_l = reshape(pre_bn1_running_var, [1 1 2]);
% layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l,'TrainedMean',m2_l,'TrainedVariance',v2_l,'NumChannels',2,'Epsilon',2);
% layer3 = ReluLayer;
% w4_l = pre_conv2_W;
% b4_l = reshape(pre_conv2_b, [1, 1, 2]);
% layer4 = Conv2DLayer(w4_l, b4_l, [1,1,1,1], [2,2], [1,1]);
% w5_l = reshape(pre_bn2_weight, [1 1 2]);
% b5_l = reshape(pre_bn2_bias, [1 1 2]);
% m5_l = reshape(pre_bn2_running_mean, [1 1 2]);
% v5_l = reshape(pre_bn2_running_var, [1 1 2]);
% layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l,'TrainedMean',m5_l,'TrainedVariance',v5_l,'NumChannels',2,'Epsilon',2);
% layer6 = ReluLayer;
% layer7 = FlattenLayer;
% layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});
% 
% % ODE layer
% global W1_ode b1_ode W2_ode b2_ode
% W1_ode = ode_W1; b1_ode = ode_b1'; W2_ode = ode_W2; b2_ode = ode_b2';
% ode_p = struct(); ode_p.system_choice = 19; ode_p.t_final = 1; ode_p.timeStep = 20;
% ode_p.W1 = W1_ode; ode_p.b1 = b1_ode; ode_p.W2 = W2_ode; ode_p.b2 = b2_ode;
% ode_layer = TNODEVODELayer(ode_p);
% 
% % Post-ODE
% fc1 = FullyConnectedLayer(post_fc1_W, post_fc1_b');
% relu_l = ReluLayer;
% fc2 = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
% % Forward pass on clean image
% x = double(img);
% fprintf('Input to network: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = net_pre.evaluate(x);
% fprintf('After pre-ODE (flatten): size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = ode_layer.evaluate(x);
% fprintf('After ODE: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = fc1.evaluate(x);
% x = relu_l.evaluate(x);
% x = fc2.evaluate(x);
% fprintf('Final output (7 classes): %s\n', mat2str(x', 4));
% 
% [~, pred] = max(x);
% class_labels = {'bubble','copepod','diatom_chain','faecal_pellets','oil','oily_gas','other'};
% fprintf('MATLAB prediction: %d (%s)\n', pred, class_labels{pred});
% fprintf('True label:        %d (%s)\n', labels(1), class_labels{labels(1)});
% 
% if pred == labels(1)
%     fprintf('MATCH — clean image classifies correctly in MATLAB.\n');
% else
%     fprintf('MISMATCH — this is the root cause of falsification!\n');
% end
% fprintf('--- End Diagnostic ---\n\n');
% 
% 
% % Parameters
% pix = 1;          % number of pixels per image to attack (max: 192 = 8*8*3)
% numT = 1;         % number of images to test
% noise = 0.5/255;  % L_inf_norm radius (will be scaled by 255 inside the verification function)
% rng(2022);        % Set random seed
% 
% % --- CHOOSE HEURISTIC HERE ---
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% splitting_heuristic = 'naive';
% % -----------------------------
% 
% % Load copepod test images
% % load('copepod_test_images_tiny.mat');  % contains: images [8,8,3,20], labels [1,20]
% 
% % Build test set with numT images: shape 8x8x3xnumT
% XTest = zeros(8, 8, 3, numT, 'double');
% YTest = zeros(numT, 1, 'double');
% 
% for k = 1:numT
%     YTest(k) = labels(k);
%     XTest(:,:,:,k) = double(images(:,:,:,k));
% end
% 
% fprintf('Data loaded. Running copepod NODE_RA verification on %d images...\n', numT);
% 
% % Reach computation (copepod tiny network, using TNODEV)
% reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);
% 
% end

%% % %-------------1st nODE Network `IMG_SIZE = 8`-------------% %         |------> Working verification result 


function run_copepod_quick()
% Copepod Neural ODE verification using TNODEV
% Adapted from run_mnist_quick.m

% To suppress the warnings
warning('off', 'MATLAB:odearguments:InconsistentDataType');


% === Load data and weights ===
load('odenet_tiny_copepod_weights.mat');
load('copepod_test_images_tiny.mat');

% === DIAGNOSTIC: Check basic image info ===
img = double(images(:,:,:,1));
fprintf('Image size: %s\n', mat2str(size(img)));
fprintf('Image range: [%.4f, %.4f]\n', min(img(:)), max(img(:)));
fprintf('True label: %d (copepod)\n', labels(1));
% === DIAGNOSTIC: Check basic image info ===

% === DIAGNOSTIC: Run clean forward pass with permutation fix ===
fprintf('\n--- Clean Forward Pass Diagnostic ---\n');

% Build permutation (same as reach function)
H_flat = 4; W_flat = 4; C_flat = 2;
n_flat = H_flat * W_flat * C_flat;
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

% Apply permutation to weights
ode_W1_p = ode_W1(:, perm);
ode_W2_p = ode_W2(perm, :);
ode_b2_p = ode_b2(perm);
post_fc1_W_p = post_fc1_W(:, perm);

% Build pre-ODE network
w1 = pre_conv1_W;
b1_l = reshape(pre_conv1_b, [1, 1, 2]);
layer1 = Conv2DLayer(w1, b1_l, [1,1,1,1], [1,1], [1,1]);
w2_l = reshape(pre_bn1_weight, [1 1 2]);
b2_l = reshape(pre_bn1_bias, [1 1 2]);
m2_l = reshape(pre_bn1_running_mean, [1 1 2]);
v2_l = reshape(pre_bn1_running_var, [1 1 2]);
% layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l,'TrainedMean',m2_l,'TrainedVariance',v2_l,'NumChannels',2,'Epsilon',2);
layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l,'TrainedMean',m2_l,'TrainedVariance',v2_l,'NumChannels',2,'Epsilon',pre_bn1_eps);
layer3 = ReluLayer;
w4_l = pre_conv2_W;
b4_l = reshape(pre_conv2_b, [1, 1, 2]);
layer4 = Conv2DLayer(w4_l, b4_l, [1,1,1,1], [2,2], [1,1]);
w5_l = reshape(pre_bn2_weight, [1 1 2]);
b5_l = reshape(pre_bn2_bias, [1 1 2]);
m5_l = reshape(pre_bn2_running_mean, [1 1 2]);
v5_l = reshape(pre_bn2_running_var, [1 1 2]);
% layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l,'TrainedMean',m5_l,'TrainedVariance',v5_l,'NumChannels',2,'Epsilon',2);
layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l,'TrainedMean',m5_l,'TrainedVariance',v5_l,'NumChannels',2,'Epsilon',pre_bn2_eps);
layer6 = ReluLayer;
layer7 = FlattenLayer;
layer7.Type = 'nnet.cnn.layer.FlattenLayer';
net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});

% ODE layer with permuted weights
global W1_ode b1_ode W2_ode b2_ode
W1_ode = ode_W1_p;
b1_ode = ode_b1';
W2_ode = ode_W2_p;
b2_ode = ode_b2_p';
ode_p = struct();
ode_p.system_choice = 19;
ode_p.t_final = 1;
ode_p.timeStep = 20;
ode_p.W1 = W1_ode;
ode_p.b1 = b1_ode;
ode_p.W2 = W2_ode;
ode_p.b2 = b2_ode;
ode_layer = TNODEVODELayer(ode_p);

% Post-ODE with permuted fc1
fc1 = FullyConnectedLayer(post_fc1_W_p, post_fc1_b');
relu_l = ReluLayer;
fc2 = FullyConnectedLayer(post_fc2_W, post_fc2_b');

% Forward pass on clean image
x = double(img);
fprintf('Input to network: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = net_pre.evaluate(x);
fprintf('After pre-ODE (flatten): size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = ode_layer.evaluate(x);
fprintf('After ODE: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = fc1.evaluate(x);
x = relu_l.evaluate(x);
x = fc2.evaluate(x);
fprintf('Final output (7 classes): %s\n', mat2str(x', 4));

[~, pred] = max(x);
class_labels = {'bubble','copepod','diatom_chain','faecal_pellets','oil','oily_gas','other'};
fprintf('MATLAB prediction: %d (%s)\n', pred, class_labels{pred});
fprintf('True label:        %d (%s)\n', labels(1), class_labels{labels(1)});

if pred == labels(1)
    fprintf('MATCH — clean image classifies correctly in MATLAB.\n');
else
    fprintf('MISMATCH — this is the root cause of falsification!\n');
end
fprintf('--- End Diagnostic ---\n\n');
% === DIAGNOSTIC: Run clean forward pass withb permutation fix ===

% === Verification Parameters ===
pix = 192;          % number of pixels per image to attack (max: 192 = 8*8*3)
numT = 1;         % number of images to test
noise = 5/255;  % L_inf_norm radius (images already in [0,1])
rng(2022);        % Set random seed

% --- CHOOSE HEURISTIC HERE ---
% Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
splitting_heuristic = 'naive';
% -----------------------------

% Build test set with numT images: shape 8x8x3xnumT
XTest = zeros(8, 8, 3, numT, 'double');
YTest = zeros(numT, 1, 'double');

for k = 1:numT
    YTest(k) = labels(k);
    XTest(:,:,:,k) = double(images(:,:,:,k));
end

fprintf('Data loaded. Running copepod NODE_RA verification on %d images...\n', numT);

% Reach computation (copepod tiny network, using TNODEV)
reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);

end

%% % %-------------2nd nODE Network `IMG_SIZE = 16`-------------% %        |------> No verification or Falsification result 

% function run_copepod_quick()
% % Copepod Neural ODE verification using TNODEV
% % Adapted from run_mnist_quick.m
% 
% % To suppress the warnings
% warning('off', 'MATLAB:odearguments:InconsistentDataType');
% 
% 
% % === Load data and weights ===
% load('odenet_tiny_copepod_weights.mat');
% load('copepod_test_images_tiny.mat');
% 
% % === DIAGNOSTIC: Check basic image info ===
% img = double(images(:,:,:,1));
% fprintf('Image size: %s\n', mat2str(size(img)));
% fprintf('Image range: [%.4f, %.4f]\n', min(img(:)), max(img(:)));
% fprintf('True label: %d (copepod)\n', labels(1));
% % === DIAGNOSTIC: Check basic image info ===
% 
% % === DIAGNOSTIC: Run clean forward pass with permutation fix ===
% fprintf('\n--- Clean Forward Pass Diagnostic ---\n');
% 
% % Build permutation (same as reach function)
% H_flat = 8; W_flat = 8; C_flat = 4;
% n_flat = H_flat * W_flat * C_flat;
% perm = zeros(n_flat, 1);
% for c = 0:C_flat-1
%     for w = 0:W_flat-1
%         for h = 0:H_flat-1
%             ml_idx = h + H_flat*w + H_flat*W_flat*c + 1;
%             pt_idx = c*H_flat*W_flat + h*W_flat + w + 1;
%             perm(ml_idx) = pt_idx;
%         end
%     end
% end
% 
% % Apply permutation to weights
% ode_W1_p = ode_W1(:, perm);
% ode_W2_p = ode_W2(perm, :);
% ode_b2_p = ode_b2(perm);
% post_fc1_W_p = post_fc1_W(:, perm);
% 
% % Build pre-ODE network
% w1 = pre_conv1_W;
% b1_l = reshape(pre_conv1_b, [1, 1, 4]);
% layer1 = Conv2DLayer(w1, b1_l, [1,1,1,1], [1,1], [1,1]);
% w2_l = reshape(pre_bn1_weight, [1 1 4]);
% b2_l = reshape(pre_bn1_bias, [1 1 4]);
% m2_l = reshape(pre_bn1_running_mean, [1 1 4]);
% v2_l = reshape(pre_bn1_running_var, [1 1 4]);
% layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l,'TrainedMean',m2_l,'TrainedVariance',v2_l,'NumChannels',4,'Epsilon',pre_bn1_eps);
% layer3 = ReluLayer;
% w4_l = pre_conv2_W;
% b4_l = reshape(pre_conv2_b, [1, 1, 4]);
% layer4 = Conv2DLayer(w4_l, b4_l, [1,1,1,1], [2,2], [1,1]);
% w5_l = reshape(pre_bn2_weight, [1 1 4]);
% b5_l = reshape(pre_bn2_bias, [1 1 4]);
% m5_l = reshape(pre_bn2_running_mean, [1 1 4]);
% v5_l = reshape(pre_bn2_running_var, [1 1 4]);
% layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l,'TrainedMean',m5_l,'TrainedVariance',v5_l,'NumChannels',4,'Epsilon',pre_bn2_eps);
% layer6 = ReluLayer;
% layer7 = FlattenLayer;
% layer7.Type = 'nnet.cnn.layer.FlattenLayer';
% net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});
% 
% % ODE layer with permuted weights
% global W1_ode b1_ode W2_ode b2_ode
% W1_ode = ode_W1_p;
% b1_ode = ode_b1';
% W2_ode = ode_W2_p;
% b2_ode = ode_b2_p';
% ode_p = struct();
% ode_p.system_choice = 19;
% ode_p.t_final = 1;
% ode_p.timeStep = 20;
% ode_p.W1 = W1_ode;
% ode_p.b1 = b1_ode;
% ode_p.W2 = W2_ode;
% ode_p.b2 = b2_ode;
% ode_layer = TNODEVODELayer(ode_p);
% 
% % Post-ODE with permuted fc1
% fc1 = FullyConnectedLayer(post_fc1_W_p, post_fc1_b');
% relu_l = ReluLayer;
% fc2 = FullyConnectedLayer(post_fc2_W, post_fc2_b');
% 
% % Forward pass on clean image
% x = double(img);
% fprintf('Input to network: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = net_pre.evaluate(x);
% fprintf('After pre-ODE (flatten): size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = ode_layer.evaluate(x);
% fprintf('After ODE: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));
% 
% x = fc1.evaluate(x);
% x = relu_l.evaluate(x);
% x = fc2.evaluate(x);
% fprintf('Final output (7 classes): %s\n', mat2str(x', 4));
% 
% [~, pred] = max(x);
% class_labels = {'bubble','copepod','diatom_chain','faecal_pellets','oil','oily_gas','other'};
% fprintf('MATLAB prediction: %d (%s)\n', pred, class_labels{pred});
% fprintf('True label:        %d (%s)\n', labels(1), class_labels{labels(1)});
% 
% if pred == labels(1)
%     fprintf('MATCH — clean image classifies correctly in MATLAB.\n');
% else
%     fprintf('MISMATCH — this is the root cause of falsification!\n');
% end
% fprintf('--- End Diagnostic ---\n\n');
% % === DIAGNOSTIC: Run clean forward pass withb permutation fix ===
% 
% % === Verification Parameters ===
% pix = 1;          % number of pixels per image to attack (max: 192 = 8*8*3)
% numT = 1;         % number of images to test
% noise = 0.1/255;  % L_inf_norm radius (images already in [0,1])
% rng(2022);        % Set random seed
% 
% % --- CHOOSE HEURISTIC HERE ---
% % Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
% splitting_heuristic = 'naive';
% % -----------------------------
% 
% % Build test set with numT images: shape 8x8x3xnumT
% XTest = zeros(16, 16, 3, numT, 'double');
% YTest = zeros(numT, 1, 'double');
% 
% for k = 1:numT
%     YTest(k) = labels(k);
%     XTest(:,:,:,k) = double(images(:,:,:,k));
% end
% 
% fprintf('Data loaded. Running copepod NODE_RA verification on %d images...\n', numT);
% 
% % Reach computation (copepod tiny network, using TNODEV)
% reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);
% 
% end