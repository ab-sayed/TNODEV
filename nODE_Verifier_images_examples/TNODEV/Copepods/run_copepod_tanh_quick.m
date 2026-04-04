function run_copepod_tanh_quick()
% Copepod Neural ODE (TANH) verification using TNODEV
% Adapted from run_copepod_quick.m for the tanh activation model

% === Load data and weights ===
load('odenet_tanh_8x8_copepod_weights.mat');
load('copepod_test_images_tanh_8x8.mat');

% === DIAGNOSTIC: Check basic image info ===
img = double(images(:,:,:,1));
fprintf('Image size: %s\n', mat2str(size(img)));
fprintf('Image range: [%.4f, %.4f]\n', min(img(:)), max(img(:)));
fprintf('True label: %d (copepod)\n', labels(1));

% === DIAGNOSTIC: Run clean forward pass with permutation fix ===
fprintf('\n--- Clean Forward Pass Diagnostic (TANH model) ---\n');

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

% Build pre-ODE network (TANH activations)
w1 = pre_conv1_W;
b1_l = reshape(pre_conv1_b, [1, 1, 2]);
layer1 = Conv2DLayer(w1, b1_l, [1,1,1,1], [1,1], [1,1]);
w2_l = reshape(pre_bn1_weight, [1 1 2]);
b2_l = reshape(pre_bn1_bias, [1 1 2]);
m2_l = reshape(pre_bn1_running_mean, [1 1 2]);
v2_l = reshape(pre_bn1_running_var, [1 1 2]);
layer2 = BatchNormalizationLayer('Offset',b2_l,'Scale',w2_l,'TrainedMean',m2_l,'TrainedVariance',v2_l,'NumChannels',2,'Epsilon',pre_bn1_eps);
layer3 = TanhLayer;  % <<< TANH instead of ReLU
w4_l = pre_conv2_W;
b4_l = reshape(pre_conv2_b, [1, 1, 2]);
layer4 = Conv2DLayer(w4_l, b4_l, [1,1,1,1], [2,2], [1,1]);
w5_l = reshape(pre_bn2_weight, [1 1 2]);
b5_l = reshape(pre_bn2_bias, [1 1 2]);
m5_l = reshape(pre_bn2_running_mean, [1 1 2]);
v5_l = reshape(pre_bn2_running_var, [1 1 2]);
layer5 = BatchNormalizationLayer('Offset',b5_l,'Scale',w5_l,'TrainedMean',m5_l,'TrainedVariance',v5_l,'NumChannels',2,'Epsilon',pre_bn2_eps);
layer6 = TanhLayer;  % <<< TANH instead of ReLU
layer7 = FlattenLayer;
layer7.Type = 'nnet.cnn.layer.FlattenLayer';
net_pre = NN({layer1, layer2, layer3, layer4, layer5, layer6, layer7});

% ODE layer with permuted weights (case 20 = tanh dynamics)
global W1_ode b1_ode W2_ode b2_ode
W1_ode = ode_W1_p;
b1_ode = ode_b1';
W2_ode = ode_W2_p;
b2_ode = ode_b2_p';
ode_p = struct();
ode_p.system_choice = 20;  % <<< case 20 = tanh dynamics
ode_p.t_final = 1;
ode_p.timeStep = 20;
ode_p.W1 = W1_ode;
ode_p.b1 = b1_ode;
ode_p.W2 = W2_ode;
ode_p.b2 = b2_ode;
ode_layer = TNODEVODELayer(ode_p);

% Post-ODE with permuted fc1 (TANH activation)
fc1 = FullyConnectedLayer(post_fc1_W_p, post_fc1_b');
tanh_l = TanhLayer;  % <<< TANH instead of ReLU
fc2 = FullyConnectedLayer(post_fc2_W, post_fc2_b');

% Forward pass on clean image
x = double(img);
fprintf('Input to network: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = net_pre.evaluate(x);
fprintf('After pre-ODE (flatten): size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = ode_layer.evaluate(x);
fprintf('After ODE: size=%s, range=[%.4f, %.4f]\n', mat2str(size(x)), min(x(:)), max(x(:)));

x = fc1.evaluate(x);
x = tanh_l.evaluate(x);  % <<< tanh instead of relu
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

% Compare with PyTorch if available
if exist('pytorch_logits', 'var')
    fprintf('\n--- PyTorch vs MATLAB comparison ---\n');
    fprintf('PyTorch logits:  %s\n', mat2str(pytorch_logits(1,:), 4));
    fprintf('MATLAB logits:   %s\n', mat2str(x', 4));
end

fprintf('--- End Diagnostic ---\n\n');

% === Verification Parameters ===
pix = 192;          % number of pixels to attack (max: 192 = 8*8*3)
numT = 1;         % number of images to test
noise = 0.05/255;  % L_inf_norm radius (images already in [0,1])
rng(2022);        % Set random seed

% --- CHOOSE HEURISTIC HERE ---
% Options: 'naive', 'most-sensitive-input-radius', 'interval-norm-gradient'
splitting_heuristic = 'naive';
% -----------------------------

% Build test set
XTest = zeros(8, 8, 3, numT, 'double');
YTest = zeros(numT, 1, 'double');

for k = 1:numT
    YTest(k) = labels(k);
    XTest(:,:,:,k) = double(images(:,:,:,k));
end

fprintf('Data loaded. Running copepod TANH NODE_RA verification on %d images...\n', numT);

% Run verification
reach_copepod_NODE_RA_new_tanh(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);

end