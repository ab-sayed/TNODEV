% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 27 April 2026
% Last update: 05 June 2026
% Last revision: 

function run_copepod_quick_unified()
% run_copepod_quick_unified
%   Unified diagnostic + verification driver for the TNODEV pipeline.
%   Reads architecture metadata from the .mat file so a single script
%   handles:
%     * Any IMG_SIZE (8, 16, ...)
%     * Any N_FILTERS (2, 4, ...)
%     * Activation: tanh (System_description case 20) or relu (case 19)
%     * Either 7-class (bubble..other) or 2-class (copepod, not_copepod)
%
%   Replaces the old run_copepod_tanh_quick.m and run_copepod_relu_quick.m.
%
%   To switch between models, edit the WEIGHTS_FILE and IMAGES_FILE lines
%   below. No other changes needed.
%
%   Required fields in the weights .mat (recent exports include all of
%   these; older .mat files fall back to defaults):
%     pre_conv1_W, pre_conv1_b, pre_bn1_*, pre_conv2_W, pre_conv2_b, pre_bn2_*
%     ode_W1, ode_b1, ode_W2, ode_b2
%     post_fc1_W, post_fc1_b, post_fc2_W, post_fc2_b
%     activation             ('tanh' or 'relu')         [optional, default 'tanh']
%     n_filters              (2 or 4)                   [optional, inferred from pre_conv1_W]
%     num_classes            (2 or 7)                   [optional, inferred from post_fc2_W]
%     img_size               (8 or 16)                  [optional, inferred from images]
%     spatial_after_downsample (4 or 8)                 [optional, defaults to img_size/2]
%     class_labels_str       (comma-separated)          [optional, defaults below]

% ─────────────────────────────────────────────────────────────────────────
%   1. Pick which model to verify
% ─────────────────────────────────────────────────────────────────────────
% Pick exactly one weights/images pair:

% --- Clean baselines (N_FILTERS=2) ---

% === 04-04_Tiny_model_8x8_Tanh ===
% WEIGHTS_FILE = 'odenet_tanh_8x8_copepod_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8.mat';

% === 03-04_Tiny_model_8x8_ReLU ===
WEIGHTS_FILE = 'odenet_tiny_copepod_weights.mat';
IMAGES_FILE  = 'copepod_test_images_tiny.mat';

% --- 7-class adversarial (N_FILTERS=2) ---
% WEIGHTS_FILE = 'odenet_tanh_8x8_adv_copepod_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_adv.mat';

% WEIGHTS_FILE = 'odenet_tanh_8x8_trades_copepod_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_trades.mat';

% --- Binary N_FILTERS=2 ---
% WEIGHTS_FILE = 'odenet_tanh_8x8_binary_madry_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_binary_madry.mat';

% WEIGHTS_FILE = 'odenet_tiny_binary_madry_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tiny_binary_madry.mat';

% --- Binary N_FILTERS=4 ---

% === 27-04_tanh_8_binary_madry_n4_b8 ===
% WEIGHTS_FILE = 'odenet_tanh_8x8_binary_madry_n4_b8_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_binary_madry_n4_b8.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_binary_madry_n4_b8_v2.mat';

% === 27-04_relu_8_binary_madry_n4_b8 ===
% WEIGHTS_FILE = 'odenet_tiny_binary_madry_n4_b8_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tiny_binary_madry_n4_b8.mat';
% IMAGES_FILE  = 'copepod_test_images_tiny_binary_madry_n4_b8_v2.mat';

% === 27-04_tanh_8_binary_trades_n4_b8 ===
% WEIGHTS_FILE = 'odenet_tanh_8x8_binary_trades_n4_b8_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_binary_trades_n4_b8.mat';
% IMAGES_FILE  = 'copepod_test_images_tanh_8x8_binary_trades_n4_b8_v2.mat';

% === 27-04_relu_8_binary_trades_n4_b8 ===
% WEIGHTS_FILE = 'odenet_tiny_binary_trades_n4_b8_weights.mat';
% IMAGES_FILE  = 'copepod_test_images_tiny_binary_trades_n4_b8.mat';
% IMAGES_FILE  = 'copepod_test_images_tiny_binary_trades_n4_b8_v2.mat';


% % To see the confidence_quartile % %
% load('copepod_test_images_..._v2.mat');
% disp([1:length(confidence_quartile); confidence_quartile(:)'; logit_gap(:)']');

% ─────────────────────────────────────────────────────────────────────────
%   2. Load + parse architecture metadata
% ─────────────────────────────────────────────────────────────────────────
fprintf('=== Loading model: %s ===\n', WEIGHTS_FILE);
W = load(WEIGHTS_FILE);
D = load(IMAGES_FILE);

% Copy weights into local variables (so we can pass them around).
% Required fields:
pre_conv1_W = W.pre_conv1_W;
pre_conv1_b = W.pre_conv1_b;
pre_bn1_weight = W.pre_bn1_weight;
pre_bn1_bias = W.pre_bn1_bias;
pre_bn1_running_mean = W.pre_bn1_running_mean;
pre_bn1_running_var = W.pre_bn1_running_var;
pre_bn1_eps = W.pre_bn1_eps;
pre_conv2_W = W.pre_conv2_W;
pre_conv2_b = W.pre_conv2_b;
pre_bn2_weight = W.pre_bn2_weight;
pre_bn2_bias = W.pre_bn2_bias;
pre_bn2_running_mean = W.pre_bn2_running_mean;
pre_bn2_running_var = W.pre_bn2_running_var;
pre_bn2_eps = W.pre_bn2_eps;
ode_W1 = W.ode_W1;
ode_b1 = W.ode_b1;
ode_W2 = W.ode_W2;
ode_b2 = W.ode_b2;
post_fc1_W = W.post_fc1_W;
post_fc1_b = W.post_fc1_b;
post_fc2_W = W.post_fc2_W;
post_fc2_b = W.post_fc2_b;

images = D.images;
labels = D.labels;

% --- Architecture introspection ---
% Number of filters: prefer .mat field, else infer from pre_conv1_W
if isfield(W, 'n_filters')
    nf = double(W.n_filters);
else
    nf = size(pre_conv1_W, 4);   % conv weight shape (kH, kW, in_C, out_C)
    fprintf('  [info] n_filters not in .mat; inferred nf=%d from pre_conv1_W\n', nf);
end

% Number of classes: prefer .mat field, else infer from post_fc2_W
if isfield(W, 'num_classes')
    n_class = double(W.num_classes);
else
    n_class = size(post_fc2_W, 1);
    fprintf('  [info] num_classes not in .mat; inferred n_class=%d from post_fc2_W\n', n_class);
end

% Image size: prefer .mat field, else infer from images tensor (size 1)
if isfield(W, 'img_size')
    img_size = double(W.img_size);
else
    img_size = size(images, 1);
    fprintf('  [info] img_size not in .mat; inferred img_size=%d from images\n', img_size);
end

% Spatial dim after the stride-2 conv: prefer .mat, else img_size/2
if isfield(W, 'spatial_after_downsample')
    H_flat = double(W.spatial_after_downsample);
else
    H_flat = img_size / 2;
    fprintf('  [info] spatial_after_downsample not in .mat; using img_size/2 = %d\n', H_flat);
end
W_flat = H_flat;
C_flat = nf;
n_flat = H_flat * W_flat * C_flat;

% Activation: prefer .mat field, else default to tanh
if isfield(W, 'activation')
    activation = char(string(W.activation));     % handles char, string, or cell
else
    activation = 'tanh';
    fprintf('  [info] activation not in .mat; defaulting to ''tanh''\n');
end
activation = lower(strtrim(activation));
switch activation
    case 'tanh'
        system_choice = 20;
    case 'relu'
        system_choice = 19;
    otherwise
        error('run_copepod_quick_unified:unknown_activation', ...
              'Unknown activation ''%s'' (expected ''tanh'' or ''relu'')', activation);
end

% Class labels: prefer .mat field, else fall back by class count
if isfield(W, 'class_labels_str')
    cls_str = char(string(W.class_labels_str));
    class_labels = strsplit(cls_str, ',');
elseif n_class == 2
    class_labels = {'copepod', 'not_copepod'};
else
    class_labels = {'bubble','copepod','diatom_chain', ...
                    'faecal_pellets','oil','oily_gas','other'};
end

fprintf('--- Model summary ---\n');
fprintf('  activation:    %s   (system_choice = %d)\n', activation, system_choice);
fprintf('  n_filters:     %d\n', nf);
fprintf('  img_size:      %d\n', img_size);
fprintf('  spatial flat:  %dx%dx%d = %d\n', H_flat, W_flat, C_flat, n_flat);
fprintf('  num_classes:   %d\n', n_class);
fprintf('  class_labels:  %s\n', strjoin(class_labels, ', '));


% ─────────────────────────────────────────────────────────────────────────
%   3. Quick checks on image data
% ─────────────────────────────────────────────────────────────────────────
img = double(images(:,:,:,1));
fprintf('\n--- Test image info ---\n');
fprintf('  image size:    %s\n', mat2str(size(img)));
fprintf('  image range:   [%.4f, %.4f]\n', min(img(:)), max(img(:)));
fprintf('  true label:    %d (%s)\n', labels(1), class_labels{labels(1)});


% ─────────────────────────────────────────────────────────────────────────
%   4. Build PyTorch -> MATLAB flatten permutation
% ─────────────────────────────────────────────────────────────────────────
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

ode_W1_p = ode_W1(:, perm);
ode_W2_p = ode_W2(perm, :);
ode_b2_p = ode_b2(perm);
post_fc1_W_p = post_fc1_W(:, perm);


% ─────────────────────────────────────────────────────────────────────────
%   5. Forward-pass diagnostic in MATLAB
% ─────────────────────────────────────────────────────────────────────────
fprintf('\n--- Clean forward pass diagnostic ---\n');

% Pre-ODE
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

% ODE layer
global W1_ode b1_ode W2_ode b2_ode
W1_ode = ode_W1_p;
b1_ode = ode_b1';
W2_ode = ode_W2_p;
b2_ode = ode_b2_p';

ode_p = struct();
ode_p.system_choice = system_choice;
ode_p.t_final = 1;
ode_p.timeStep = 20;
ode_p.W1 = W1_ode;
ode_p.b1 = b1_ode;
ode_p.W2 = W2_ode;
ode_p.b2 = b2_ode;
ode_layer = TNODEVODELayer(ode_p);

% Post-ODE
fc1_layer  = FullyConnectedLayer(post_fc1_W_p, post_fc1_b');
post_act   = make_activation_layer(activation);
fc2_layer  = FullyConnectedLayer(post_fc2_W, post_fc2_b');

% Forward pass on the first image
x = double(img);
fprintf('  input:               size=%s, range=[%.4f, %.4f]\n', ...
        mat2str(size(x)), min(x(:)), max(x(:)));

x = net_pre.evaluate(x);
fprintf('  after pre-ODE:       size=%s, range=[%.4f, %.4f]\n', ...
        mat2str(size(x)), min(x(:)), max(x(:)));

x = ode_layer.evaluate(x);
fprintf('  after ODE:           size=%s, range=[%.4f, %.4f]\n', ...
        mat2str(size(x)), min(x(:)), max(x(:)));

x = fc1_layer.evaluate(x);
x = post_act.evaluate(x);
x = fc2_layer.evaluate(x);
fprintf('  final logits (%d cls): %s\n', n_class, mat2str(x', 4));

[~, pred] = max(x);
fprintf('  MATLAB prediction: %d (%s)\n', pred, class_labels{pred});
fprintf('  true label:        %d (%s)\n', labels(1), class_labels{labels(1)});

if pred == labels(1)
    fprintf('  MATCH - clean image classifies correctly in MATLAB.\n');
else
    fprintf('  MISMATCH - this is the root cause of any falsification!\n');
end

% Compare with PyTorch logits if exported
if isfield(D, 'pytorch_logits')
    fprintf('\n--- PyTorch vs MATLAB ---\n');
    fprintf('  PyTorch logits:  %s\n', mat2str(D.pytorch_logits(1,:), 4));
    fprintf('  MATLAB  logits:  %s\n', mat2str(x', 4));
end

fprintf('--- end diagnostic ---\n\n');


% ─────────────────────────────────────────────────────────────────────────
%   6. Verification parameters
% ─────────────────────────────────────────────────────────────────────────
pix    = img_size * img_size * 3;     % attack all input pixels by default
numT   = 1;                           % number of images to verify
noise  = 0.01/255;                     % L_inf radius | works for 0.01/255 - 0.1/255 - 0.5/255 | 1/255 goes into splitting
rng(2022);

splitting_heuristic = 'naive';        % 'naive' | 'most-sensitive-input-radius' | 'interval-norm-gradient'

% Build XTest, YTest from the loaded images
XTest = zeros(img_size, img_size, 3, numT, 'double');
YTest = zeros(numT, 1, 'double');
for k = 1:numT
    YTest(k) = labels(k);
    XTest(:,:,:,k) = double(images(:,:,:,k));
end

fprintf('Running NODE_RA verification on %d image(s) | activation=%s | n_filters=%d | n_class=%d\n', ...
        numT, activation, nf, n_class);

reach_copepod_NODE_RA_new_unified(WEIGHTS_FILE, pix, numT, noise, XTest, YTest, ...
                          'inf', splitting_heuristic);

end


% =========================================================================
% Local helper: pick the right activation layer
% =========================================================================
function act_layer = make_activation_layer(activation)
    switch lower(activation)
        case 'tanh'
            act_layer = TanhLayer;
        case 'relu'
            act_layer = ReluLayer;
        otherwise
            error('make_activation_layer:unknown', ...
                  'Unknown activation ''%s''', activation);
    end
end