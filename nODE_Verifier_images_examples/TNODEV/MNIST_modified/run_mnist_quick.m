function run_mnist_quick()

% Much smaller version of run_mnist_inf for testing

% Parameters
pix = 1; % number of pixels per image to attack / Highest working no. 135 pixels @ 63 iter. @ 19.5 mins  | Max. number is 784
numT = 1;  % number of images to test <-- change this to 1, 2, 5, ... as you like
noise = 0.5/255; % L_inf_norm radius (will be scaled by 255 inside the verification function)
rng(2022); % Set random seed

% --- CHOOSE HEURISTIC HERE ---   |--> Uncomment only for `reach_cnn_tiny_NODE_RA_new.m`
% Options: 'naive', 'jacobian', 'gradient'
splitting_heuristic = 'naive'; 
% -----------------------------

% Load all test images
Xall = processMNISTimages('t10k-images.idx3-ubyte');
Yall = processMNISTlabels('t10k-labels.idx1-ubyte');

Xall = single(extractdata(Xall));
Yall = single(Yall);

% % Build tiny test set (just first image)
% XTest = Xall(:,:,:,1);
% XTest = reshape(XTest, 28,28,1,1); % shape (H,W,C,N)
% YTest = single(Yall(1));

% Build test set with numT images: shape 28x28x1xnumT
XTest = zeros(28,28,1,numT, 'single');
YTest = zeros(numT,1, 'single');

for k = 1:numT
    YTest(k) = Yall(k);
    XTest(:,:,:,k) = Xall(:,:,:,k);
end

% % Reach computation (only tiny network, once) % Original NNV
% reach_cnn_tiny(pix, numT, noise, XTest, YTest, 'inf');

% Reach computation (tiny network, using my NODE_RA.m)
% reach_cnn_tiny_NODE_RA(pix, numT, noise, XTest, YTest, 'inf');  % Similar to NNV

% reach_cnn_tiny_NODE_RA_LayerByLayer(pix, numT, noise, XTest, YTest, 'inf');  % Layer by Layer

% fprintf('Data loaded. Running NODE_RA_NEW verification on %d images...\n', numT);


% % Reach computation (only tiny network, once) % Original NNV
% reach_cnn_tiny(pix, numT, noise, XTest, YTest, 'inf');

% Reach computation (tiny network, using our own NODE_RA_new.m)
reach_cnn_tiny_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);

end