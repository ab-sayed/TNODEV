% run_mnist_smoketest_expm.m
% =========================================================================
% Smoke test: full-image L_inf verification on MNIST tiny ODE-CNN
% Linear ODE reach via matrix exponential (Path A).
%
% Differs from run_mnist_smoketest.m:
%   - ODE step uses closed-form expm-based interval reach instead of CTMM
%   - Everything else identical
%
% Authors:  Abdelrahman Sayed Sayed, Pierre-Jean Meyer, Mohamed Ghazel
% Date:     May 2026
% =========================================================================

% function run_mnist_smoketest_expm()
% 
% % --- Parameters ---
% pix   = 784;       % number of pixels per image to attack % FULL image attack is 784
% numT  = 1;         % number of images to test  
% noise = 0.5/255;   % L_inf_norm radius
% rng(2022);         % Set random seed
% 
% % --- Load test data ---
% Xall = processMNISTimages('t10k-images.idx3-ubyte');
% Yall = processMNISTlabels('t10k-labels.idx1-ubyte');
% Xall = single(extractdata(Xall));
% Yall = single(Yall);
% 
% XTest = zeros(28, 28, 1, numT, 'single');
% YTest = zeros(numT, 1, 'single');
% for k = 1:numT
%     XTest(:,:,:,k) = Xall(:,:,:,k);
%     YTest(k)       = Yall(k);
% end
% 
% % --- Run smoke test (expm-based) ---
% reach_cnn_tiny_NODE_RA_smoketest_expm(pix, numT, noise, XTest, YTest, 'inf');
% end

%% V2

function run_mnist_smoketest_expm()

pix         = 784;        % FULL image attack
numT        = 50;          % set to 100 for full sweep
noise       = 0.5/255;    % paper default
fast_bounds = true;      % toggle to true for fast-but-looser path
rng(2022); 

Xall = processMNISTimages('t10k-images.idx3-ubyte');
Yall = processMNISTlabels('t10k-labels.idx1-ubyte');
Xall = single(extractdata(Xall));
Yall = single(Yall);

XTest = zeros(28, 28, 1, numT, 'single');
YTest = zeros(numT, 1, 'single');
for k = 1:numT
    XTest(:,:,:,k) = Xall(:,:,:,k);
    YTest(k)       = Yall(k);
end

reach_cnn_tiny_NODE_RA_smoketest_expm( ...
    pix, numT, noise, XTest, YTest, 'inf', fast_bounds);
end