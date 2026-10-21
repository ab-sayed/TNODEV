% run_mnist_smoketest_expm_medium.m
% =========================================================================
% Smoke test: full-image L_inf verification on MNIST MEDIUM ODE-CNN
% Linear ODE reach via matrix exponential (Path A).
%
% Mirrors run_mnist_smoketest_expm.m but for the medium-sized network:
%   - Conv/BN channels: 10 (vs 4 in tiny)
%   - ODE layer states: 1690 (vs 676 in tiny)
%   - Network file:    odecnn_mnist_mid2.mat
%
% Authors:  Abdelrahman Sayed Sayed, Pierre-Jean Meyer, Mohamed Ghazel
% Date:     May 2026
% =========================================================================

% function run_mnist_smoketest_expm_medium()
% 
% % --- Parameters ---
% pix   = 784;        % FULL image attack
% numT  = 1;          % set to 100 once smoke test passes
% noise = 0.5/255;    % paper default; sweep manually as needed
% rng(2022);
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
% % --- Run smoke test (expm-based, medium network) ---
% reach_cnn_medium_NODE_RA_smoketest_expm(pix, numT, noise, XTest, YTest, 'inf');
% end

%% V2

function run_mnist_smoketest_expm_medium()

pix         = 784;        % FULL image attack
numT        = 50;          % set to 100 for full sweep
noise       = 0.5/255;    % paper default
fast_bounds = false;      % toggle to true for fast-but-looser path
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

reach_cnn_medium_NODE_RA_smoketest_expm( ...
    pix, numT, noise, XTest, YTest, 'inf', fast_bounds);
end