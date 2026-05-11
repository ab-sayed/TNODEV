% run_mnist_smoketest.m
% =========================================================================
% Smoke test: full-image L_inf verification on MNIST tiny ODE-CNN
% Single-pass, NO refinement, NO falsification.
% Goal: report what TNODEV produces in seconds-to-minutes per image and
%       identify exactly where bounds explode (if they do).
%
% Pipeline matches §3.3 of Manzanas Lopez et al. (FORMATS 2022):
%   ImageStar -> NNV Pre-ODE (layers 1-7) -> TNODEV ODE (layer 8) ->
%   NNV final FC (layer 9) -> robustness check.
%
% Authors:  Abdelrahman Sayed Sayed, Pierre-Jean Meyer, Mohamed Ghazel
% Date:     May 2026
% =========================================================================

function run_mnist_smoketest()

    % --- Parameters ---
    pix   = 784;        % FULL image attack (every pixel perturbed)
    numT  = 1;          % single image for smoke test
    noise = 0.5/255;    % L_inf radius (paper default; scaled by 255 inside)
    rng(2022);

    % --- Load test data ---
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

    % --- Run smoke test ---
    reach_cnn_tiny_NODE_RA_smoketest(pix, numT, noise, XTest, YTest, 'inf');
end