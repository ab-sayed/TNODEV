% =========================================================================
% run_copepod_verification.m
% =========================================================================
% Quick-start script to run copepod Neural ODE verification with TNODEV.
%
% Before running:
%   1. Copy odenet_tiny_copepod_weights.mat to this directory
%   2. Copy copepod_test_images_tiny.mat to this directory
%   3. Add case 19 to your System_description.m
%   4. Ensure NNV and TNODEV are on the MATLAB path
%
% Usage:
%   >> run_copepod_verification
% =========================================================================

%% Configuration
pix   = 1;          % number of pixels to attack (start with 1)
numT  = 5;          % number of copepod images to verify
noise = 0.02;       % L-inf perturbation radius (in [0,1] pixel range)
perturbation = 'linf';
splitting = 'naive'; % 'naive' or 'eigenvalue'

%% Check files exist
assert(exist('odenet_tiny_copepod_weights.mat', 'file') == 2, ...
    'Missing: odenet_tiny_copepod_weights.mat — run the Python export script first.');
assert(exist('copepod_test_images_tiny.mat', 'file') == 2, ...
    'Missing: copepod_test_images_tiny.mat — run the Python export script first.');

%% Run verification
fprintf('Starting copepod Neural ODE verification...\n');
fprintf('  Pixels: %d, Images: %d, Noise: %.4f\n\n', pix, numT, noise);

reach_copepod_NODE_RA(pix, numT, noise, perturbation, splitting);
