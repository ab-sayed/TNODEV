% run_mnist_smoketest_expm_medium.m
% =========================================================================
%
% Part of the TNODEV Toolbox
% Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

function run_mnist_smoketest_expm_medium()

pix         = 784;        % FULL image attack
numT        = 50;          % set to 100 for full sweep
noise       = 0.5/255;    % paper default
fast_bounds = true;      % toggle to true for fast-but-looser bounds
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

reach_cnn_medium_NODE_RA_expm( ...
    pix, numT, noise, XTest, YTest, 'inf', fast_bounds);
end