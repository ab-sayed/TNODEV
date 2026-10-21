function run_copepod_quick()
    try
        % --- ADD PATHS ---
        addpath('data');
        addpath('/cluster/home/asibrahi/Documents/nODE_Verifier/nODE_Verifier_images_examples/TNODEV/Examples');
        addpath('/cluster/home/asibrahi/Documents/nnv/code/nnv/');
        run('/cluster/home/asibrahi/Documents/nnv/code/nnv/startup_nnv.m');

        % Parameters
        pix = 1;             % pixels to attack (max: 192 = 8*8*3)
        numT = 1;            % 1 image for smoke test
        noise = 0.5/255;     % L_inf radius
        rng(2022);
        splitting_heuristic = 'most-sensitive-input-radius';

        % Load copepod test images
        load('copepod_test_images_tiny.mat');  % images [8,8,3,20], labels [1,20]

        % Build test set
        XTest = zeros(8, 8, 3, numT, 'double');
        YTest = zeros(numT, 1, 'double');
        for k = 1:numT
            YTest(k) = labels(k);
            XTest(:,:,:,k) = double(images(:,:,:,k));
        end

        fprintf('Copepod verification: %d images, %d pixels, noise=%.6f, heuristic=%s\n', ...
                numT, pix, noise, splitting_heuristic);

        % Run verification
        reach_copepod_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);

        exit(0);
    catch ME
        disp('FATAL ERROR CAUGHT BY WRAPPER:');
        disp(getReport(ME, 'extended', 'hyperlinks', 'off'));
        exit(1);
    end
end
