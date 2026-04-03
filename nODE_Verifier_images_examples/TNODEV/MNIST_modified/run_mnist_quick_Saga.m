function run_mnist_quick()
    try
        % --- ADD THESE TWO PATHS FIRST ---
        % 1. Add your local data folder
        addpath('data');
        % Add the Examples folder so MATLAB can find System_description.m 
	addpath('/cluster/home/asibrahi/Documents/nODE_Verifier/nODE_Verifier_images_examples/TNODEV/Examples');        

        % 2. Initialize the NNV toolbox (adjust this path if your NNV is installed elsewhere)
        % Assuming NNV is somewhere in your home directory or projects folder:
        addpath('/cluster/home/asibrahi/Documents/nnv/code/nnv/'); 
        run('/cluster/home/asibrahi/Documents/nnv/code/nnv/startup_nnv.m'); 
        
        % NOTE: If you previously added NNV to your permanent MATLAB path 
        % via `pathtool` or a `startup.m` file, you might not need the lines above.
        % However, when running via mpirun/batch, it's always safest to explicitly
        % load required toolboxes at the very top of the script!
        % ---------------------------------

        % Much smaller version of run_mnist_inf for testing

        % Parameters
        pix = 1; % number of pixels per image to attack
        numT = 1;  % number of images to test
        % noise = 0.5/255; % L_inf_norm radius --> Original similar to NNV2.0
        noise = 0.1/255; % Start small to prove the pipeline works
        rng(2022); % Set random seed

        % --- CHOOSE HEURISTIC HERE ---   |--> Uncomment only for `reach_cnn_tiny_NODE_RA_new.m`
        % Options: 'naive', 'jacobian', 'gradient'
        splitting_heuristic = 'naive'; 

        % Load all test images (Explicitly looking inside the 'data' folder)
        Xall = processMNISTimages('data/t10k-images.idx3-ubyte');
        Yall = processMNISTlabels('data/t10k-labels.idx1-ubyte');

        Xall = single(extractdata(Xall));
        Yall = single(Yall);

        % Build test set with numT images: shape 28x28x1xnumT
        XTest = zeros(28,28,1,numT, 'single');
        YTest = zeros(numT,1, 'single');
        
        for k = 1:numT
            YTest(k) = Yall(k);
            XTest(:,:,:,k) = Xall(:,:,:,k);
        end

        % Reach computation (tiny network, using our own NODE_RA_new.m)
        reach_cnn_tiny_NODE_RA_new(pix, numT, noise, XTest, YTest, 'inf', splitting_heuristic);

        % --- HPC ZOMBIE PROTECTION ---
        % If everything succeeds, forcefully exit cleanly with code 0
        exit(0);
      
    catch ME
        % If ANY error happens (missing files, out of memory, toolbox errors), 
        % print the exact error to the Slurm log file and forcefully exit!
        disp('FATAL ERROR CAUGHT BY WRAPPER:');
        disp(getReport(ME, 'extended', 'hyperlinks', 'off'));
        exit(1);
    end
end
