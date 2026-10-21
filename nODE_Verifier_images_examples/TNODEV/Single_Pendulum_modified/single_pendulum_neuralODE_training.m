%% Train the Surrogate Neural ODE Model
clear; clc;

% 1. Load the data generated in Step 1
load('pendulum_surrogate_data.mat'); % Loads X_train and dX_train

% 2. Define the network architecture
% We will use 2 hidden layers with 64 neurons each.
% hiddenLayerSizes = [64, 64]; 
% surrogateNet = fitnet(hiddenLayerSizes, 'trainlm'); % Levenberg-Marquardt backpropagation

hiddenLayerSizes = [20]; % 1 hidden layer, 20 neurons
surrogateNet = fitnet(hiddenLayerSizes, 'trainbr');

% 3. Set activation functions
% Hyperbolic tangent sigmoid transfer function `tansig`: https://www.mathworks.com/help/deeplearning/ref/tansig.html
% Tanh (tansig in MATLAB) is highly recommended for continuous ODEs 
% because it creates a smooth, continuously differentiable vector field.
% surrogateNet.layers{1}.transferFcn = 'tansig';
% surrogateNet.layers{2}.transferFcn = 'tansig';
% surrogateNet.layers{3}.transferFcn = 'purelin'; % Output layer must be linear

% % 1 Layer only
surrogateNet.layers{1}.transferFcn = 'tansig';  % The single hidden layer
surrogateNet.layers{2}.transferFcn = 'purelin'; % The output layer

% 4. Configure training parameters
surrogateNet.trainParam.epochs = 1000;      % Maximum number of passes
surrogateNet.trainParam.goal = 1e-6;        % Target Mean Squared Error (MSE)
surrogateNet.trainParam.min_grad = 1e-7;    % Stop if the gradient gets too flat
surrogateNet.trainParam.showWindow = true;  % Open the training GUI

% 5. Train the network
disp('Training the continuous surrogate Neural ODE...');
[surrogateNet, tr] = train(surrogateNet, X_train, dX_train);

% 6. Evaluate the fit
dX_pred = surrogateNet(X_train);
mse_error = perform(surrogateNet, dX_train, dX_pred);
disp(['Training complete! Final Mean Squared Error: ', num2str(mse_error)]);

%% Step 4: Extract and Format Weights for TNDOVE

% disp('Extracting weights into Wb cell array format...');
% 
% % Initialize the cell array (3 layers = 6 matrices for weights and biases)
% Wb = cell(1, 6); 
% 
% % Layer 1 (Input to Hidden 1)
% Wb{1} = surrogateNet.IW{1,1}; % Input Weights
% Wb{2} = surrogateNet.b{1};    % Bias 1
% 
% % Layer 2 (Hidden 1 to Hidden 2)
% Wb{3} = surrogateNet.LW{2,1}; % Layer Weights
% Wb{4} = surrogateNet.b{2};    % Bias 2
% 
% % Layer 3 (Hidden 2 to Output)
% Wb{5} = surrogateNet.LW{3,2}; % Layer Weights
% Wb{6} = surrogateNet.b{3};    % Bias 3
% 
% % % Saving structure similar to the ACC
% 
% % 1. Get the training duration (D)
% D = tr.time(end); 
% 
% % 2. Get the final loss
% loss = mse_error; 
% 
% % 3. Create the parameter struct (matching the ACC formatting)
% neuralOdeParameters = struct();
% neuralOdeParameters.reachStep = 0.01;      % Standard solver step
% % neuralOdeParameters.controlPeriod = 5.0;   % Total time horizon
% neuralOdeParameters.controlPeriod = 1.0;   % Total time horizon similar to the specification
% neuralOdeParameters.method = 'approx-star'; % Standard reachability method
% 
% % 4. Save ALL variables to the .mat filee
% % save('odenet_pendulum_surrogate.mat', 'Wb');
% save('odenet_pendulum_surrogate.mat', 'Wb', 'D', 'loss', 'neuralOdeParameters');
% disp('Saved odenet_pendulum_surrogate.mat successfully!');

% UPDATED: for 1 Layer only 
disp('Extracting weights for 1-hidden-layer network into Wb cell array format...');

% Initialize the cell array (2 layers total = 4 matrices for weights and biases)
Wb = cell(1, 4); 

% Layer 1 (Input to Hidden)
Wb{1} = surrogateNet.IW{1,1}; % Input Weights (size: 20 x 3)
Wb{2} = surrogateNet.b{1};    % Bias 1 (size: 20 x 1)

% Layer 2 (Hidden to Output)
Wb{3} = surrogateNet.LW{2,1}; % Layer Weights (size: 2 x 20)
Wb{4} = surrogateNet.b{2};    % Bias 2 (size: 2 x 1)

% Extract training metadata
D = tr.time(end); 
loss = mse_error; 
neuralOdeParameters = struct();
neuralOdeParameters.reachStep = 0.01;      
neuralOdeParameters.controlPeriod = 1.5; % Updated for the shorter horizon!
neuralOdeParameters.method = 'approx-star'; 

% Save the new highly-regularized, verification-friendly file
save('odenet_pendulum_surrogate.mat', 'Wb', 'D', 'loss', 'neuralOdeParameters');
disp('Saved 1-layer odenet_pendulum_surrogate.mat successfully!');