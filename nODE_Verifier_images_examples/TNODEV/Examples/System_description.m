% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 20 October 2025
% Last update: 20 February 2026 
% Last revision:  2026

%% Descripion of the time-varying system with input
% either continuous-time dynamics: dx/dt = f(t,x,p)
% or discrete-time successor: x^+ = f(t,x,p)

% List of inputs
%   t: time
%   x: state
%   p: input value

% List of outputs
%   dx: 
%    continuous-time: vector field evaluated at time t, state x and input p
%    discrete-time: one-step successor from state x at time t with input p  

% Authors:  
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Alex Devonport, <alex_devonport -AT- berkeley.edu>, EECS, UC Berkeley
% Date: 2nd of July 2021

% function dx = System_description(t,x,p)
% 
% %% Default value as NaN vector (not a number)
% n_x = length(x);
% dx = NaN(n_x,1);
% 
% %% User-provided system description
% % For continuous-time system: dx is the time derivative dx/dt
% % For discrete-time system: dx is the one-step successor x^+
% global system_choice
% global u        % Some systems need an external control input
% 
% switch system_choice
%     case 14
%         %% FPA System (continuous-time)
%         % System matrices and parameters
%         A = [-1.20327 -0.07202 -0.93635; 1.18810 -1.50015 0.93519]; % A (2x3)
%         B = [1.21464 -0.10502; 0.12023 0.19387; -1.36695 0.12201]; % B (3x2)
%         theta = [-7.5649e-5; 1.34708e-4; -6.24925e-6]; % Bias vector (3x1)
%         tau = 10^6;
%         tau1 = -(1/tau); % time constant for neurons
%         W = [[zeros(2), A]; [zeros(3,2), B*A]]; % composite weight matrix combining `A` and `B*A`
%         % Define the autonomous system dynamics as a function
%         dx = tau1*x + W*tanh(x);  % Autonomous system
%     case 15
%         %% Spiral_non System (continuous-time)
%         % Neural network dynamics: dx = w2*tanh(w1*x + b1) + b2
%         w1 = [0.2911133,  0.12008807;
%               -0.24582624, 0.23181419;
%               -0.25797904, 0.21687193;
%               -0.19282854, -0.2602416;
%               0.26780415, -0.20697702;
%               0.23462369, 0.2294843;
%               -0.2583547,  0.21444395;
%               -0.04514714, 0.29514763;
%               -0.15318371, -0.275755;
%               0.24873598, 0.21018365];
%         b1 = [0.0038677; -0.00026365; -0.00716897; 0.02469357; 0.01338706;...
%               0.00856025; -0.00888401; 0.00516089; -0.00634514; -0.01914518];
%         w2 = [-0.58693904, -0.814841, -0.8175157, 0.97060364, 0.6908913,...
%               -0.92446184, -0.79249185, -1.1507587, 1.2072723, -0.7983982;
%               1.1564877, -0.8991244, -1.0774536, -0.6731967, 1.0154784,...
%               0.8984464, -1.0766245, -0.238209, -0.5233613, 0.8886671];
%         b2 = [-0.04129209; -0.01508532];
% 
%         % Verify input dimensions
%         if ~isvector(x) || length(x) ~= 2
%             error('State vector x must be 2x1 for Spiral_non system, got size %s', mat2str(size(x)));
%         end
%         if ~isempty(p) && (~isvector(p) || length(p) ~= 2)
%             error('Input vector p must be 2x1 or empty for Spiral_non system, got size %s', mat2str(size(p)));
%         end
% 
%         % Ensure x is a column vector
%         x = x(:);
% 
%         % Compute dynamics
%         try
%             dx = w2 * tanh(w1 * x + b1) + b2;
%         catch e
%             error('Error computing dynamics: %s. Check matrix dimensions: w1=%s, x=%s, b1=%s, w2=%s, b2=%s', ...
%                   e.message, mat2str(size(w1)), mat2str(size(x)), mat2str(size(b1)), mat2str(size(w2)), mat2str(size(b2)));
%         end
% 
% 
%         % Add disturbance if provided
%         if ~isempty(p)
%             dx = dx + p;
%         end
%     case 16 % === MNIST neural ODE dynamics: ẋ = Aout x + Bout ===
%         % MNIST example from NNV
%         % Linear ODE dynamics: dx/dt = A_mnist * x + B_mnist
%         % The matrices Aout_global and Bout_global must be populated in NODE_RA.m
%         % dx = A_mnist * x + B_mnist;
%         global Aout_global Bout_global
% 
%         % Check if globals are actually loaded to prevent silent errors
%         if isempty(Aout_global) || isempty(Bout_global)
%             error('System_description: Globals Aout_global or Bout_global are empty. Ensure they are assigned in NODE_RA.m before simulation.');
%         end
% 
%         dx = Aout_global * x + Bout_global;
% 
%     otherwise
%         error('System_description: Unknown system_choice %d', system_choice);
% end

%% ADDED ACC example

function dx = System_description(t,x,p)

%% Default value as NaN vector (not a number)
n_x = length(x);
dx = NaN(n_x,1);

%% User-provided system description
% For continuous-time system: dx is the time derivative dx/dt
% For discrete-time system: dx is the one-step successor x^+
global system_choice
global u        % Some systems need an external control input

switch system_choice
    case 14
        %% FPA System (continuous-time)
        % System matrices and parameters
        A = [-1.20327 -0.07202 -0.93635; 1.18810 -1.50015 0.93519]; % A (2x3)
        B = [1.21464 -0.10502; 0.12023 0.19387; -1.36695 0.12201]; % B (3x2)
        theta = [-7.5649e-5; 1.34708e-4; -6.24925e-6]; % Bias vector (3x1)
        tau = 10^6;
        tau1 = -(1/tau); % time constant for neurons
        W = [[zeros(2), A]; [zeros(3,2), B*A]]; % composite weight matrix combining `A` and `B*A`
        % Define the autonomous system dynamics as a function
        dx = tau1*x + W*tanh(x);  % Autonomous system
    case 15
        %% Spiral_non System (continuous-time)
        % Neural network dynamics: dx = w2*tanh(w1*x + b1) + b2
        w1 = [0.2911133,  0.12008807;
              -0.24582624, 0.23181419;
              -0.25797904, 0.21687193;
              -0.19282854, -0.2602416;
              0.26780415, -0.20697702;
              0.23462369, 0.2294843;
              -0.2583547,  0.21444395;
              -0.04514714, 0.29514763;
              -0.15318371, -0.275755;
              0.24873598, 0.21018365];
        b1 = [0.0038677; -0.00026365; -0.00716897; 0.02469357; 0.01338706;...
              0.00856025; -0.00888401; 0.00516089; -0.00634514; -0.01914518];
        w2 = [-0.58693904, -0.814841, -0.8175157, 0.97060364, 0.6908913,...
              -0.92446184, -0.79249185, -1.1507587, 1.2072723, -0.7983982;
              1.1564877, -0.8991244, -1.0774536, -0.6731967, 1.0154784,...
              0.8984464, -1.0766245, -0.238209, -0.5233613, 0.8886671];
        b2 = [-0.04129209; -0.01508532];

        % Verify input dimensions
        if ~isvector(x) || length(x) ~= 2
            error('State vector x must be 2x1 for Spiral_non system, got size %s', mat2str(size(x)));
        end
        if ~isempty(p) && (~isvector(p) || length(p) ~= 2)
            error('Input vector p must be 2x1 or empty for Spiral_non system, got size %s', mat2str(size(p)));
        end

        % Ensure x is a column vector
        x = x(:);

        % Compute dynamics
        try
            dx = w2 * tanh(w1 * x + b1) + b2;
        catch e
            error('Error computing dynamics: %s. Check matrix dimensions: w1=%s, x=%s, b1=%s, w2=%s, b2=%s', ...
                  e.message, mat2str(size(w1)), mat2str(size(x)), mat2str(size(b1)), mat2str(size(w2)), mat2str(size(b2)));
        end


        % Add disturbance if provided
        if ~isempty(p)
            dx = dx + p;
        end
    case 16 
        % === MNIST neural ODE dynamics: ẋ = Aout x + Bout ===
        % MNIST example from NNV
        % Linear ODE dynamics: dx/dt = A_mnist * x + B_mnist
        % The matrices Aout_global and Bout_global must be populated in NODE_RA.m
        % dx = A_mnist * x + B_mnist;
        global Aout_global Bout_global

        % Check if globals are actually loaded to prevent silent errors
        if isempty(Aout_global) || isempty(Bout_global)
            error('System_description: Globals Aout_global or Bout_global are empty. Ensure they are assigned in NODE_RA.m before simulation.');
        end
        
        dx = Aout_global * x + Bout_global;
        
    case 17 
        %% === ACC Non-Linear Neural ODE Plant ===
        % 8-dimensional state vector:
        % x1: lead_car pos, x2: lead_car vel, x3: lead_car internal
        % x4: ego_car pos,  x5: ego_car vel,  x6: ego_car internal
        % x7: ego accel,    x8: lead accel (-2)
        global w1_acc b1_acc w2_acc b2_acc
        
        % Lazy loading to avoid reading the .mat file 1000 times per second
        % This drastically speeds up the continuous solver steps
        if isempty(w1_acc)
            pl = load("plant_3rd_order_tanh.mat");
            w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
            b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
            w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
            b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));

            % FIX: Force biases to be column vectors to prevent MATLAB 
            % from doing implicit matrix expansion!
            b1_acc = b1_acc(:);
            b2_acc = b2_acc(:);

        end
        
        % Ensure x is a column vector
        x = x(:);
        
        % Kinematic relations (dx/dt of position = velocity, etc.)
        x1_dot = x(2); % 
        x2_dot = x(3);
        x4_dot = x(5);
        x5_dot = x(6);
        
        % Neural Network calculates complex internal state derivatives
        % Inputs to NN are states: 3, 6, 7, 8
        nn_input = x([3, 6, 7, 8]);
        y = tanh(w1_acc * nn_input + b1_acc); % These weights come from `plant_3rd_order_tanh.mat` (Physics NN), NOT the controller!
        
        % dnn_out is a 4x1 vector
        dnn_out = w2_acc * y + b2_acc;
        
        % Construct the full 8-dimensional derivative vector.
        % We override the NN's predictions for dims 7 and 8 to 0, 
        % mimicking NNV's discrete holding behavior for acceleration.
        dx = [x1_dot;     % dx1
              x2_dot;     % dx2
              dnn_out(1); % dx3 <--- This is the derivative of x3 (Lead Car Jerk)
              x4_dot;     % dx4
              x5_dot;     % dx5
              dnn_out(2); % dx6 <--- This is the derivative of x6 (Ego Car Jerk)
              0;          % dx7 (Ego Accel held constant)
              0];         % dx8 (Lead Accel held constant)
    otherwise
        error('System_description: Unknown system_choice %d', system_choice);
end
