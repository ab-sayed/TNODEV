% =========================================================================
% File: System_description.m
%
% Purpose:
%   User-provided description of the system's continuous-time dynamics, 
%   dispatched per system_choice.
%
%   Supported systems (system_choice):
%     14   FPA              (5D)
%     15   Spiral_non       (2D)
%     16   MNIST            (GNODE)
%     17   ACC nonlinear    (8D, tanh)
%     21   Spiral_linear    (2D)
%     22   Cartpole CTRNN   (12D)
%     24   ACC linear       (8D)
%
% Part of the TNODEV Toolbox
% % Paper: TNODEV: Toolbox for Neural ODE Verification
%
% License: CC BY-NC-SA 4.0
%
% Version History:
% Last update: 19 April 2026 
% Last revision: 04 June 2026
% =========================================================================

%------------- BEGIN CODE --------------
function dx = System_description(t, x, p) 

    %% Default value as NaN vector (not a number)
    n_x = length(x);
    dx  = NaN(n_x, 1);

    %% User-provided system description
    % For continuous-time systems: dx is the time derivative dx/dt
    global system_choice
    global u  % some systems require an external control input

    switch system_choice

        case 14
            %% --- FPA System (continuous-time, 5D) ---
            % System matrices and parameters
            A     = [-1.20327 -0.07202 -0.93635;  1.18810 -1.50015  0.93519];     % A (2x3)
            B     = [ 1.21464 -0.10502;  0.12023  0.19387; -1.36695  0.12201];    % B (3x2)
            theta = [-7.5649e-5; 1.34708e-4; -6.24925e-6];                         % bias (3x1) 
            tau   = 10^6;
            tau1  = -(1/tau);                                                      % time constant
            W     = [zeros(2),   A;
                     zeros(3,2), B*A];                                             % composite weight matrix combining A and B*A

            % Autonomous system dynamics
            dx = tau1 * x + W * tanh(x);


        case 15
            %% --- Spiral_non System (continuous-time, 2D) ---
            % Neural network dynamics:  dx = w2 * tanh(w1 * x + b1) + b2
            w1 = [ 0.2911133,   0.12008807;
                  -0.24582624,  0.23181419;
                  -0.25797904,  0.21687193;
                  -0.19282854, -0.2602416;
                   0.26780415, -0.20697702;
                   0.23462369,  0.2294843;
                  -0.2583547,   0.21444395;
                  -0.04514714,  0.29514763;
                  -0.15318371, -0.275755;
                   0.24873598,  0.21018365];
            b1 = [ 0.0038677; -0.00026365; -0.00716897;  0.02469357;  0.01338706; ...
                   0.00856025; -0.00888401;  0.00516089; -0.00634514; -0.01914518];
            w2 = [-0.58693904, -0.814841,   -0.8175157,   0.97060364,  0.6908913, ...
                  -0.92446184, -0.79249185, -1.1507587,   1.2072723,  -0.7983982;
                   1.1564877,  -0.8991244,  -1.0774536,  -0.6731967,   1.0154784, ...
                   0.8984464,  -1.0766245,  -0.238209,   -0.5233613,   0.8886671];
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
                error(['Error computing dynamics: %s. Check matrix dimensions: ' ...
                       'w1=%s, x=%s, b1=%s, w2=%s, b2=%s'], ...
                      e.message, mat2str(size(w1)), mat2str(size(x)), ...
                      mat2str(size(b1)), mat2str(size(w2)), mat2str(size(b2)));
            end

            % Add disturbance if provided
            if ~isempty(p)
                dx = dx + p;
            end


        case 16
            %% --- MNIST Neural ODE Linear Dynamics ---
            % MNIST example from NNV.  Linear ODE:  dx/dt = Aout_global * x + Bout_global.
            global Aout_global Bout_global

            if isempty(Aout_global) || isempty(Bout_global)
                error(['System_description: Globals Aout_global or Bout_global are empty. ' ...
                       'Ensure they are assigned in NODE_RA_New.m before simulation.']);
            end

            dx = Aout_global * x + Bout_global;


        case 17
            %% --- ACC Nonlinear Neural ODE Plant (8D) ---
            % 8-dimensional state vector:
            %   x1 : lead car position    x4 : ego car position
            %   x2 : lead car velocity    x5 : ego car velocity
            %   x3 : lead car internal    x6 : ego car internal
            %   x7 : ego acceleration     x8 : lead acceleration (-2)
            global w1_acc b1_acc w2_acc b2_acc

            % Avoids reading the .mat file 1000 times per second.
            % This drastically speeds up the continuous solver steps.
            if isempty(w1_acc)
                pl     = load("plant_3rd_order_tanh.mat");
                w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));

                % Forces biases to be column vectors to prevent
                % MATLAB's implicit matrix expansion.
                b1_acc = b1_acc(:);
                b2_acc = b2_acc(:);
            end

            % Ensure x is a column vector
            x = x(:);

            % Kinematic relations (dx/dt of position = velocity, etc.)
            x1_dot = x(2);
            x2_dot = x(3);
            x4_dot = x(5);
            x5_dot = x(6);

            % Neural network computes the complex internal-state derivatives.
            % Inputs to the NN are states {3, 6, 7, 8}.
            % NB: these weights come from `plant_3rd_order_tanh.mat` (the
            % physics NN), NOT the controller.
            nn_input = x([3, 6, 7, 8]);
            y        = tanh(w1_acc * nn_input + b1_acc);

            % dnn_out is a 4x1 vector
            dnn_out = w2_acc * y + b2_acc;

            % Construct the full 8D derivative vector. The NN's predictions
            % for dims 7 and 8 are taken as-is here; NNV's discrete holding
            % behaviour for acceleration is mimicked elsewhere in the
            % pipeline.
            dx = [x1_dot;       % dx1: lead car velocity
                  x2_dot;       % dx2: lead car acceleration
                  dnn_out(1);   % dx3: NN out 1   (lead car jerk)
                  x4_dot;       % dx4: ego car velocity
                  x5_dot;       % dx5: ego car acceleration
                  dnn_out(2);   % dx6: NN out 2   (ego car jerk)
                  dnn_out(3);   % dx7: NN out 3   (ego target acceleration derivative)
                  dnn_out(4)];  % dx8: NN out 4   (lead target acceleration derivative)


        case 24
            %% --- ACC Linear 3rd-order NODE Plant (8D) ---
            global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin

            if isempty(w1_acc_lin)
                pl         = load("plant_3rd_order_node.mat");   
                w1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
                b1_acc_lin = b1_acc_lin(:);
                b2_acc_lin = b2_acc_lin(:);
            end

            x        = x(:);
            nn_input = x([3, 6, 7, 8]);
            y        = w1_acc_lin * nn_input + b1_acc_lin;     % NO tanh activation
            dnn_out  = w2_acc_lin * y + b2_acc_lin;

            dx = [x(2); x(3); dnn_out(1);
                  x(5); x(6); dnn_out(2);
                  dnn_out(3); dnn_out(4)];


        case 21
            %% --- Spiral_linear System (continuous-time, 2D) ---
            % From Manzanas Lopez et al., FORMATS 2022 (spiral.m).
            % Architecture: 2D -> 10 hidden (IDENTITY, no activation) -> 2D
            % Effective dynamics:  dx = w2 * (w1*x + b1) + b2
            %                         = (w2*w1) * x + (w2*b1 + b2)
            % i.e. a pure linear 2D ODE:  dx = A_eff * x + b_eff.
            w1_sl = [-0.32294768,  0.59955627;
                      0.47014388, -0.39748120;
                     -0.56326932,  0.33752987;
                      0.45147443,  0.31528524;
                      0.41403031, -0.47271276;
                     -0.12952870, -0.62095606;
                     -0.41343114, -0.45678866;
                     -0.33266136,  0.29245856;
                      0.50114638,  0.39612201;
                      0.47665390,  0.55137879];
            b1_sl = [ 0.0038923009;   0.0037905588;   0.0017197595;  -0.0033185149; ...
                      0.0024190384;  -0.0013056855;   0.0011365928;  -0.00042518601; ...
                     -0.0025141449;   0.0010660964];
            w2_sl = [-0.50525320,  0.34800902, -0.34015974, -0.40054744,  0.39193857, ...
                      0.59363592,  0.56743664, -0.33811751, -0.36945280, -0.46805024;
                     -0.41715327,  0.56257814, -0.56921810,  0.60423535,  0.53992182, ...
                     -0.14412111, -0.45906776, -0.35295558,  0.49238238,  0.43526673];
            b2_sl = [-0.0013696412;  0.00060380378];

            if ~isvector(x) || length(x) ~= 2
                error('State vector x must be 2x1 for Spiral_linear, got size %s', mat2str(size(x)));
            end
            x  = x(:);
            dx = w2_sl * (w1_sl * x + b1_sl) + b2_sl;


        case 22
            %% --- Cartpole 12D with CT-RNN neural-network controller ---
            % State: [x_00, x_01, x_02, x_03, h_00 ... h_07]
            %   x_00 : cart position     x_01 : cart velocity
            %   x_02 : pole angle        x_03 : pole rate
            %   h_0i : 8 CT-RNN hidden states (each with its own tanh activation
            %          and recurrent weights)
            % Action: scalar output of a readout tanh applied to h_00..h_07.
            % Physics: standard cartpole equations.
            if ~isvector(x) || length(x) ~= 12
                error('State vector x must be 12x1 for Cartpole, got size %s', mat2str(size(x)));
            end
            x  = x(:);
            dx = CartpoleCTRNN(x, []);


        otherwise
            error('System_description: Unknown system_choice %d', system_choice);
    end
end
%------------- END OF CODE --------------
