% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 16 March 2026
% Last update: 19 April 2026 
% Last revision: XX XX 2026

%% V1.0

% %------------- BEGIN CODE --------------
% function reachable_tube_bounds = compute_CORA_Reachable_Tube(params, system_choice)
% % compute_CORA_Reachable_Tube.m 
% % Unified CORA reachability analysis for all continuous/hybrid Neural ODE systems.
% % Returns the overall spatial reachable tube bounds (lower and upper) across time.
% 
%     % === Validate required fields ===
%     required_fields = {'timeStep', 'tFinal', 'taylorTerms', 'zonotopeOrder', ...
%                        'alg', 'tensorOrder', 'x0', 'Initial_radius'};
%     for i = 1:length(required_fields)
%         if ~isfield(params, required_fields{i})
%             error('Missing required parameter: params.%s', required_fields{i});
%         end
%     end
% 
%     % === Define System-Specific Architecture ===
%     switch system_choice
%         case 15 % Spiral_non (2D)
%             dim = 2;
%             sys_handle = @spiral_non;
% 
%         case 14 % FPA (5D)
%             dim = 5;
%             sys_handle = @CTRNN_FPA;
% 
%         case 17 % ACC (8D)
%             dim = 8;
%             sys_handle = @acc_node_cora;
% 
%             % Load Global Weights (Required before CORA evaluates ACC)
%             global w1_acc b1_acc w2_acc b2_acc
%             if isempty(w1_acc)
%                 % fprintf('  Loading Neural ODE weights for CORA (ACC)...\n'); % Commented for clearner print flow in the command window
%                 pl = load("plant_3rd_order_tanh.mat");
%                 w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
%                 b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
%                 w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
%                 b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
%                 b1_acc = b1_acc(:); 
%                 b2_acc = b2_acc(:);
%             end
% 
%         otherwise
%             error('CORA Reachable Tube not implemented for system_choice = %d', system_choice);
%     end
% 
%     % === Initial set ===
%     lb = params.x0 - params.Initial_radius;
%     ub = params.x0 + params.Initial_radius;
%     R0_initial = interval(lb, ub);
% 
%     % === Reachability parameters ===
%     reach_params.R0       = zonotope(R0_initial);
%     reach_params.U        = zonotope([0, 0]); % No external input for autonomous plants
%     reach_params.timeStep = params.timeStep;
%     reach_params.tFinal   = params.tFinal;
% 
%     % === CORA options ===
%     options.timeStep      = params.timeStep;
%     options.taylorTerms   = params.taylorTerms;
%     options.zonotopeOrder = params.zonotopeOrder;
%     options.alg           = params.alg;
%     options.tensorOrder   = params.tensorOrder;
% 
%     % === Define system ===
%     sys = nonlinearSys(sys_handle, dim, 1); 
% 
%     % === Perform reachability ===
%     tic;
%     R = reach(sys, reach_params, options);
%     reachTime = toc;
%     fprintf('  CORA Reachability analysis completed in %.4f seconds\n', reachTime);
% 
%     % === Validate output structure ===
%     try
%         if ~isfield(R.timePoint, 'set') || isempty(R.timePoint.set)
%             error('R.timePoint.set is missing or empty.');
%         end
%     catch
%         error('Reachability analysis failed: R.timePoint.set not accessible.');
%     end
% 
%     % === Compute overall tube bounds ===
%     min_x_overall = inf(dim, 1);
%     max_x_overall = -inf(dim, 1);
% 
%     steps = length(R.timePoint.set);
% 
%     % Loop through time steps and extract interval hull of each zonotope
%     for i = 1:steps
%         current_interval = interval(R.timePoint.set{i}); 
%         min_x_overall = min(min_x_overall, current_interval.inf);
%         max_x_overall = max(max_x_overall, current_interval.sup);
%     end
% 
%     % === Store and return ===
%     reachable_tube_bounds.lb = min_x_overall;
%     reachable_tube_bounds.ub = max_x_overall;
% end
% 
% %------------- END OF CODE --------------

%% V1.1

%------------- BEGIN CODE --------------
function reachable_tube_bounds = compute_CORA_Reachable_Tube(params, system_choice)
% compute_CORA_Reachable_Tube.m 
% Unified CORA reachability analysis for all continuous/hybrid Neural ODE systems.
% Returns the overall spatial reachable tube bounds (lower and upper) across time.

    % === Validate required fields ===
    required_fields = {'timeStep', 'tFinal', 'taylorTerms', 'zonotopeOrder', ...
                       'alg', 'tensorOrder', 'x0', 'Initial_radius'};
    for i = 1:length(required_fields)
        if ~isfield(params, required_fields{i})
            error('Missing required parameter: params.%s', required_fields{i});
        end
    end

    % === Define System-Specific Architecture ===
    switch system_choice
        case 15 % Spiral_non (2D)
            dim = 2;
            sys_handle = @spiral_non;
            
        case 14 % FPA (5D)
            dim = 5;
            sys_handle = @CTRNN_FPA;
            
        case 17 % Nonlinear ACC (8D)
            dim = 8;
            sys_handle = @acc_node_cora;
            % Load Global Weights (Required before CORA evaluates ACC)
            global w1_acc b1_acc w2_acc b2_acc
            if isempty(w1_acc)
                % fprintf('  Loading Neural ODE weights for CORA (ACC)...\n'); % Commented for clearner print flow in the command window
                pl = load("plant_3rd_order_tanh.mat");
                w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
                b1_acc = b1_acc(:); 
                b2_acc = b2_acc(:);
            end

        case 24 % Linear ACC NODE (8D)
            dim = 8;
            sys_handle = @acc_linear_cora;

            % Lazy-load linear plant weights
            global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin
            if isempty(w1_acc_lin)
                pl = load("plant_3rd_order_linear.mat");
                w1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
                b1_acc_lin = b1_acc_lin(:);
                b2_acc_lin = b2_acc_lin(:);
            end
            
        case 21 % Spiral_linear (2D)
            dim = 2;
            sys_handle = @spiral;              % your existing spiral.m file

        case 22 % Cartpole (12D)
            dim = 12;
            sys_handle = @CartpoleCTRNN;       % your existing CartpoleCTRNN.m file

        case 21 % Spiral_linear (2D)
            dim = 2;
            sys_handle = @spiral_linear_cora;

        case 22 % Cartpole (12D)
            dim = 12;
            sys_handle = @cartpole_ctrnn_cora;
            
        otherwise
            error('CORA Reachable Tube not implemented for system_choice = %d', system_choice);
    end

    % === Initial set ===
    lb = params.x0 - params.Initial_radius;
    ub = params.x0 + params.Initial_radius;
    R0_initial = interval(lb, ub);
    
    % === Reachability parameters ===
    reach_params.R0       = zonotope(R0_initial);
    reach_params.U        = zonotope([0, 0]); % No external input for autonomous plants
    reach_params.timeStep = params.timeStep;
    reach_params.tFinal   = params.tFinal;

    % === CORA options ===
    options.timeStep      = params.timeStep;
    options.taylorTerms   = params.taylorTerms;
    options.zonotopeOrder = params.zonotopeOrder;
    options.alg           = params.alg;
    options.tensorOrder   = params.tensorOrder;

    % === Define system ===
    sys = nonlinearSys(sys_handle, dim, 1); 

    % === Perform reachability ===
    tic;
    R = reach(sys, reach_params, options);
    reachTime = toc;
    fprintf('  CORA Reachability analysis completed in %.4f seconds\n', reachTime);

    % === Validate output structure ===
    try
        if ~isfield(R.timePoint, 'set') || isempty(R.timePoint.set)
            error('R.timePoint.set is missing or empty.');
        end
    catch
        error('Reachability analysis failed: R.timePoint.set not accessible.');
    end

    % === Compute overall tube bounds ===
    min_x_overall = inf(dim, 1);
    max_x_overall = -inf(dim, 1);

    steps = length(R.timePoint.set);
    
    % Loop through time steps and extract interval hull of each zonotope
    for i = 1:steps
        current_interval = interval(R.timePoint.set{i}); 
        min_x_overall = min(min_x_overall, current_interval.inf);
        max_x_overall = max(max_x_overall, current_interval.sup);
    end

    % === Store and return ===
    reachable_tube_bounds.lb = min_x_overall;
    reachable_tube_bounds.ub = max_x_overall;
end

%------------- END OF CODE --------------