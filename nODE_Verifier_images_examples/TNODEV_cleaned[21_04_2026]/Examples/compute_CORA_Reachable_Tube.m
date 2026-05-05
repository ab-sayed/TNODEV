% =========================================================================
% File: compute_CORA_Reachable_Tube.m
%
% Purpose:
%   Unified CORA reachability analysis for all continuous / hybrid
%   Neural ODE systems supported by the toolbox. Returns the overall
%   spatial reachable-tube bounds (lower and upper) across time.
%
% Description:
%   Builds a CORA nonlinearSys for the requested system_choice, runs
%   reach() over the requested time horizon, and computes the
%   per-dimension interval hull of all reachable sets across time steps.
%
%   Supported systems (system_choice):
%     14   FPA           (5D)
%     15   Spiral_non    (2D)
%     17   ACC nonlinear (8D, tanh activation)
%     21   Spiral_linear (2D)
%     22   Cartpole CTRNN (12D)
%     24   ACC linear    (8D, no activation)
%
%   For systems that depend on pretrained Neural ODE weights (17, 24),
%   the weights are lazy-loaded into globals on the first call.
%
% Inputs:
%   params         : struct with fields
%                       .timeStep, .tFinal,
%                       .taylorTerms, .zonotopeOrder,
%                       .alg, .tensorOrder,
%                       .x0, .Initial_radius
%   system_choice  : integer system identifier (see list above)
%
% Output:
%   reachable_tube_bounds : struct with fields
%                              .lb : per-dimension lower bound across time
%                              .ub : per-dimension upper bound across time
%
% Prerequisites:
%   - CORA toolbox on the MATLAB path
%
% Part of the TNODEV Toolbox
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
% =========================================================================

%------------- BEGIN CODE --------------
function reachable_tube_bounds = compute_CORA_Reachable_Tube(params, system_choice)

    %% 1. Validate required fields
    required_fields = {'timeStep', 'tFinal', 'taylorTerms', 'zonotopeOrder', ...
                       'alg', 'tensorOrder', 'x0', 'Initial_radius'};
    for i = 1:length(required_fields)
        if ~isfield(params, required_fields{i})
            error('Missing required parameter: params.%s', required_fields{i});
        end
    end


    %% 2. Define system-specific architecture
    switch system_choice
        case 15   % Spiral_non (2D)
            dim        = 2;
            sys_handle = @spiral_non;

        case 14   % FPA (5D)
            dim        = 5;
            sys_handle = @CTRNN_FPA;

        case 17   % Nonlinear ACC (8D, tanh)
            dim        = 8;
            sys_handle = @acc_node_cora;

            % Lazy-load nonlinear plant weights into globals
            global w1_acc b1_acc w2_acc b2_acc
            if isempty(w1_acc)
                pl     = load("plant_3rd_order_tanh.mat");
                w1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
                b1_acc = b1_acc(:);
                b2_acc = b2_acc(:);
            end

        case 24   % Linear ACC NODE (8D, no activation)
            dim        = 8;
            sys_handle = @acc_linear_cora;

            % Lazy-load linear plant weights into globals
            global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin
            if isempty(w1_acc_lin)
                pl         = load("plant_3rd_order_linear.mat");
                w1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Weights));
                b1_acc_lin = double(extractdata(pl.neuralOdeParameters.fc1.Bias));
                w2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Weights));
                b2_acc_lin = double(extractdata(pl.neuralOdeParameters.fc2.Bias));
                b1_acc_lin = b1_acc_lin(:);
                b2_acc_lin = b2_acc_lin(:);
            end

        case 21   % Spiral_linear (2D)
            dim        = 2;
            sys_handle = @spiral;

        case 22   % Cartpole CTRNN (12D)
            dim        = 12;
            sys_handle = @CartpoleCTRNN;

        otherwise
            error('CORA Reachable Tube not implemented for system_choice = %d', system_choice);
    end


    %% 3. Initial set
    lb         = params.x0 - params.Initial_radius;
    ub         = params.x0 + params.Initial_radius;
    R0_initial = interval(lb, ub);


    %% 4. Reachability parameters
    reach_params.R0       = zonotope(R0_initial);
    reach_params.U        = zonotope([0, 0]);   % no external input for autonomous plants
    reach_params.timeStep = params.timeStep;
    reach_params.tFinal   = params.tFinal;


    %% 5. CORA options
    options.timeStep      = params.timeStep;
    options.taylorTerms   = params.taylorTerms;
    options.zonotopeOrder = params.zonotopeOrder;
    options.alg           = params.alg;
    options.tensorOrder   = params.tensorOrder;


    %% 6. Define system and run reachability
    sys = nonlinearSys(sys_handle, dim, 1);

    tic;
    R         = reach(sys, reach_params, options);
    reachTime = toc;
    fprintf('  CORA Reachability analysis completed in %.4f seconds\n', reachTime);


    %% 7. Validate output structure
    try
        if ~isfield(R.timePoint, 'set') || isempty(R.timePoint.set)
            error('R.timePoint.set is missing or empty.');
        end
    catch
        error('Reachability analysis failed: R.timePoint.set not accessible.');
    end


    %% 8. Compute overall tube bounds (per-dimension interval hull across time)
    min_x_overall = inf(dim, 1);
    max_x_overall = -inf(dim, 1);

    steps = length(R.timePoint.set);
    for i = 1:steps
        current_interval = interval(R.timePoint.set{i});
        min_x_overall    = min(min_x_overall, current_interval.inf);
        max_x_overall    = max(max_x_overall, current_interval.sup);
    end


    %% 9. Store and return
    reachable_tube_bounds.lb = min_x_overall;
    reachable_tube_bounds.ub = max_x_overall;

end
%------------- END OF CODE --------------
