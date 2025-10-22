%% Sample Points: Extract vertex points from each Star set and compute a 2D convex hull for the projection (using MATLAB’s convhull) --> Working code as of 03/08 
% % Used for computing lower and upper bounds

%% for the 1st approach (initial step)

function fpa_reach() 

    reachstep = 0.05; % step size to compute reach sets
    final_time = 2; % Reduced time horizon
    Initial_radius = 0.01; % Uncertainty in dynamics
    model = NonLinearODE(5,1,@CTRNN_FPA, reachstep, final_time, eye(5));

    % Change default options
    model.options.timeStep = 0.05;
    model.options.taylorTerms = 3;
    model.options.zonotopeOrder = 10;
    model.options.alg = 'lin';
    model.options.tensorOrder = 2;
    model = ODEblockLayer(model, final_time, reachstep, true);
    nn = NN({model});

    % Initial states
    x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
    lb = x0 - Initial_radius;
    ub = x0 + Initial_radius;
    init_set = Star(lb, ub);

    % Compute reachability analysis
    t = tic;
    R_tube = nn.reach(init_set);
    time = toc(t);

    % for all 5 dimensions
    min_x = inf(5, 1);
    max_x = -inf(5, 1);
    points = []; % Store 5D points for all dimensions
    for i = 1:length(R_tube)
        box = R_tube(i).getBox;
        lb = box.lb;
        ub = box.ub;
        fprintf('Star set %d: lb = [%s], ub = [%s]\n', i, num2str(lb'), num2str(ub'));
        min_x = min(min_x, lb);
        max_x = max(max_x, ub);

        % Add 8 corner points for the 5D box (subset to manage memory) --> The original
        % was 8 not 32 like I modified here
        corners = [

          lb(1) lb(2) lb(3) lb(4) lb(5); 
          lb(1) lb(2) lb(3) lb(4) ub(5);
          lb(1) lb(2) lb(3) ub(4) lb(5);
          lb(1) lb(2) ub(3) lb(4) lb(5);
          ub(1) lb(2) lb(3) lb(4) lb(5);
          ub(1) ub(2) lb(3) lb(4) lb(5);
          ub(1) lb(2) ub(3) lb(4) lb(5);
          ub(1) lb(2) lb(3) ub(4) lb(5)

          % lb(1) lb(2) lb(3) lb(4) lb(5); 
          % lb(1) lb(2) lb(3) lb(4) ub(5);
          % lb(1) lb(2) lb(3) ub(4) lb(5);
          % lb(1) lb(2) ub(3) lb(4) lb(5);
          % lb(1) lb(2) lb(3) lb(4) lb(5); 
          % lb(1) lb(2) lb(3) lb(4) ub(5);
          % lb(1) lb(2) lb(3) ub(4) lb(5);
          % lb(1) lb(2) ub(3) lb(4) lb(5);
          % lb(1) lb(2) lb(3) lb(4) lb(5); 
          % lb(1) lb(2) lb(3) lb(4) ub(5);
          % lb(1) lb(2) lb(3) ub(4) lb(5);
          % lb(1) lb(2) ub(3) lb(4) lb(5);
          % lb(1) lb(2) lb(3) lb(4) lb(5); 
          % lb(1) lb(2) lb(3) lb(4) ub(5);
          % lb(1) lb(2) lb(3) ub(4) lb(5);
          % lb(1) lb(2) ub(3) lb(4) lb(5);
          % ub(1) lb(2) lb(3) lb(4) lb(5);
          % ub(1) ub(2) lb(3) lb(4) lb(5);
          % ub(1) lb(2) ub(3) lb(4) lb(5);
          % ub(1) lb(2) lb(3) ub(4) lb(5);
          % ub(1) lb(2) lb(3) lb(4) lb(5);
          % ub(1) ub(2) lb(3) lb(4) lb(5);
          % ub(1) lb(2) ub(3) lb(4) lb(5);
          % ub(1) lb(2) lb(3) ub(4) lb(5);
          % ub(1) lb(2) lb(3) lb(4) lb(5);
          % ub(1) ub(2) lb(3) lb(4) lb(5);
          % ub(1) lb(2) ub(3) lb(4) lb(5);
          % ub(1) lb(2) lb(3) ub(4) lb(5);
          % ub(1) lb(2) lb(3) lb(4) lb(5);
          % ub(1) ub(2) lb(3) lb(4) lb(5);
          % ub(1) lb(2) ub(3) lb(4) lb(5);
          % ub(1) lb(2) lb(3) ub(4) lb(5)
         ];
         points = [points; corners];
      end

      % Ensure points has 5 columns
      if size(points, 2) ~= 5
        error('Points matrix has %d columns, expected 5', size(points, 2));
      end
      fprintf('Points matrix size: %s\n', num2str(size(points)));

      % Compute reachable tube and invariant set
      reachable_tube = [min_x, max_x]; % 5x2 matrix
      S = reachable_tube;

      %Save results
      save('fpa_reach.mat', 'R_tube', 'time', 'reachable_tube', 'S', 'points', '-v7.3');

      % Display results
      disp('Reachable Tube Bounds:');
      disp('Lower bounds:');
      disp(min_x');
      disp('Upper bounds:');
      disp(max_x');

end


%% for the 2nd approach (incremental step)

% function fpa_reach()
% 
%     % Parameters
%     reachstep = 0.05; % Step size to compute reach sets
%     initial_time = 0; % Starting time for the first phase
%     intermediate_time = 1; % Intermediate time to define the new initial set
%     final_time = 2; % Final time
%     Initial_radius = 0.01; % Uncertainty in dynamics
% 
%     % First phase: Model from t=0 to t=1
%     model1 = NonLinearODE(5, 0, @CTRNN_FPA, reachstep, intermediate_time, eye(5));
%     model1.options.timeStep = reachstep; % Ensure consistency with reachstep
%     model1.options.taylorTerms = 3;
%     model1.options.zonotopeOrder = 10;
%     model1.options.alg = 'lin';
%     model1.options.tensorOrder = 2;
%     model1 = ODEblockLayer(model1, intermediate_time, reachstep, true);
%     nn1 = NN({model1});
% 
%     % Initial states at t=0
%     x0 = [0, -0.58587, 0.8, 0.52323, 0.7]';
%     lb = x0 - Initial_radius;
%     ub = x0 + Initial_radius;
%     init_set = Star(lb, ub);
% 
%     % Compute reachability analysis from t=0 to t=1
%     t = tic;
%     R_tube1 = nn1.reach(init_set);
%     time_phase1 = toc(t);
%     fprintf('Phase 1 computation time (t=0 to t=1): %.2f seconds\n', time_phase1);
%     fprintf('Length of R_tube1: %d\n', length(R_tube1)); % Debug output
% 
%     % Extract the reachable set at t=1 or the last available set
%     expected_steps = round(intermediate_time / reachstep) + 1; % Expected index for t=1
%     if length(R_tube1) < expected_steps
%         warning('Length of R_tube1 (%d) is less than expected steps (%d). Using the last available set.', length(R_tube1), expected_steps);
%         index_t1 = length(R_tube1); % Use the last set as an approximation
%     else
%         index_t1 = expected_steps; % Use the calculated index
%     end
%     init_set_t1 = R_tube1(index_t1); % Star set at or near t=1
%     fprintf('Using Star set at index %d for t=1\n', index_t1);
% 
%     % Second phase: Model from t=1 to t=2
%     model2 = NonLinearODE(5, 0, @CTRNN_FPA, reachstep, final_time - intermediate_time, eye(5));
%     model2.options.timeStep = reachstep; % Ensure consistency with reachstep
%     model2.options.taylorTerms = 3;
%     model2.options.zonotopeOrder = 10;
%     model2.options.alg = 'lin';
%     model2.options.tensorOrder = 2;
%     model2 = ODEblockLayer(model2, final_time - intermediate_time, reachstep, true);
%     nn2 = NN({model2});
% 
%     % Compute reachability analysis from t=1 to t=2
%     t = tic;
%     R_tube2 = nn2.reach(init_set_t1);
%     time_phase2 = toc(t);
%     fprintf('Phase 2 computation time (t=1 to t=2): %.2f seconds\n', time_phase2);
% 
%     % Combine the tubes (optional, for reference; only R_tube2 is the target interval)
%     R_tube = [R_tube1(1:index_t1), R_tube2]; % Full tube from t=0 to t=2, but we focus on t=1 to t=2
% 
%     % Compute bounding box at t=2 (end of the second phase)
%     final_index = length(R_tube2); % Last Star set corresponds to t=2
%     box_final = R_tube2(final_index).getBox;
%     lb_final = box_final.lb;
%     ub_final = box_final.ub;
% 
%     % Compute reachable tube and points for the second phase (t=1 to t=2)
%     min_x = inf(5, 1);
%     max_x = -inf(5, 1);
%     points = [];
%     for i = 1:length(R_tube2)
%         box = R_tube2(i).getBox;
%         lb = box.lb;
%         ub = box.ub;
%         fprintf('Star set %d (t=1 to t=2): lb = [%s], ub = [%s]\n', i, num2str(lb'), num2str(ub'));
%         min_x = min(min_x, lb);
%         max_x = max(max_x, ub);
% 
%         % Add 8 corner points for the 5D box
%         corners = [
%             lb(1) lb(2) lb(3) lb(4) lb(5);
%             lb(1) lb(2) lb(3) lb(4) ub(5);
%             lb(1) lb(2) lb(3) ub(4) lb(5);
%             lb(1) lb(2) ub(3) lb(4) lb(5);
%             ub(1) lb(2) lb(3) lb(4) lb(5);
%             ub(1) ub(2) lb(3) lb(4) lb(5);
%             ub(1) lb(2) ub(3) lb(4) lb(5);
%             ub(1) lb(2) lb(3) ub(4) lb(5)
%         ];
%         points = [points; corners];
%     end
% 
%     % Ensure points has 5 columns
%     if size(points, 2) ~= 5
%         error('Points matrix has %d columns, expected 5', size(points, 2));
%     end
%     fprintf('Points matrix size: %s\n', num2str(size(points)));
% 
%     % Compute reachable tube and invariant set for t=1 to t=2
%     reachable_tube = [min_x, max_x]; % 5x2 matrix
%     S = reachable_tube;
% 
%     % Save results
%     fprintf('Saving results to fpa_reach.mat...\n');
%     save('fpa_reach.mat', 'R_tube', 'time_phase1', 'time_phase2', 'reachable_tube', 'S', 'points', '-v7.3');
% 
%     % Display results
%     disp('Reachable Tube Bounds (t=1 to t=2):');
%     disp('Lower bounds:');
%     disp(min_x');
%     disp('Upper bounds:');
%     disp(max_x');
% 
% end