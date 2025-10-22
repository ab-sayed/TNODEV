%% Original Code
% % Function defined in a different file for CORA
% clc;clear;
% 
% % Change default options
% options.timeStep = 0.01;
% options.taylorTerms = 2;
% options.zonotopeOrder = 20;
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% % Initial states
% x0 = [2.0;0.0]; % This is like the initial input to the ODEblock (initial state)
% Initial_radius = 0.2; % Uncertainty in dynamics (initial states)
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% Splits = 2;
% % init_set = Star(lb,ub);
% % input_set = Star(0,0); % No inputs, but need to define it
% 
% R0 = interval(lb,ub);
% U = zonotope([0,0]);
% 
% params.R0=zonotope(R0);
% params.U=U;
% params.timeStep=0.01;
% params.tFinal=6
%  sys = nonlinearSys(@spiral_non, 2, 1); % CORA nonlinearSys class
% 
%  % R = reach(sys, params, options); % CORA reach method using zonotope and conservative linearization R = reach(sys, obj.params, obj.options); % CORA reach method using zonotope and conservative linearization
% 
% 
%  %plot(R,[1,2],'FaceColor','r','EdgeColor','none');
%    %plot(R,[1,2],'r');
%   %Star.plotBoxes_2D(Rall,1,2,'k');
% 
% 
%   steps=600;
% 
%   intervalStep(1) = (ub(1)-lb(1))/Splits;
%   intervalStep(2) = (ub(2)-lb(2))/Splits;
%   for i=1:Splits
%       for j =1:Splits
%              lbsplit(1) = lb(1) + (i-1)*intervalStep(1);
%              lbsplit(2) = lb(2) + (j-1)*intervalStep(2);
%              ubsplit(1) = lb(1) + i*intervalStep(1);
%              ubsplit(2) = lb(2) + j*intervalStep(2);
% 
% 
%       R0_splits{(i-1)*Splits+j} = interval(lbsplit', ubsplit');
% 
%       hold on;
%       plot(interval(lbsplit', ubsplit'),[1,2],'b');
%       params.R0= zonotope(R0_splits{(i-1)*Splits+j});
%       R_splits{(i-1)*Splits+j} = reach(sys, params, options);
%       end
%   end
% 
% % for i =1:4
% %  plot(RB{i},[1,2],'FaceColor','b','EdgeColor','none');
% % end
% grid on;
% xlabel('y_{1}');
% ylabel('y_{2}');
% 
% plot(R.timePoint.set{steps},[1,2],'FaceColor','r');
% 
%  for i=1:size(R_splits,2)
%        hold on;
%        plot(R_splits{i}.timePoint.set{steps},[1,2],'FaceColor','b');
%  end 
% 
%  saferegion=interval([0.36;-1.2],[0.8;-0.6]);
% 
%  plot(saferegion,[1,2],'g','Linewidth',2.5);



 %% working
% clc; clear;
% 
% % Change default options
% options.timeStep = 0.01;
% options.taylorTerms = 2;
% options.zonotopeOrder = 20;
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% % Initial states
% x0 = [2.0; 0.0]; % Initial state
% Initial_radius = 0.2; % Uncertainty in initial states
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% Splits = 2; % Number of splits per dimension
% 
% R0 = interval(lb, ub); % Full initial set
% U = zonotope([0, 0]); % No input
% 
% params.R0 = zonotope(R0);
% params.U = U;
% params.timeStep = 0.01;
% params.tFinal = 6; % Note: missing semicolon in your code, added here
% 
% % Define the nonlinear system
% sys = nonlinearSys(@spiral_non, 2, 1); % Ensure @spiral_non is defined
% 
% % Compute steps correctly
% steps = floor(params.tFinal / options.timeStep) + 1; % Should be 601
% disp(['Number of steps: ' num2str(steps)]);
% 
% % Split the initial set and compute reachable sets
% intervalStep(1) = (ub(1) - lb(1)) / Splits;
% intervalStep(2) = (ub(2) - lb(2)) / Splits;
% for i = 1:Splits
%     for j = 1:Splits
%         lbsplit(1) = lb(1) + (i-1) * intervalStep(1);
%         lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
%         ubsplit(1) = lb(1) + i * intervalStep(1);
%         ubsplit(2) = lb(2) + j * intervalStep(2);
% 
%         R0_splits{(i-1)*Splits + j} = interval(lbsplit', ubsplit');
% 
%         % Plot initial split regions
%         hold on;
%         plot(interval(lbsplit', ubsplit'), [1, 2], 'b');
% 
%         params.R0 = zonotope(R0_splits{(i-1)*Splits + j});
%         R_splits{(i-1)*Splits + j} = reach(sys, params, options);
%     end
% end
% 
% % Plotting
% figure; % Explicitly create a new figure
% hold on;
% 
% % Plot reachable sets for each split at all time steps (optional)
% for i = 1:size(R_splits, 2)
%     for j = 1:steps
%         plot(R_splits{i}.timePoint.set{j}, [1, 2], 'b');
%     end
% end
% 
% % Plot final reachable sets for each split
% for i = 1:size(R_splits, 2)
%     plot(R_splits{i}.timePoint.set{steps}, [1, 2], 'FaceColor', 'b');
% end
% 
% % Plot safe region
% saferegion = interval([0.36; -1.2], [0.8; -0.6]);
% plot(saferegion, [1, 2], 'g', 'LineWidth', 2.5);
% 
% % Formatting
% grid on;
% xlabel('y_{1}');
% ylabel('y_{2}');
% hold off;

%%
% clc; clear;
% 
% % Change default options
% options.timeStep = 0.01;
% options.taylorTerms = 2;
% options.zonotopeOrder = 20;
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% % Initial states
% x0 = [2.0; 0.0];
% Initial_radius = 0.2;
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% Splits = 2;
% 
% R0 = interval(lb, ub);
% U = zonotope([0, 0]);
% 
% params.R0 = zonotope(R0);
% params.U = U;
% params.timeStep = 0.01;
% params.tFinal = 6;
% 
% % Define the nonlinear system
% sys = nonlinearSys(@spiral_non, 2, 1);
% 
% % Compute reachable set for full initial set
% R = reach(sys, params, options);
% 
% % Compute steps
% steps = floor(params.tFinal / options.timeStep) + 1; % 601
% disp(['Number of steps: ' num2str(steps)]);
% 
% % Split the initial set and compute reachable sets
% intervalStep(1) = (ub(1) - lb(1)) / Splits;
% intervalStep(2) = (ub(2) - lb(2)) / Splits;
% for i = 1:Splits
%     for j = 1:Splits
%         lbsplit(1) = lb(1) + (i-1) * intervalStep(1);
%         lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
%         ubsplit(1) = lb(1) + i * intervalStep(1);
%         ubsplit(2) = lb(2) + j * intervalStep(2);
% 
%         R0_splits{(i-1)*Splits + j} = interval(lbsplit', ubsplit');
% 
%         hold on;
%         plot(interval(lbsplit', ubsplit'), [1, 2], 'b');
% 
%         params.R0 = zonotope(R0_splits{(i-1)*Splits + j});
%         R_splits{(i-1)*Splits + j} = reach(sys, params, options);
%     end
% end
% 
% % Plotting
% figure;
% hold on;
% 
% % Plot full reachable set (all steps, optional)
% for j = 1:steps
%     plot(R.timePoint.set{j}, [1, 2], 'r');
% end
% 
% % Plot split reachable sets (all steps, optional)
% for i = 1:size(R_splits, 2)
%     for j = 1:steps
%         plot(R_splits{i}.timePoint.set{j}, [1, 2], 'b');
%     end
% end
% 
% % Plot final sets
% plot(R.timePoint.set{steps}, [1, 2], 'FaceColor', 'r');
% for i = 1:size(R_splits, 2)
%     plot(R_splits{i}.timePoint.set{steps}, [1, 2], 'FaceColor', 'b');
% end
% 
% % Plot safe region
% saferegion = interval([0.36; -1.2], [0.8; -0.6]);
% plot(saferegion, [1, 2], 'g', 'LineWidth', 2.5);
% 
% % Formatting
% grid on;
% xlabel('y_{1}');
% ylabel('y_{2}');
% hold off;
%%
clc; clear;

% Change default options
options.timeStep = 0.01;        % Smaller time step for finer control ~ original: 0.01 / Finer: 0.005
options.taylorTerms = 2;         % Taylor terms for nonlinear approximation ~ original: 2 / 4
options.zonotopeOrder = 20;      % Zonotope order for final set ~ original: 20 / 50
% options.intermediateOrder = 10;  % Intermediate zonotope order required for 'poly' ~ This wasn't in the original code
options.alg = 'lin';            % Polynomial approximation ~ check with 'poly' as well as using 'lin' for a non-linear system may lead to over-approximation
options.tensorOrder = 2;

% Initial states
x0 = [2.0; 0.0];
Initial_radius = 0.2;
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

Splits = 2;

R0 = interval(lb, ub);
U = zonotope([0, 0]);

params.R0 = zonotope(R0);
params.U = U;
params.timeStep = 0.01; % original: 0.01 / 0.001 ~ the computation took too much time / 0.005
params.tFinal = 6; % original: 6 / 4/ 2

% Define the nonlinear system
sys = nonlinearSys(@spiral_non, 2, 1);

% Compute steps
steps = floor(params.tFinal / options.timeStep) + 1; % Should be 601 = (6/0.01) + 1 
disp(['Number of steps: ' num2str(steps)]);


% Compute reachable set for full initial set with error handling
tic; % Timer for the full reachable set
try
    R = reach(sys, params, options);
    disp('Full reachable set computed successfully.');
catch e
    disp('Error computing full reachable set:');
    disp(e.message);
    R = []; % Set to empty to skip plotting if failed
end
fullSetTime = toc;
disp(['Time to compute full reachable set: ' num2str(fullSetTime) ' seconds']);

% Split the initial set and compute reachable sets
tic; % Timer for the splitted inital set reachable set
intervalStep(1) = (ub(1) - lb(1)) / Splits;
intervalStep(2) = (ub(2) - lb(2)) / Splits;
R0_splits = cell(Splits * Splits, 1);
R_splits = cell(Splits * Splits, 1);
for i = 1:Splits
    for j = 1:Splits
        lbsplit(1) = lb(1) + (i-1) * intervalStep(1);
        lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
        ubsplit(1) = lb(1) + i * intervalStep(1);
        ubsplit(2) = lb(2) + j * intervalStep(2);

        idx = (i-1) * Splits + j;
        R0_splits{idx} = interval(lbsplit', ubsplit');
        
        % Plot initial split regions
        figure(1);
        hold on;
        plot(interval(lbsplit', ubsplit'), [1, 2], 'b');
        
        params.R0 = zonotope(R0_splits{idx});
        try
            R_splits{idx} = reach(sys, params, options);
            disp(['Split ' num2str(idx) ' computed successfully.']);
        catch e
            disp(['Error computing split ' num2str(idx) ':']);
            disp(e.message);
            R_splits{idx} = []; % Set to empty if failed
        end
    end
end
splitSetTime = toc;
% disp(['Time to compute splitted reachable sets: ' num2str(elapsedTime) ' seconds']);
disp(['Time to compute splitted reachable sets: %.2f seconds\n', splitSetTime]);


% Plotting
figure(1);
hold on;

% Plot full reachable set (if computed)
if ~isempty(R)
    for j = 1:steps
        plot(R.timePoint.set{j}, [1, 2], 'r');
    end
    plot(R.timePoint.set{steps}, [1, 2], 'FaceColor', 'r');
end

% Plot split reachable sets
for i = 1:size(R_splits, 2)
    if ~isempty(R_splits{i})
        for j = 1:steps
            plot(R_splits{i}.timePoint.set{j}, [1, 2], 'b');
        end
        plot(R_splits{i}.timePoint.set{steps}, [1, 2], 'FaceColor', 'b');
    end
end

% Plot safe region
saferegion = interval([0.36; -1.2], [0.8; -0.6]);
plot(saferegion, [1, 2], 'g', 'LineWidth', 2.5);

% Formatting
grid on;
xlabel('y_{1}');
ylabel('y_{2}');
hold off;