%%Original
% % Function defined in a different file for CORA
% clc;clear;
% Neh=2;
% % Change default options
% options.timeStep = 0.01;
% options.taylorTerms = 2;
% options.zonotopeOrder = 20;
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% % Initial states
% x0 = [2.0;0.0]; % This is like the initial input to the ODEblock (initial state)
% Initial_radius = 0.5; % Uncertainty in dynamics (initial states)
% lb = x0 - Initial_radius;
% ub = x0 + Initial_radius;
% 
% % x0 = [-0.001, 0.001]^12;
% 
% 
% % init_set = Star(lb,ub);
% % input_set = Star(0,0); % No inputs, but need to define it
% 
% R0 = interval(lb,ub);
% U = zonotope([0,0]);
% 
% params.R0=zonotope(R0);
% params.U=U;
% params.timeStep=0.01;
% params.tFinal=6;
% sys = nonlinearSys(@spiral_non, 2, 1); % CORA nonlinearSys class
% 
%  % R = reach(sys, params, options); % CORA reach method using zonotope and conservative linearization R = reach(sys, obj.params, obj.options); % CORA reach method using zonotope and conservative linearization
% 
% % Measure the computation time for the reachability analysis
% tic;  % Start timing
% 
%  %plot(R,[1,2],'FaceColor','r','EdgeColor','none');
%    %plot(R,[1,2],'r');
%   %Star.plotBoxes_2D(Rall,1,2,'k');
% 
%   steps = floor(params.tFinal / options.timeStep) + 1; % Should be 601 = (6/0.01) + 1 
%   % steps=600+1;
%   % steps=401;
% 
% SplitsAll = 7; %4~fig (a) or 7~fig (b)
%   SplitsB = 4;
% intervalStep(1) = (ub(1)-lb(1))/SplitsAll;
%   intervalStep(2) = (ub(2)-lb(2))/SplitsAll;
%   for i=1:SplitsAll
%       for j =1:SplitsAll
%              lbsplit(1) = lb(1) + (i-1)*intervalStep(1);
%              lbsplit(2) = lb(2) + (j-1)*intervalStep(2);
%              ubsplit(1) = lb(1) + i*intervalStep(1);
%              ubsplit(2) = lb(2) + j*intervalStep(2);
% 
% 
%       R0_splits{(i-1)*SplitsAll+j} = interval(lbsplit', ubsplit');
% 
%         params.R0=zonotope(R0_splits{(i-1)*SplitsAll+j});
%       R_splits{(i-1)*SplitsAll+j} = reach(sys, params, options);
%       end
%   end
% 
%   count =1;
%   intervalStep(1) = (ub(1)-lb(1))/SplitsB;
%   intervalStep(2) = (ub(2)-lb(2))/SplitsB;
% for i=1:Neh
%     mintemp = R0.inf;
%     maxtemp = R0.sup;
%     mintemp(i) = R0.sup(i);
%     R0B{i}= interval(mintemp,maxtemp);
%     if i==1
%         lbsplit(i) = R0.sup(i);
%         ubsplit(i) = R0.sup(i);
%        for j=1:SplitsB
%       lbsplit(2) = lb(2) + (j-1)*intervalStep(2);
%       ubsplit(2) = lb(2) + j*intervalStep(2);          
%       R0B_splits{count} = interval(lbsplit', ubsplit');
%       count =count+1;
%        end
%     else
%         lbsplit(i) = R0.sup(i);
%         ubsplit(i) = R0.sup(i);
%        for j=1:SplitsB
%       lbsplit(1) = lb(1) + (j-1)*intervalStep(2);
%       ubsplit(1) = lb(1) + j*intervalStep(2);          
%       R0B_splits{count} = interval(lbsplit', ubsplit');
%       count =count+1;
%        end
%     end
% end
% 
% for i=1:Neh
%     mintemp = R0.inf;
%     maxtemp = R0.sup;
%     maxtemp(i) = R0.inf(i);
%     R0B{i+Neh}= interval(mintemp,maxtemp);
%         if i==1
%         lbsplit(i) = R0.inf(i);
%         ubsplit(i) = R0.inf(i);
%        for j=1:SplitsB
%       lbsplit(2) = lb(2) + (j-1)*intervalStep(2);
%       ubsplit(2) = lb(2) + j*intervalStep(2);          
%       R0B_splits{count} = interval(lbsplit', ubsplit');
%       count =count+1;
%        end
%     else
%         lbsplit(i) = R0.inf(i);
%         ubsplit(i) = R0.inf(i);
%        for j=1:SplitsB
%       lbsplit(1) = lb(1) + (j-1)*intervalStep(2);
%       ubsplit(1) = lb(1) + j*intervalStep(2);          
%       R0B_splits{count} = interval(lbsplit', ubsplit');
%       count =count+1;
%        end
%     end
% end
% 
% % for i=1:SplitsB*4
% %     hold on
% %     plot(zonotope(R0B_splits{i}),[1,2],'FaceColor',[rand(1),rand(1),rand(1)]);
% % end
% % 
% % for i=1:SplitsAll^2
% %   hold on
% %     plot(zonotope(R0_splits{i}),[1,2],'FaceColor',[rand(1),rand(1),rand(1)]);
% % end  
% 
% grid on;
% xlabel('y_{1}');
% ylabel('y_{2}');
% 
% %plot(R.timePoint.set{steps},[1,2],'FaceColor','r');
% 
%  for i =1:SplitsAll^2
%     hold on;
%     %plot(pZB{i},[1,2],'y','Splits',2);
%     plot(R_splits{i}.timePoint.set{steps},[1,2],'FaceColor','r');
% end
% 
% for i=1:4*SplitsB
%     params.R0=zonotope(R0B_splits{i});
%     RB_splits{i} = reach(sys, params, options);
%     hold on;
%     plot(RB_splits{i}.timePoint.set{steps},[1,2],'FaceColor','b');
% end
% 
% % Stop timing for the partitioned reachability analysis
% time_partitions = toc;
% fprintf('Computation time for partitioned reachability analysis: %.2f seconds\n', time_partitions);
% 
% saferegion=interval([-0.08;-1.50],[0.90;-0.30]);
% 
%  hold on;
% plot(saferegion,[1,2],'g','Linewidth',2);

%%
clc; clear;

Neh = 2;

% Change default options
options.timeStep = 0.01;        % Smaller time step for finer control
options.taylorTerms = 2;         % More Taylor terms for accuracy
options.zonotopeOrder = 20;      % Higher order to handle complexity, as it allow more generators, accommodating a larger set before aborting
% options.intermediateOrder = 10;  % Required for 'poly'
options.alg = 'lin';            % Better for nonlinear systems
options.tensorOrder = 2;

% Initial states
x0 = [2.0; 0.0];
Initial_radius = 0.5;            
lb = x0 - Initial_radius;
ub = x0 + Initial_radius;

% x0 = [-0.001, 0.001]^12;

% init_set = Star(lb,ub);
% input_set = Star(0,0); % No inputs, but need to define it

R0 = interval(lb, ub);
U = zonotope([0, 0]);

params.R0 = zonotope(R0);
params.U = U;
params.timeStep = 0.01;
params.tFinal = 6;               % Original: 6

sys = nonlinearSys(@spiral_non, 2, 1);

% Compute steps
steps = floor(params.tFinal / options.timeStep) + 1; % Should be 601
disp(['Number of steps: ' num2str(steps)]);

% Compute full reachable set
tic; % Timer for the full reachable set
try
    R = reach(sys, params, options);
    disp('Full reachable set computed successfully.');
catch e
    disp('Error computing full reachable set:');
    disp(e.message);
    R = [];
end
fullSetTime = toc;
fprintf('Computation time for full reachable set: %.2f seconds\n', fullSetTime);

% Compute split reachable sets
tic; % Timer for the splitted reachable sets

SplitsAll = 4; %4~fig (a) or 7~fig (b)
SplitsB = 4;   

% Compute R_splits (all splits)
intervalStep(1) = (ub(1) - lb(1)) / SplitsAll;
intervalStep(2) = (ub(2) - lb(2)) / SplitsAll;
R0_splits = cell(SplitsAll^2, 1);
R_splits = cell(SplitsAll^2, 1);
for i = 1:SplitsAll
    for j = 1:SplitsAll
        lbsplit(1) = lb(1) + (i-1) * intervalStep(1);
        lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
        ubsplit(1) = lb(1) + i * intervalStep(1);
        ubsplit(2) = lb(2) + j * intervalStep(2);

        idx = (i-1) * SplitsAll + j;
        R0_splits{idx} = interval(lbsplit', ubsplit');
        params.R0 = zonotope(R0_splits{idx});
        try
            R_splits{idx} = reach(sys, params, options);
            disp(['R_split ' num2str(idx) ' computed successfully.']);
        catch e
            disp(['Error computing R_split ' num2str(idx) ':']);
            disp(e.message);
            R_splits{idx} = [];
        end
    end
end

% Compute RB_splits (boundary splits)
count = 1;
intervalStep(1) = (ub(1) - lb(1)) / SplitsB;
intervalStep(2) = (ub(2) - lb(2)) / SplitsB;
R0B = cell(2 * Neh, 1);
R0B_splits = cell(4 * SplitsB, 1);
RB_splits = cell(4 * SplitsB, 1);

for i = 1:Neh
    mintemp = R0.inf;
    maxtemp = R0.sup;
    mintemp(i) = R0.sup(i);
    R0B{i} = interval(mintemp, maxtemp);
    if i == 1
        lbsplit(i) = R0.sup(i);
        ubsplit(i) = R0.sup(i);
        for j = 1:SplitsB
            lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
            ubsplit(2) = lb(2) + j * intervalStep(2);
            R0B_splits{count} = interval(lbsplit', ubsplit');
            count = count + 1;
        end
    else
        lbsplit(i) = R0.sup(i);
        ubsplit(i) = R0.sup(i);
        for j = 1:SplitsB
            lbsplit(1) = lb(1) + (j-1) * intervalStep(1);
            ubsplit(1) = lb(1) + j * intervalStep(1);
            R0B_splits{count} = interval(lbsplit', ubsplit');
            count = count + 1;
        end
    end
end

for i = 1:Neh
    mintemp = R0.inf;
    maxtemp = R0.sup;
    maxtemp(i) = R0.inf(i);
    R0B{i + Neh} = interval(mintemp, maxtemp);
    if i == 1
        lbsplit(i) = R0.inf(i);
        ubsplit(i) = R0.inf(i);
        for j = 1:SplitsB
            lbsplit(2) = lb(2) + (j-1) * intervalStep(2);
            ubsplit(2) = lb(2) + j * intervalStep(2);
            R0B_splits{count} = interval(lbsplit', ubsplit');
            count = count + 1;
        end
    else
        lbsplit(i) = R0.inf(i);
        ubsplit(i) = R0.inf(i);
        for j = 1:SplitsB
            lbsplit(1) = lb(1) + (j-1) * intervalStep(1);
            ubsplit(1) = lb(1) + j * intervalStep(1);
            R0B_splits{count} = interval(lbsplit', ubsplit');
            count = count + 1;
        end
    end
end

for i = 1:4 * SplitsB
    params.R0 = zonotope(R0B_splits{i});
    try
        RB_splits{i} = reach(sys, params, options);
        disp(['RB_split ' num2str(i) ' computed successfully.']);
    catch e
        disp(['Error computing RB_split ' num2str(i) ':']);
        disp(e.message);
        RB_splits{i} = [];
    end
end

splitSetTime = toc;
fprintf('Computation time for splitted reachable sets: %.2f seconds\n', splitSetTime);

% Plotting
figure;
hold on;

% Plot R_splits
for i = 1:SplitsAll^2
    if ~isempty(R_splits{i})
        plot(R_splits{i}.timePoint.set{steps}, [1, 2], 'FaceColor', 'r');
    end
end

% Plot RB_splits
for i = 1:4 * SplitsB
    if ~isempty(RB_splits{i})
        plot(RB_splits{i}.timePoint.set{steps}, [1, 2], 'FaceColor', 'b');
    end
end

% Plot safe region
saferegion = interval([-0.08; -1.50], [0.90; -0.30]);
plot(saferegion, [1, 2], 'g', 'LineWidth', 2);

% Formatting
grid on;
xlabel('y_{1}');
ylabel('y_{2}');
hold off;