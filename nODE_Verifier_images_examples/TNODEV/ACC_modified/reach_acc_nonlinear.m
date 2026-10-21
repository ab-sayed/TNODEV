%% Original code

% function reach_acc_nonlinear()
% 
%     % Reachability analysis of Adaptive Cruise Control with a neuralODE (nonlinear) as a plant model
% 
%     % safety specification: relative distance > safe distance
%     % dis = x_lead - x_ego  
%     % safe distance between two cars, see here 
%     % https://www.mathworks.com/help/mpc/examples/design-an-adaptive-cruise-control-system-using-model-predictive-control.html
%     % dis_safe = D_default + t_gap * v_ego;
% 
% 
%     %% Load objects
% 
%     % Load controller
%     net = load_NN_from_mat('controller_5_20.mat');
% 
%     % Load plant from neuralODE function @tanh_plant
%     reachStep = 0.01;
%     controlPeriod = 0.1;
%     states = 8;
%     C = eye(states); C(7,7) = 0; C(end) = 0;
%     plant = NonLinearODE(8,1,@tanh_plant,reachStep,controlPeriod,C);
%     plant.options.tensorOrder = 2;
% 
% 
%     %% Reachability analysis
% 
%     % Set reachability options
%     % initial condition of x_lead
%     xlead = [90 110];
%     % initial condition of v_lead
%     v_lead = [32 32.2];
%     % initial condition of x_ego
%     x_ego = [10 11]; 
%     % initial condition of v_ego
%     v_ego = [30 30.2];
%     % input set 
%     U = Star(0,0);
%     % initial state set
%     lb = [xlead(1);v_lead(1);0;x_ego(1);v_ego(1);0;0;-2]; %lower bound
%     ub = [xlead(2);v_lead(2);0;x_ego(2);v_ego(2);0;0;-2]; % upper bound
%     X0 = Star(lb,ub); %
%     map_mat = [0 0 0 0 1 0 0 0;
%                 1 0 0 -1 0 0 0 0;
%                 0 1 0 0 -1 0 0 0];
%     U_fix = Star([30;1.4],[30;1.4]); % vset and tgap
% 
%     % Perform reachability analysis
%     trajR = X0;
%     trajU = [];
%     N = 50;
%     % Start computation
%     t = tic;
%     for k=1:N   % RA loop
%         R1 = plant.stepReachStar(X0,U); % reachability of plant
%         X0 = R1(end); % Get only last reach set (reach set at control period)
%         trajR = [trajR X0]; % Keep track of trajectory of NNCS
%         ppp = X0.affineMap(map_mat,[]);
%         Uin = U_fix.concatenate(ppp);
%         Rc = net.reach(Uin);
%         trajU = [trajU Rc];
%         x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
%         X0 = X0.affineMap(C(1:6,:),[]); % Get set for variables 1 to 6
%         X0 = X0.concatenate(Rc); % Add state/input 7 (a_ego)
%         X0 = X0.concatenate(x08);
%     end
%     rT = toc(t);
% 
%     % Save results
%     save('results_tanhplant.mat','rT',"trajR","trajU");
% 
%     %% Visualize results
% 
%     % Transform reach sets for visualization
%     t_gap = 1.4;
%     D_default = 10;
%     alp = 1;
%     outAll = [];
%     safe_dis = [];
%     for i=1:length(trajR)
%         outAll = [outAll trajR(i).affineMap([1 0 0 -1 0 0 0 0], [])]; % distance between cars
%         safe_dis = [safe_dis trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default)]; % safe distance
%     end
%     times = 0:0.1:0.1*N; % to plot in x-axis (time)
% 
%     % Create figure
%     f = figure;
%     hold on;
%     pb = plot(0,85,'m');
%     pr = plot(0,85,'k');
%     Star.plotRanges_2D(outAll,1,times,'k');
%     hold on;
%     Star.plotRanges_2D(safe_dis,1,times,'m');
% 
%     % Enhance figure for paper
%     ax = gca; % Get current axis
%     ax.XAxis.FontSize = 15; % Set font size of axis
%     ax.YAxis.FontSize = 15;
%     xlabel('Time (s)');
%     ylabel('Distance (m)')
%     ylim([40 110])
%     legend([pr,pb],{'rel dist','safe dist'},"Location","best",'FontSize',14);
% 
%     % Save figure
%     if is_codeocean
%         exportgraphics(f,'/results/logs/acc_nl.pdf','ContentType','vector');
%         saveas(f,'/results/logs/acc_nl.png');
%     else
%         exportgraphics(f,'../../acc_nl.pdf','ContentType','vector');
%     end
% 
% end

%% Code with time checks for the RA + Verification

function reach_acc_nonlinear()
    
    % Reachability analysis of Adaptive Cruise Control with a neuralODE (nonlinear) as a plant model

    % safety specification: relative distance > safe distance
    % dis = x_lead - x_ego  
    % safe distance between two cars, see here 
    % https://www.mathworks.com/help/mpc/examples/design-an-adaptive-cruise-control-system-using-model-predictive-control.html
    % dis_safe = D_default + t_gap * v_ego;


    %% Load objects
    
    % Load controller
    net = load_NN_from_mat('controller_5_20.mat');

    % Load plant from neuralODE function @tanh_plant
    reachStep = 0.01;
    controlPeriod = 0.1;
    states = 8;
    C = eye(states); C(7,7) = 0; C(end) = 0;
    plant = NonLinearODE(8,1,@tanh_plant,reachStep,controlPeriod,C);
    plant.options.tensorOrder = 2;
    

    %% Reachability analysis

    % Set reachability options
    % initial condition of x_lead
    xlead = [90 110];
    % initial condition of v_lead
    v_lead = [32 32.2];
    % initial condition of x_ego
    x_ego = [10 11]; 
    % initial condition of v_ego
    v_ego = [30 30.2];
    % input set 
    U = Star(0,0);
    % initial state set
    lb = [xlead(1);v_lead(1);0;x_ego(1);v_ego(1);0;0;-2]; %lower bound
    ub = [xlead(2);v_lead(2);0;x_ego(2);v_ego(2);0;0;-2]; % upper bound
    X0 = Star(lb,ub); %
    map_mat = [0 0 0 0 1 0 0 0;
                1 0 0 -1 0 0 0 0;
                0 1 0 0 -1 0 0 0];
    U_fix = Star([30;1.4],[30;1.4]); % vset and tgap
    
    % Perform reachability analysis
    trajR = X0;
    trajU = [];
    N = 50;
    
    % --- Start Full Verification Timer ---
    t_full = tic;
    
    % --- Start Computation Timer (Reachability Only) ---
    t_reach = tic;
    for k=1:N
        R1 = plant.stepReachStar(X0,U); % reachability of plant
        X0 = R1(end); % Get only last reach set (reach set at control period)
        trajR = [trajR X0]; % Keep track of trajectory of NNCS
        ppp = X0.affineMap(map_mat,[]);
        Uin = U_fix.concatenate(ppp);
        Rc = net.reach(Uin);
        trajU = [trajU Rc];
        x08 = X0.affineMap([0 0 0 0 0 0 0 1],[]);
        X0 = X0.affineMap(C(1:6,:),[]); % Get set for variables 1 to 6
        X0 = X0.concatenate(Rc); % Add state/input 7 (a_ego)
        X0 = X0.concatenate(x08);
    end
    rT = toc(t_reach); % End Reachability Timer

    %% Extract Sets and Check Specification
    t_gap = 1.4;
    D_default = 10;
    alp = 1;
    outAll = [];
    safe_dis = [];
    
    is_safe = true; % Assume safe until proven otherwise

    for i=1:length(trajR)
        % Map to relative distance and safe distance Stars
        rel_dist_star = trajR(i).affineMap([1 0 0 -1 0 0 0 0], []);
        safe_dist_star = trajR(i).affineMap([0 0 0 0 alp*t_gap 0 0 0], alp*D_default);
        
        outAll = [outAll rel_dist_star]; 
        safe_dis = [safe_dis safe_dist_star]; 
        
        % Mathematical Safety Check (rel_dist > safe_dist)
        [min_rel_dist, ~] = rel_dist_star.getRanges();
        [~, max_safe_dist] = safe_dist_star.getRanges();
        
        if min_rel_dist <= max_safe_dist
            is_safe = false;
        end
    end
    
    % --- End Full Verification Timer ---
    total_time = toc(t_full);

    % Save results
    save('results_tanhplant.mat','rT',"trajR","trajU");
    
    %% Print Output Summaries
    fprintf('\n=================================================\n');
    fprintf('VERIFICATION SUMMARY\n');
    fprintf('=================================================\n');
    if is_safe
        fprintf('Result: SAFE (Relative distance is always > Safe distance)\n');
    else
        fprintf('Result: UNSAFE (Safety violation detected)\n');
    end
    fprintf('Reachability Loop Time: %.4f seconds\n', rT);
    fprintf('Full Verification Time: %.4f seconds\n', total_time);
    fprintf('=================================================\n\n');
    
    %% Visualize results
    times = 0:0.1:0.1*N; % to plot in x-axis (time)

    % Create figure
    f = figure;
    hold on;
    pb = plot(0,85,'m');
    pr = plot(0,85,'k');
    Star.plotRanges_2D(outAll,1,times,'k');
    hold on;
    Star.plotRanges_2D(safe_dis,1,times,'m');

    % Enhance figure for paper
    ax = gca; % Get current axis
    ax.XAxis.FontSize = 15; % Set font size of axis
    ax.YAxis.FontSize = 15;
    xlabel('Time (s)');
    ylabel('Distance (m)')
    ylim([40 110])
    legend([pr,pb],{'rel dist','safe dist'},"Location","best",'FontSize',14);

    % Save figure locally as both PNG and PDF
    fprintf('Saving figures to your current directory...\n');
    exportgraphics(f, 'acc_nl.pdf', 'ContentType', 'vector');
    saveas(f, 'acc_nl.png');
    fprintf('Done!\n');

end