function spiral_reach()
    % Load solver parameters
    run('solver_parameters.m');
    fprintf('Loaded solver: %s\n', func2str(parameters.ode_solver));

    approach = 'initial'; % 'initial' or 'incremental'

    % Parameters
    reachstep = 0.01;
    initial_time = 0;
    intermediate_time = 1;
    final_time = 1;
    Initial_radius = 0.01;

    % Time points
    t_points = initial_time:reachstep:final_time;

    % Initial set (2D)
    x0 = [2.0; 0.0]; 
    lb = x0 - Initial_radius;
    ub = x0 + Initial_radius;
    init_set = Star(lb, ub);

    if strcmp(approach, 'initial')
        % Single-step reachability
        model = NonLinearODE(2, 1, @spiral_non, reachstep, final_time, eye(2));
        model.options.timeStep = reachstep;
        model.options.taylorTerms = 2;
        model.options.zonotopeOrder = 20;
        model.options.alg = 'lin';
        model.options.tensorOrder = 2;
        model = ODEblockLayer(model, final_time, reachstep, true);
        nn = NN({model});

        t = tic;
        R_tube = nn.reach(init_set);
        time = toc(t);
        fprintf('Reachability computation time: %.2f seconds\n', time);

        final_index = length(R_tube);
        box_final = R_tube(final_index).getBox;
        lb_final = box_final.lb;
        ub_final = box_final.ub;

    else
        % Incremental reachability
        model1 = NonLinearODE(2, 1, @spiral_non, reachstep, intermediate_time, eye(2));
        model1.options.timeStep = reachstep;
        model1.options.taylorTerms = 2;
        model1.options.zonotopeOrder = 20;
        model1.options.alg = 'lin';
        model1.options.tensorOrder = 2;
        model1 = ODEblockLayer(model1, intermediate_time, reachstep, true);
        nn1 = NN({model1});

        t = tic;
        R_tube1 = nn1.reach(init_set);
        time_phase1 = toc(t);
        fprintf('Phase 1 time: %.2f seconds\n', time_phase1);

        expected_steps = round(intermediate_time / reachstep) + 1;
        index_t1 = min(expected_steps, length(R_tube1));
        init_set_t1 = R_tube1(index_t1);

        model2 = NonLinearODE(2, 1, @spiral_non, reachstep, final_time - intermediate_time, eye(2));
        model2.options.timeStep = reachstep;
        model2.options.taylorTerms = 3;
        model2.options.zonotopeOrder = 10;
        model2.options.alg = 'lin';
        model2.options.tensorOrder = 2;
        model2 = ODEblockLayer(model2, final_time - intermediate_time, reachstep, true);
        nn2 = NN({model2});

        t = tic;
        R_tube2 = nn2.reach(init_set_t1);
        time_phase2 = toc(t);
        fprintf('Phase 2 time: %.2f seconds\n', time_phase2);

        final_index = length(R_tube2);
        box_final = R_tube2(final_index).getBox;
        lb_final = box_final.lb;
        ub_final = box_final.ub;
    end

    % Store reachable sets
    n_t = length(t_points);
    n_x = 2;
    succ_low_all_methods = NaN(n_x, n_t);
    succ_up_all_methods = NaN(n_x, n_t);

    if strcmp(approach, 'initial')
        for k = 1:n_t
            if k <= length(R_tube)
                box = R_tube(k).getBox;
                succ_low_all_methods(:, k) = box.lb;
                succ_up_all_methods(:, k) = box.ub;
            end
        end
    else
        expected_steps_phase1 = round(intermediate_time / reachstep) + 1;
        for k = 1:expected_steps_phase1
            if k <= length(R_tube1)
                box = R_tube1(k).getBox;
                succ_low_all_methods(:, k) = box.lb;
                succ_up_all_methods(:, k) = box.ub;
            end
        end
        for k = (expected_steps_phase1 + 1):n_t
            idx = k - expected_steps_phase1;
            if idx <= length(R_tube2)
                box = R_tube2(idx).getBox;
                succ_low_all_methods(:, k) = box.lb;
                succ_up_all_methods(:, k) = box.ub;
            end
        end
    end

    % Monte Carlo sampling
    num_samples = 1000;
    final_states = zeros(2, num_samples);
    fprintf('Simulating %d samples...\n', num_samples);
    t_start = tic;
    for i = 1:num_samples
        x0_sample = lb + rand(2,1) .* (ub - lb);
        [~, x_traj] = parameters.ode_solver(@(t, x) spiral_non(x, zeros(1,1)), [initial_time final_time], x0_sample, parameters.ode_options);
        final_states(:,i) = x_traj(end,:)';
    end
    t_samples = toc(t_start);
    fprintf('Simulation time: %.2f seconds\n', t_samples);

    % Area computation (only one projection possible in 2D)
    width = ub_final(1) - lb_final(1);
    height = ub_final(2) - lb_final(2);
    nnv_bounding_area = width * height;
    x_min = min(final_states(1, :));
    x_max = max(final_states(1, :));
    y_min = min(final_states(2, :));
    y_max = max(final_states(2, :));
    area_actual = (x_max - x_min) * (y_max - y_min);
    nnv_tightness_ratio = nnv_bounding_area / area_actual;

    fprintf('\nTightness Metric (Area Ratio) in 2D:\n');
    fprintf('NNV Star Set: %.2f\n', nnv_tightness_ratio);
    fprintf('Bounding Box Area = %.6f, Actual Area = %.6f\n', nnv_bounding_area, area_actual);

    % Save results
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename_base = sprintf('reachability_spiral_NNV_%s_%s', approach, timestamp);
    save(filename_base, 'succ_low_all_methods', 'succ_up_all_methods', 'final_states', ...
        't_points', 'n_x', 'nnv_bounding_area', 'area_actual', 'nnv_tightness_ratio', '-v7.3');
    disp(['Data saved to ' filename_base '.mat']);
end
