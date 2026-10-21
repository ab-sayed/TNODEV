% Paper:
%   Mixed Monotonicity Reachability Analysis of Neural ODE: A Trade-Off Between Tightness and Efficiency
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 1st of August 2025
% Last update: 30th of May 2026
% Last revision: 30th of May 2026


%------------- BEGIN CODE --------------

function spiral_reach_TMLR(approach)
    % Handle missing input argument with a prompt or default
    if nargin < 1
        approach = input('Select approach (''initial'' or ''incremental''): ', 's');
        if isempty(approach)
            approach = 'initial'; % Default to initial if no input
        end
    end

    % Input validation for approach
    if ~ismember(approach, {'initial', 'incremental'})
        error('Approach must be ''initial'' or ''incremental''');
    end
    fprintf('Using approach: %s\n', approach);

    % Use MATLAB's default ode45 solver with default options
    ode_solver = @ode45;
    ode_options = odeset();
    % fprintf('Using solver: ode45\n');

    % Parameters
    reachstep = 0.01; % Adjusted to ensure divisibility for the case of final_time=1 / 0.01
    initial_time = 0;
    intermediate_time = 0.01; % for the case of final_time=1
    final_time = 1;
    Initial_radius = 0.2;

    % Time points
    t_points = initial_time:reachstep:final_time;

    % Initial set (2D)
    x0 = [2.0; 0.0];
    lb = x0 - Initial_radius; % [1.8; -0.2]
    ub = x0 + Initial_radius; % [2.2; 0.2]
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
        % fprintf('Length of R_tube: %d\n', final_index); % Debug output
        if isempty(R_tube) || final_index < 1
            error('Reachability computation failed: R_tube is empty');
        end
        final_star_set = R_tube(final_index);
        if ~isa(final_star_set, 'Star')
            error('final_star_set is not a Star object, type is: %s', class(final_star_set));
        end
        box_final = final_star_set.getBox;
        lb_final = box_final.lb;
        ub_final = box_final.ub;

    else % incremental
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
        fprintf('Length of R_tube1: %d, index_t1: %d\n', length(R_tube1), index_t1); % Debug output
        if isempty(R_tube1) || index_t1 < 1
            error('Phase 1 reachability failed: R_tube1 is empty');
        end
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
        fprintf('Length of R_tube2: %d\n', final_index); % Debug output
        if isempty(R_tube2) || final_index < 1
            error('Phase 2 reachability failed: R_tube2 is empty');
        end
        final_star_set = R_tube2(final_index);
        if ~isa(final_star_set, 'Star')
            error('final_star_set is not a Star object, type is: %s', class(final_star_set));
        end
        box_final = final_star_set.getBox;
        lb_final = box_final.lb;
        ub_final = box_final.ub;
    end

    % Store reachable sets (full-set tube)
    % NOTE: computed for completeness; currently unused (not saved/plotted).
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
    % fprintf('Simulating %d samples using ode45...\n', num_samples);
    t_start = tic;
    for i = 1:num_samples
        x0_sample = lb + rand(2,1) .* (ub - lb);
        [~, x_traj] = ode_solver(@(t, x) spiral_non(x, zeros(1,1)), [initial_time final_time], x0_sample, ode_options);
        final_states(:,i) = x_traj(end,:)';
    end
    t_samples = toc(t_start);
    % fprintf('Simulation time: %.2f seconds\n', t_samples);

    % Area computation (only one projection possible in 2D)
    % For full reachable set
    % width = ub_final(1) - lb_final(1);
    % height = ub_final(2) - lb_final(2);
    % nnv_bounding_area = width * height;
    % x_min = min(final_states(1, :));
    % x_max = max(final_states(1, :));
    % y_min = min(final_states(2, :));
    % y_max = max(final_states(2, :));
    % area_actual = (x_max - x_min) * (y_max - y_min);
    % nnv_tightness_ratio = nnv_bounding_area / area_actual;
    %
    % fprintf('\nTightness Metric (Area Ratio) in 2D:\n');
    % fprintf('NNV Star Set (Full Reachable Set): %.2f\n', nnv_tightness_ratio);
    % fprintf('Full Reachable Set Bounding Box Area = %.6f, Actual Area = %.6f\n', nnv_bounding_area, area_actual);

    % Volume-based metric for the full reachable set (NNV 2.0)
    dim = 2; % 2D system
    nnv_volume_full = NaN;
    volume_per_dim_full = NaN;
    volume_box_hull = NaN;
    try
        % True polytope volume of the Star set (via its polyhedral representation)
        P_final = final_star_set.toPolyhedron;
        nnv_volume_full = P_final.volume;   % MPT3 polytope volume
        volume_per_dim_full = nnv_volume_full^(1/dim);

        fprintf('\nVolume Metric for Full Reachable Set (NNV 2.0, polytope volume):\n');
        fprintf('  Dimension n = %d\n', dim);
        fprintf('  Star Set Polytope Volume = %.6e\n', nnv_volume_full);
        fprintf('  Volume^(1/n) (per-dim metric) = %.6f\n', volume_per_dim_full);
    catch e
        warning('Polytope volume computation failed (%s). Falling back to box hull.', e.message);
    end

    % Box-hull fallback / additional metric (volume of the axis-aligned
    % bounding box of the Star set).
    try
        box_widths = ub_final - lb_final;
        volume_box_hull = prod(box_widths);
        volume_per_dim_box_hull = volume_box_hull^(1/dim);

        fprintf('\nVolume Metric for Full Reachable Set (NNV 2.0, box hull):\n');
        fprintf('  Box-Hull Volume = %.6e\n', volume_box_hull);
        fprintf('  Volume^(1/n) (per-dim metric) = %.6f\n', volume_per_dim_box_hull);
    catch e
        warning('Box-hull volume computation failed (%s).', e.message);
    end

    % Plotting main figure
    figure('Name', 'Spiral Reachability Analysis - $x_1 - x_2$');
    hold on;
    h1 = []; % Initialize handle for full reachable set
    if isa(final_star_set, 'Star')
        try
            fprintf('Attempting to plot Polyhedron with red boundaries\n');
            P = final_star_set.toPolyhedron;
            h1 = P.plot('edgecolor', 'r', 'alpha', 0, 'wire', true, 'linewidth', 2, 'DisplayName', 'Full Reachable Set'); % Red thicker boundaries
        catch e
            warning('Failed to plot star set: %s', e.message);
            % Fallback to manual box plotting
            try
                box = final_star_set.getBox;
                if isa(box, 'Box') && isvector(box.lb) && isvector(box.ub)
                    fprintf('Manually plotting box with red boundaries\n');
                    x = [box.lb(1) box.ub(1) box.ub(1) box.lb(1) box.lb(1)];
                    y = [box.lb(2) box.lb(2) box.ub(2) box.ub(2) box.lb(2)];
                    h1 = plot(x, y, 'r-', 'LineWidth', 2, 'DisplayName', 'Full Reachable Set'); % Manual red thicker rectangle
                else
                    warning('Invalid box for plotting');
                end
            catch e2
                warning('Fallback plotting failed: %s', e2.message);
            end
        end
    else
        warning('Cannot plot: final_star_set is not a valid Star object');
    end

    % Plot Monte Carlo sampled points
    h2 = plot(final_states(1, :), final_states(2, :), 'k.', 'MarkerSize', 5, 'DisplayName', 'Sampled points'); % Black dots

    xlabel('$x_1$', 'Interpreter', 'latex');
    ylabel('$x_2$', 'Interpreter', 'latex');
    title('$x_1$ vs. $x_2$', 'Interpreter', 'latex');
    grid on;
    if ~isempty(h1)
        legend([h1, h2], {'Full Reachable Set', 'Sampled points'}, 'Location', 'southeast'); % Legend with explicit handles
    else
        legend(h2, 'Sampled points', 'Location', 'southeast'); % Fallback legend
        warning('No boundaries plotted; legend shows only sampled points');
    end
    hold off;

    % Timestamp / filename for the .mat save
    % NOTE: filename retains the legacy "Boundary" token to stay compatible
    % with existing downstream loaders, even though only full-set data is saved.
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename_base = sprintf('reachability_data_Boundary_sys_spiral_NNV_%s_%s', approach, timestamp);

    % Save full reachable-set vertices for comparison with other Tools & Methods
    full_boundary_vertices = []; % vertices of the FULL reachable set (Nx2)
    if isa(final_star_set, 'Star')
        try
            P = final_star_set.toPolyhedron;
            if ~isempty(P.V)
                full_boundary_vertices = P.V; % Full set vertices as Nx2 matrix
            end
        catch e
            warning('Failed to compute full set vertices: %s', e.message);
        end
    end
    save(filename_base, 'full_boundary_vertices', '-v7.3');
    disp(['Data saved to ' filename_base '.mat']);
end

%------------- END OF CODE --------------
