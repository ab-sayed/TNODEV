% function plot_fpa()
%     %% Process and plot all FPA and Cartpole reachability results
% 
%     %% 3 - Load NNV results
%     % Damped Forced Pendulum
%     fpa_nnv = "fpa_reach.mat";
%     fpa_nnv = load(fpa_nnv);
% 
%     %% 5 - Plot FPA reach sets
%     f_fpa = figure;
%     hold on;
%     Star.plotBoxes_2D_noFill(fpa_nnv.R_tube, 1, 2, 'b');
% 
%     grid;
%     xlabel('$x_1$', 'Interpreter', 'latex');
%     ylabel('$x_2$', 'Interpreter', 'latex');
%     legend('off'); % Turn off the legend
%     set(gca, 'FontSize', 12, 'FontName', 'Times');
% 
%     % Save figures in the current working directory
%     current_dir = pwd; % Get the current working directory
%     if is_codeocean
%         exportgraphics(f_fpa, fullfile('/results/logs', 'fpa.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile('/results/logs', 'fpa.png'));
%     else
%         exportgraphics(f_fpa, fullfile(current_dir, 'fpa.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile(current_dir, 'fpa.png'));
%     end
% end

%% 5D Using Star.plotBoxes_2D_noFill

% function plot_fpa()
%     %% Process and plot all FPA reachability results in 5D via pairwise 2D projections
% 
%     %% Load NNV results
%     fpa_nnv = "fpa_reach.mat";
%     fpa_nnv = load(fpa_nnv);
%     R_tube = fpa_nnv.R_tube; % Reachable sets
%     S = fpa_nnv.S;       % Invariant set [min_x, max_x]
% 
%     %% Plot FPA reachable sets in all pairwise 2D projections
%     f_fpa = figure('Position', [100, 100, 1200, 800]); % Larger figure for subplots
%     hold on;
% 
%     % Define all pairs of dimensions (5 choose 2 = 10 pairs)
%     dims = nchoosek(1:5, 2); % Pairs: [1,2], [1,3], ..., [4,5]
%     num_plots = size(dims, 1); % 10 pairs
%     rows = 2; cols = 5; % Grid layout for 10 subplots
% 
%     for i = 1:num_plots
%         subplot(rows, cols, i);
%         hold on;
% 
%         % Plot reachable sets for the current pair of dimensions
%         dim1 = dims(i, 1);
%         dim2 = dims(i, 2);
%         Star.plotBoxes_2D_noFill(R_tube, dim1, dim2, 'b');
% 
%         % Plot invariant set S as a red rectangle
%         lb = S(:, 1); % Lower bounds
%         ub = S(:, 2); % Upper bounds
%         rectangle('Position', [lb(dim1), lb(dim2), ub(dim1)-lb(dim1), ub(dim2)-lb(dim2)], ...
%                   'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
% 
%         % Styling
%         grid on;
%         xlabel(['$x_', num2str(dim1), '$'], 'Interpreter', 'latex');
%         ylabel(['$x_', num2str(dim2), '$'], 'Interpreter', 'latex');
%         set(gca, 'FontSize', 12, 'FontName', 'Times');
%         legend('off');
%         title(['Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2), '$'], ...
%               'Interpreter', 'latex', 'FontSize', 10);
%     end
% 
%     % Adjust layout
%     sgtitle('FPA Reachable Sets: Pairwise 2D Projections', 'Interpreter', 'latex', 'FontSize', 14);
%     set(gcf, 'Color', 'white');
% 
%     % Save figures in the current working directory
%     current_dir = pwd;
%     if is_codeocean
%         exportgraphics(f_fpa, fullfile('/results/logs', 'fpa_5D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile('/results/logs', 'fpa_5D.png'));
%     else
%         exportgraphics(f_fpa, fullfile(current_dir, 'fpa_5D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile(current_dir, 'fpa_5D.png'));
%     end
% end

%% RT as a whole instead of a sequence of boxes

% function plot_fpa()
%     %% Process and plot FPA reachable tube as a whole in 2D projection
% 
%     %% Load NNV results
%     fpa_nnv = "fpa_reach.mat";
%     fpa_nnv = load(fpa_nnv);
%     R_tube = fpa_nnv.R_tube; % Reachable sets
%     reachable_tube = fpa_nnv.reachable_tube; % Overall bounds
%     S = fpa_nnv.S;       % Invariant set
% 
%     %% Plot reachable tube as a filled region (e.g., x1 vs x2)
%     f_fpa = figure('Position', [100, 100, 600, 500]);
%     hold on;
% 
%     % Define dimensions for 2D projection (e.g., x1 vs x2)
%     dim1 = 1; % x1
%     dim2 = 2; % x2
% 
%     % Plot reachable tube as a filled region
%     lb = reachable_tube(:, 1); % Lower bounds
%     ub = reachable_tube(:, 2); % Upper bounds
%     x = [lb(dim1), ub(dim1), ub(dim1), lb(dim1), lb(dim1)];
%     y = [lb(dim2), lb(dim2), ub(dim2), ub(dim2), lb(dim2)];
%     fill(x, y, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Red filled region with transparency
% 
%     % Plot boundary of reachable tube (approximation)
%     plot([lb(dim1), ub(dim1), ub(dim1), lb(dim1), lb(dim1)], ...
%          [lb(dim2), lb(dim2), ub(dim2), ub(dim2), lb(dim2)], ...
%          'b-', 'LineWidth', 2); % Blue boundary
% 
%     % Plot invariant set S as a green rectangle
%     rectangle('Position', [lb(dim1), lb(dim2), ub(dim1)-lb(dim1), ub(dim2)-lb(dim2)], ...
%               'EdgeColor', 'g', 'LineStyle', '--', 'LineWidth', 1.5);
% 
%     % Styling
%     grid on;
%     xlabel(['$x_', num2str(dim1), '$'], 'Interpreter', 'latex');
%     ylabel(['$x_', num2str(dim2), '$'], 'Interpreter', 'latex');
%     set(gca, 'FontSize', 12, 'FontName', 'Times');
%     title(['Reachable Tube Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2), '$'], ...
%           'Interpreter', 'latex', 'FontSize', 14);
%     set(gcf, 'Color', 'white');
% 
%     % Save figures
%     current_dir = pwd;
%     if is_codeocean
%         exportgraphics(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.png'));
%     else
%         exportgraphics(f_fpa, fullfile(current_dir, 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile(current_dir, 'fpa_tube_2D.png'));
%     end
% end

%% Using Convex Hull
% Loads points from fpa_reach.mat for the Sample Points function
% Use convhull to compute a 2D convex hull of the corner points, creating a smoother boundary
% 
% 
% function plot_fpa()
%     %% Process and plot FPA reachable tube as a whole in 2D projection
% 
%     %% Load NNV results
%     fpa_nnv = "fpa_reach.mat";
%     fpa_nnv = load(fpa_nnv);
%     R_tube = fpa_nnv.R_tube; % Reachable tube: Set of all trajectories [0, final_time]
%     reachable_tube = fpa_nnv.reachable_tube; % Overall bounds
%     S = fpa_nnv.S;       % Sound Invariant set
%     points = fpa_nnv.points; % 2D points for convex hull
% 
%     %% Plot reachable tube as a filled region (e.g., x1 vs x2)
%     f_fpa = figure('Position', [100, 100, 600, 500]);
%     hold on;
% 
%     % Define dimensions for 2D projection (e.g., x1 vs x2)
%     dim1 = 1; % x1
%     dim2 = 2; % x2
% 
%     % Compute convex hull of points
%     if ~isempty(points)
%         k = convhull(points(:,1), points(:,2));
%         fill(points(k,1), points(k,2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Red filled region
%         plot(points(k,1), points(k,2), 'b-', 'LineWidth', 2); % Blue boundary
%     end
% 
%     % Plot sound invariant set S as a green rectangle
%     lb = S(:, 1); % Lower bounds
%     ub = S(:, 2); % Upper bounds
%     rectangle('Position', [lb(dim1), lb(dim2), ub(dim1)-lb(dim1), ub(dim2)-lb(dim2)], ...
%               'EdgeColor', 'g', 'LineStyle', '--', 'LineWidth', 1.5);
% 
%     % Styling
%     grid on;
%     xlabel(['$x_', num2str(dim1), '$'], 'Interpreter', 'latex');
%     ylabel(['$x_', num2str(dim2), '$'], 'Interpreter', 'latex');
%     set(gca, 'FontSize', 12, 'FontName', 'Times');
%     title(['Reachable Tube Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2), '$'], ...
%           'Interpreter', 'latex', 'FontSize', 14);
%     set(gcf, 'Color', 'white');
% 
%     % Save figures
%     current_dir = pwd;
%     if is_codeocean
%         exportgraphics(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.png'));
%     else
%         exportgraphics(f_fpa, fullfile(current_dir, 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile(current_dir, 'fpa_tube_2D.png'));
%     end
% end

%% Working Code
% function plot_fpa()
%     %% Process and plot FPA reachable tube as a whole in 2D projection
% 
%     %% Load NNV results
%     fpa_nnv = "fpa_reach.mat";
%     fpa_nnv = load(fpa_nnv);
%     R_tube = fpa_nnv.R_tube; % Reachable tube: Set of all trajectories [0, final_time]
%     reachable_tube = fpa_nnv.reachable_tube; % Overall bounds
%     S = fpa_nnv.S;       % Sound Invariant set
%     points = fpa_nnv.points; % 2D points for convex hull
% 
%     %% Plot reachable tube as a filled region (e.g., x1 vs x2)
%     f_fpa = figure('Position', [100, 100, 600, 500]);
%     hold on;
% 
%     % Define dimensions for 2D projection (e.g., x1 vs x2)
%     dim1 = 1; % x1
%     dim2 = 2; % x2
% 
%     % Compute convex hull of points
%     if ~isempty(points)
%         k = convhull(points(:,1), points(:,2));
%         h_fill = fill(points(k,1), points(k,2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
%                      'DisplayName', 'RT (interior)'); % Label for filled region
%         h_boundary = plot(points(k,1), points(k,2), 'b-', 'LineWidth', 2, ...
%                          'DisplayName', 'RT (Boundary)'); % Label for boundary
%     end
% 
%     % Plot sound invariant set S as a green rectangle using patch
%     lb = S(:, 1); % Lower bounds
%     ub = S(:, 2); % Upper bounds
%     x = [lb(dim1), ub(dim1), ub(dim1), lb(dim1)];
%     y = [lb(dim2), lb(dim2), ub(dim2), ub(dim2)];
%     % h_rect = patch('XData', x, 'YData', y, 'EdgeColor', 'g', 'LineStyle', '--', 'LineWidth', 1.5, ...
%     %                'FaceColor', 'none', 'DisplayName', 'Sound Invariant Set'); % Label for sound invariant, computed as the tightest bounding box enclosing all reachable sets in RT
% 
%     h_rect = patch('XData', x, 'YData', y, 'EdgeColor', 'g', 'LineStyle', '--', 'LineWidth', 1.5, ...
%                    'FaceColor', 'none', 'DisplayName', 'Sound Set'); % Label for sound invariant, computed as the tightest bounding box enclosing all reachable sets in RT
% 
% 
%     % Styling
%     grid on;
%     xlabel(['$x_', num2str(dim1), '$'], 'Interpreter', 'latex');
%     ylabel(['$x_', num2str(dim2), '$'], 'Interpreter', 'latex');
%     set(gca, 'FontSize', 12, 'FontName', 'Times');
%     title(['Reachable Tube Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2), '$'], ...
%           'Interpreter', 'latex', 'FontSize', 14);
%     set(gcf, 'Color', 'white');
% 
%     % Add legend
%     legend([h_fill, h_boundary, h_rect], 'Location', 'northeast'); % Display legend with all elements
% 
%     % Save figures
%     current_dir = pwd;
%     if is_codeocean
%         exportgraphics(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile('/results/logs', 'fpa_tube_2D.png'));
%     else
%         exportgraphics(f_fpa, fullfile(current_dir, 'fpa_tube_2D.pdf'), 'ContentType', 'vector');
%         saveas(f_fpa, fullfile(current_dir, 'fpa_tube_2D.png'));
%     end
% end

%% 3 plots
function plot_fpa()
    % Process and plot FPA reachable tube in 2D projections for x1-x2, x3-x4, and x4-x5
    % Compute and display the area of the convex hull (blue boundary) for each projection

    try
        % Load NNV results
        fpa_nnv = load('fpa_reach.mat');
        R_tube = fpa_nnv.R_tube; % Reachable tube: Set of all trajectories [0, final_time]
        reachable_tube = fpa_nnv.reachable_tube; % Overall bounds
        S = fpa_nnv.S; % Sound invariant set
        points = fpa_nnv.points; % 5D points for convex hull

        % Check points dimensions
        [num_points, num_dims] = size(points);
        fprintf('Points matrix: %d points, %d dimensions\n', num_points, num_dims);
        if num_dims < 5
            warning('Points matrix has only %d columns; some projections (x3-x4, x4-x5) may be skipped', num_dims);
        end
        % Limit points to avoid memory issues
        if num_points > 1000
            points = points(1:1000, :);
            fprintf('Limited points to 1000 to prevent memory issues\n');
        end

        % Define dimension pairs for projections
        dim_pairs = [1, 2; 3, 4; 4, 5]; % x1-x2, x3-x4, x4-x5
        pair_names = {'x1_x2', 'x3_x4', 'x4_x5'}; % For filenames
        areas = zeros(size(dim_pairs, 1), 1); % Store area for each projection

        % Loop over each dimension pair
        for p = 1:size(dim_pairs, 1)
            dim1 = dim_pairs(p, 1); % First dimension
            dim2 = dim_pairs(p, 2); % Second dimension

            % Skip if dimensions exceed points columns
            if dim1 > num_dims || dim2 > num_dims
                warning('Skipping projection x%d-x%d: points matrix has only %d columns', dim1, dim2, num_dims);
                continue;
            end

            % Create new figure for each projection
            f_fpa = figure('Position', [100, 100, 600, 500]);
            hold on;

            % Plot convex hull of points and compute area
            if ~isempty(points)
                try
                    k = convhull(points(:, dim1), points(:, dim2));
                    % Compute area of the convex hull
                    hull_area = polyarea(points(k, dim1), points(k, dim2));
                    areas(p) = hull_area;
                    fprintf('Area of convex hull for x%d-x%d: %.6f\n', dim1, dim2, hull_area);

                    % Plot filled region and boundary
                    h_fill = fill(points(k, dim1), points(k, dim2), 'r', 'FaceAlpha', 0.3, ...
                                  'EdgeColor', 'none', 'DisplayName', 'RT (interior)');
                    h_boundary = plot(points(k, dim1), points(k, dim2), 'b-', 'LineWidth', 2, ...
                                      'DisplayName', 'RT (Boundary)');
                catch err
                    warning('Failed to compute convex hull for x%d-x%d: %s', dim1, dim2, err.message);
                    close(f_fpa);
                    continue;
                end
            else
                warning('No points available for convex hull in projection x%d-x%d', dim1, dim2);
                close(f_fpa);
                continue;
            end

            % Plot sound invariant set S as a green rectangle using patch
            lb = S(:, 1); % Lower bounds
            ub = S(:, 2); % Upper bounds
            if size(S, 1) >= max(dim1, dim2)
                x = [lb(dim1), ub(dim1), ub(dim1), lb(dim1)];
                y = [lb(dim2), lb(dim2), ub(dim2), ub(dim2)];
                h_rect = patch('XData', x, 'YData', y, 'EdgeColor', 'g', 'LineStyle', '--', ...
                               'LineWidth', 1.5, 'FaceColor', 'none', 'DisplayName', 'Sound Set');
            else
                warning('S matrix does not have enough rows for x%d-x%d projection', dim1, dim2);
                h_rect = [];
            end

            % Styling
            grid on;
            xlabel(['$x_', num2str(dim1), '$'], 'Interpreter', 'latex');
            ylabel(['$x_', num2str(dim2), '$'], 'Interpreter', 'latex');
            set(gca, 'FontSize', 12, 'FontName', 'Times');
            title(['Reachable Tube Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2), '$, Area = ', num2str(hull_area, '%.6f')], ...
                  'Interpreter', 'latex', 'FontSize', 14);
            set(gcf, 'Color', 'white');

            % Without the area
            % title(['Reachable Tube Projection on $x_', num2str(dim1), '$-$x_', num2str(dim2)], ...
            %       'Interpreter', 'latex', 'FontSize', 14);
            % set(gcf, 'Color', 'white');

            % Add legend
            legend_entries = [h_fill, h_boundary];
            legend_names = {'RT (interior)', 'RT (Boundary)'};
            if ~isempty(h_rect)
                legend_entries = [legend_entries, h_rect];
                legend_names = [legend_names, 'Sound Set'];
            end
            legend(legend_entries, legend_names, 'Location', 'northeast');

            % Save figures
            current_dir = pwd;
            if exist('is_codeocean', 'var') && is_codeocean
                exportgraphics(f_fpa, fullfile('/results/logs', ['fpa_tube_', pair_names{p}, '.pdf']), ...
                              'ContentType', 'vector');
                saveas(f_fpa, fullfile('/results/logs', ['fpa_tube_', pair_names{p}, '.png']));
            else
                exportgraphics(f_fpa, fullfile(current_dir, ['fpa_tube_', pair_names{p}, '.pdf']), ...
                              'ContentType', 'vector');
                saveas(f_fpa, fullfile(current_dir, ['fpa_tube_', pair_names{p}, '.png']));
            end

            % Close figure to free memory
            close(f_fpa);
        end

        % Save areas to a file
        save('fpa_areas.mat', 'areas', 'dim_pairs', 'pair_names', '-v7.3');
        disp('Areas of convex hulls for each projection:');
        for p = 1:length(areas)
            if areas(p) > 0
                fprintf('x%d-x%d: %.6f\n', dim_pairs(p,1), dim_pairs(p,2), areas(p));
            end
        end
    catch err
        fprintf('Error in plot_fpa: %s\n', err.message);
        rethrow(err);
    end
end




