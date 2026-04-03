% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 29 January 2026
% Last update: 17 March 2026 
% Last revision:  2026

%%

% %------------- BEGIN OF CODE V1.4  -------------- |--> Added getBox()
% instead of getRanges()

% classdef TNODEVODELayer < handle
%     % TNODEVodeLayer: A custom NNV layer wrapper for NODE_RA_NEW.m` function
%     % This layer bridges NNV's Star sets with the interval-based NODE_RA_NEW.m function
%     % IMPROVEMENTS:
%     % - Generic 'evaluate' method: Uses a function handle for dynamics which
%     % links directly to System_description.m file 
% 
%     properties
%         % system_description % Core system definition (can be a struct, object, or function handle)
%         params % Structure to hold parameters required by NODE_RA_New.m (system_choice, t_final, A, B, etc.)
%     end
% 
%     methods
%         % Constructor
%         function obj = TNODEVODELayer(params)
%             % params:  Struct containing 't_final', 'timeStep', etc.
%             obj.params = params;
% 
%             % - INITIALIZE GLOBALS FOR SYSTEM DESCRIPTION -
%             % This is the "Link" that makes System_description.m work.
%             % We pull A and B directly from the input params.
%             global system_choice Aout_global Bout_global
% 
%             system_choice = obj.params.system_choice;
% 
%             % Only set A/B globals if they exist in params (Specific to case 16)
%             if isfield(obj.params, 'A') && isfield(obj.params, 'B')
%                 % Aout_global = obj.params.A;
%                 % Bout_global = obj.params.B;
% 
%                 % FIX: Cast to double to make ode45 happy
%                 Aout_global = double(obj.params.A);
%                 Bout_global = double(obj.params.B);
%             end
%         end
% 
%         % --- Generic evaluate method ---
% 
%         % function y = evaluate(obj, x)   
%         %     % 1. Ensure globals are active (in case they were cleared)
%         %     global system_choice Aout_global Bout_global
%         %     system_choice = obj.params.system_choice;
%         % 
%         %     if isfield(obj.params, 'A') && isfield(obj.params, 'B')
%         %         Aout_global = obj.params.A;
%         %         Bout_global = obj.params.B;
%         %     end
%         % 
%         %     % 2. Define generic function handle pointing to System_description.m
%         %     % Signature: dx = f(t, x, p) -> we pass [] for p
%         %     ode_fun = @(t, state) System_description(t, state, []);
%         % 
%         %     % 3. Run Simulation
%         %     t_span = [0, obj.params.t_final];
%         %     [~, y_traj] = ode45(ode_fun, t_span, x);
%         %     y = y_traj(end, :)';
%         % end
% 
%         % FIX to remove the ode45 warnings printed
%         function y = evaluate(obj, x)   
%             % 1. Ensure globals are active
%             global system_choice Aout_global Bout_global
%             system_choice = obj.params.system_choice;
% 
%             if isfield(obj.params, 'A') && isfield(obj.params, 'B')
%                 % Aout_global = obj.params.A;
%                 % Bout_global = obj.params.B;
% 
%                 % FIX: Cast to double here as well for safety
%                 Aout_global = double(obj.params.A);
%                 Bout_global = double(obj.params.B);
%             end
% 
%             % 2. Define generic function handle
%             ode_fun = @(t, state) System_description(t, state, []);
% 
%             % 3. Run Simulation (FIX: FORCE DOUBLE PRECISION)
%             t_span = [0, obj.params.t_final];
% 
%             % Casting x to double prevents "Mixture of single/double" warnings
%             [~, y_traj] = ode45(ode_fun, t_span, double(x));
%             y = y_traj(end, :)';
%         end
% 
% 
%         % Reachability Method (The core function NNV calls)
%         function S = reach(obj, inputSet, ~)
%             % inputSet: This will be a Star or ImageStar object from the previous layer
%             % ~: Ignored argument (NNV often passes 'method' here)
% 
%             % 1. Extract bounds from the input Star set using `getBox()`
%             if isa(inputSet, 'ImageStar') || isa(inputSet, 'Star')
%                  % Handle potential arrays of Stars (if ReLU caused splitting)
%                  num_stars = length(inputSet);
% 
%                  % Pre-calculate the size based on the first element to preallocate
%                  % first_box = inputSet(1).getBox();
% 
%                  % --- PATCH: Handle ImageStar from NNV FlattenLayer ---
%                 if isa(inputSet(1), 'ImageStar')
%                     % toStar() perfectly preserves the exact mathematical dependencies 
%                     % while converting the object to a standard 1D Star class
%                     star_input = inputSet(1).toStar();
%                     first_box = star_input.getBox();
%                 else
%                     first_box = inputSet(1).getBox();
%                 end
% 
%                  dim_size = length(first_box.lb(:));
% 
%                  lb_all = zeros(dim_size, num_stars);
%                  ub_all = zeros(dim_size, num_stars);
% 
%                  % for i = 1:num_stars
%                  %     B = inputSet(i).getBox();
%                  %     % Ensure column vectors
%                  %     lb_all(:, i) = reshape(B.lb, [], 1);
%                  %     ub_all(:, i) = reshape(B.ub, [], 1);
%                  % end
% 
%                  for i = 1:num_stars
%                      % --- PATCH: Handle ImageStar inside the loop ---
%                      if isa(inputSet(i), 'ImageStar')
%                          temp_star = inputSet(i).toStar();
%                          B = temp_star.getBox();
%                      else
%                          B = inputSet(i).getBox();
%                      end
% 
%                      % Ensure column vectors
%                      lb_all(:, i) = reshape(B.lb, [], 1);
%                      ub_all(:, i) = reshape(B.ub, [], 1);
%                  end
% 
%                  % Take the absolute Interval Hull across all potential splits
%                  lb = min(lb_all, [], 2);
%                  ub = max(ub_all, [], 2);
%             else
%                 error('TNODEVODELayer expects Input to be Star or ImageStar.');
%             end
% 
%             % 2. Prepare parameters for NODE_RA_NEW.m 
%             current_params = obj.params;
% 
%             % Calculate center (x0) and radius from the current input bounds
%             current_params.x0 = (lb + ub) / 2;
%             current_params.Initial_radius = (ub - lb) / 2;
% 
%             % 3. Call our custom RA function
%             % Ensure NODE_RA_NEW.m is in your path
%             results = NODE_RA_New(current_params);
% 
%             % 4. Convert results back to a Star Set
%             % NODE_RA_new returns intersection bounds (intervals)
%             new_lb = results.intersect_low;
%             new_ub = results.intersect_up;
% 
%             % Conversion of Interval to star-set
%             % Create a new Star set representing the hyperrectangle [new_lb, new_ub]
%             S = Star(new_lb, new_ub);
%         end
%     end
% end

% %------------- END OF CODE V1.4  --------------

%%
% %------------- BEGIN OF CODE V1.5  -------------- |--> Added getBoxFast()
% instead of getBox()

classdef TNODEVODELayer < handle
    % TNODEVodeLayer: A custom NNV layer wrapper for NODE_RA_NEW.m` function
    % This layer bridges NNV's Star sets with the interval-based NODE_RA_NEW.m function
    % IMPROVEMENTS:
    % - Generic 'evaluate' method: Uses a function handle for dynamics which
    % links directly to System_description.m file 

    properties
        % system_description % Core system definition (can be a struct, object, or function handle)
        params % Structure to hold parameters required by NODE_RA_New.m (system_choice, t_final, A, B, etc.)
    end

    methods
        % Constructor
        function obj = TNODEVODELayer(params)
            % params:  Struct containing 't_final', 'timeStep', etc.
            obj.params = params;

            % - INITIALIZE GLOBALS FOR SYSTEM DESCRIPTION -
            % This is the "Link" that makes System_description.m work.
            % We pull A and B directly from the input params.
            global system_choice Aout_global Bout_global
            
            system_choice = obj.params.system_choice;

            % Only set A/B globals if they exist in params (Specific to case 16)
            if isfield(obj.params, 'A') && isfield(obj.params, 'B')
                % Aout_global = obj.params.A;
                % Bout_global = obj.params.B;

                % FIX: Cast to double to make ode45 happy
                Aout_global = double(obj.params.A);
                Bout_global = double(obj.params.B);
            end
        end

        % --- Generic evaluate method ---

        % function y = evaluate(obj, x)   
        %     % 1. Ensure globals are active (in case they were cleared)
        %     global system_choice Aout_global Bout_global
        %     system_choice = obj.params.system_choice;
        % 
        %     if isfield(obj.params, 'A') && isfield(obj.params, 'B')
        %         Aout_global = obj.params.A;
        %         Bout_global = obj.params.B;
        %     end
        % 
        %     % 2. Define generic function handle pointing to System_description.m
        %     % Signature: dx = f(t, x, p) -> we pass [] for p
        %     ode_fun = @(t, state) System_description(t, state, []);
        % 
        %     % 3. Run Simulation
        %     t_span = [0, obj.params.t_final];
        %     [~, y_traj] = ode45(ode_fun, t_span, x);
        %     y = y_traj(end, :)';
        % end

        % FIX to remove the ode45 warnings printed
        function y = evaluate(obj, x)   
            % 1. Ensure globals are active
            global system_choice Aout_global Bout_global
            system_choice = obj.params.system_choice;
            
            if isfield(obj.params, 'A') && isfield(obj.params, 'B')
                % Aout_global = obj.params.A;
                % Bout_global = obj.params.B;
               
                % FIX: Cast to double here as well for safety
                Aout_global = double(obj.params.A);
                Bout_global = double(obj.params.B);
            end
            
            % 2. Define generic function handle
            ode_fun = @(t, state) System_description(t, state, []);
            
            % 3. Run Simulation (FIX: FORCE DOUBLE PRECISION)
            t_span = [0, obj.params.t_final];
            
            % Casting x to double prevents "Mixture of single/double" warnings
            [~, y_traj] = ode45(ode_fun, t_span, double(x));
            y = y_traj(end, :)';
        end
       

        % Reachability Method (The core function NNV calls)
        function S = reach(obj, inputSet, ~)
            % inputSet: This will be a Star or ImageStar object from the previous layer
            % ~: Ignored argument (NNV often passes 'method' here)
            
            % 1. Extract bounds from the input Star set using `getBox()`
            if isa(inputSet, 'ImageStar') || isa(inputSet, 'Star')
                 % Handle potential arrays of Stars (if ReLU caused splitting)
                 num_stars = length(inputSet);

                 % Pre-calculate the size based on the first element to preallocate
                 % first_box = inputSet(1).getBox();
                 
                
                 % --- PATCH: Handle ImageStar & Optimize Bounding Box ---
                 if isa(inputSet(1), 'ImageStar')
                     star_input = inputSet(1).toStar();
                     first_box = star_input.getBoxFast(); % Optimized
                 else
                     first_box = inputSet(1).getBoxFast(); % Optimized
                 end
                 
                 dim_size = length(first_box.lb(:));
                 lb_all = zeros(dim_size, num_stars);
                 ub_all = zeros(dim_size, num_stars);
                 for i = 1:num_stars
                     % --- PATCH: Handle ImageStar inside the loop ---
                     if isa(inputSet(i), 'ImageStar')
                         temp_star = inputSet(i).toStar();
                         B = temp_star.getBoxFast(); % Optimized
                     else
                         B = inputSet(i).getBoxFast(); % Optimized
                     end
                     
                     % Ensure column vectors
                     lb_all(:, i) = reshape(B.lb, [], 1);
                     ub_all(:, i) = reshape(B.ub, [], 1);
                 end

                 % Take the absolute Interval Hull across all potential splits
                 lb = min(lb_all, [], 2);
                 ub = max(ub_all, [], 2);
            else
                error('TNODEVODELayer expects Input to be Star or ImageStar.');
            end
           
            % 2. Prepare parameters for NODE_RA_NEW.m 
            current_params = obj.params;

            % Calculate center (x0) and radius from the current input bounds
            current_params.x0 = (lb + ub) / 2;
            current_params.Initial_radius = (ub - lb) / 2;
            
            % 3. Call our custom RA function
            % Ensure NODE_RA_NEW.m is in your path
            results = NODE_RA_New(current_params);
            
            % 4. Convert results back to a Star Set
            % NODE_RA_new returns intersection bounds (intervals)
            new_lb = results.intersect_low;
            new_ub = results.intersect_up;
            
            % Conversion of Interval to star-set
            % Create a new Star set representing the hyperrectangle [new_lb, new_ub]
            S = Star(new_lb, new_ub);
        end
    end
end

% %------------- END OF CODE V1.5  --------------
