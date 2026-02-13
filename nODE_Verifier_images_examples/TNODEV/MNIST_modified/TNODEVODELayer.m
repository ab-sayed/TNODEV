% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 29 January 2026
% Last update: 09 February 2026 
% Last revision:  2026

% %------------- BEGIN OF CODE V1.1  --------------
% classdef TNODEVODELayer < handle
%     % TNODEVodeLayer: A custom NNV layer wrapper for `NODE_RA.m` function
% 
%     properties
%         params % Structure to hold parameters required by NODE_RA (system_choice, t_final, etc.)
%     end
% 
%     methods
%         % Constructor
%         function obj = TNODEVODELayer(params)
%             obj.params = params;
%         end
% 
%         % Concrete execution (required if you run net.evaluate, but optional for reachability only)
%         function y = evaluate(obj, x)
%             % This is for single input simulation. 
%             % If you don't need concrete simulation, you can leave this empty or 
%             % implement a simple ODE solver call here.
%             error('Concrete evaluation not implemented for TNODEVODELayer yet.');
%         end
% 
%         % Reachability Method (The core function NNV calls)
%         function S = reach(obj, inputSet, ~)
%             % inputSet: This will be a Star or ImageStar object from the previous layer
%             % ~: Ignored argument (NNV often passes 'method' here)
% 
%             % Conversion of star-set to Interval ---------------------->IMP
%             % 1. Extract bounds from the input Star set
%             if isa(inputSet, 'ImageStar')
%                 % If it's an ImageStar, we need to flatten it or get ranges directly
%                  [lb, ub] = inputSet.getRanges();
%             elseif isa(inputSet, 'Star')
%                  [lb, ub] = inputSet.getRanges();
%             else
%                 error('TNODEVODELayer expects Input to be Star or ImageStar.');
%             end
% 
%             % 2. Prepare parameters for NODE_RA
%             % We update the radius/center based on the current input set
%             current_params = obj.params;
%             current_params.x0 = (lb + ub) / 2;
%             current_params.Initial_radius = (ub - lb) / 2;
% 
%             % 3. Call your custom function
%             % Ensure NODE_RA is in your path
%             results = NODE_RA(current_params);
% 
%             % 4. Convert results back to a Star Set
%             % NODE_RA returns intersection bounds (intervals)
%             new_lb = results.intersect_low;
%             new_ub = results.intersect_up;
% 
%             % Conversion of Interval to star-set ---------------------->IMP
%             % Create a new Star set representing the hyperrectangle [new_lb, new_ub]
%             S = Star(new_lb, new_ub);
%         end
%     end
% end
% %------------- END OF CODE V1.1  --------------

% %------------- BEGIN OF CODE V1.2  --------------
classdef TNODEVODELayer < handle
    % TNODEVodeLayer: A custom NNV layer wrapper for NODE_RA_NEW.m` function
    % This layer bridges NNV's Star sets with the interval-based NODE_RA_NEW.m function
    
    properties
        params % Structure to hold parameters required by NODE_RA_New.m (system_choice, t_final, A, B, etc.)
    end
    
    methods
        % Constructor
        function obj = TNODEVODELayer(params)
            obj.params = params;
        end
        
        % Concrete execution (required if you run net.evaluate, but optional for reachability only)
        function y = evaluate(obj, x)
            % This is for single input simulation. 
            % If you don't need concrete simulation, you can leave this empty or 
            % implement a simple ODE solver call here.
            % dx/dt = Ax + B
            if isfield(obj.params, 'A') && isfield(obj.params, 'B')
                A = obj.params.A;
                B = obj.params.B;
                t_span = [0, obj.params.t_final];
                
                % Use ODE45 for simulation
                ode_fun = @(t, state) A * state + B;
                [~, y_traj] = ode45(ode_fun, t_span, x);
                y = y_traj(end, :)';
            else
                error('Concrete evaluation requires A and B matrices in params.');
            end
        end
   
        % Reachability Method (The core function NNV calls)
        function S = reach(obj, inputSet, ~)
            % inputSet: This will be a Star or ImageStar object from the previous layer
            % ~: Ignored argument (NNV often passes 'method' here)

            % Conversion of star-set to Interval ---------------------->IMP
            % 1. Extract bounds from the input Star set
            if isa(inputSet, 'ImageStar')
                % If it's an ImageStar, we need to flatten it or get ranges directly
                 [lb, ub] = inputSet.getRanges();
            elseif isa(inputSet, 'Star')
                 [lb, ub] = inputSet.getRanges();
            else
                error('TNODEVODELayer expects Input to be Star or ImageStar.');
            end
            
            % 2. Prepare parameters for NODE_RA_NEW.m
            current_params = obj.params;
            
            % Calculate center (x0) and radius from the current input bounds
            current_params.x0 = (lb + ub) / 2;
            current_params.Initial_radius = (ub - lb) / 2;
            
            % 3. Call our custom RA function
            % Ensure NODE_RA_NEW is in your path
            results = NODE_RA_New(current_params);
            
            % 4. Convert results back to a Star Set
            % NODE_RA_new returns intersection bounds (intervals)
            new_lb = results.intersect_low;
            new_ub = results.intersect_up;
            
            % Conversion of Interval to star-set ---------------------->IMP
            % Create a new Star set representing the hyperrectangle [new_lb, new_ub]
            S = Star(new_lb, new_ub);
        end
    end
end
% %------------- END OF CODE V1.2  --------------