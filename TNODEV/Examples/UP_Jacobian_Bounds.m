% Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 21 October 2025
% Last update: 21 October 2025
% Last revision: October 2025


%% User-provided bounds on the Jacobian matrices for continuous-time or discrete-time systems
% Used in:
% - OA_methods/TIRA.m
%       to check the requirements for over-approximation methods 1,3,5 in
%       OA_methods/OA_1_CT_Monotonicity.m
%       OA_methods/OA_3_CT_Mixed_Monotonicity.m
%       OA_methods/OA_5_DT_Mixed_Monotonicity.m
% - Utilities/Growth_bound_choice.m
%       for the definition of a growth bound function to be used in
%       over-approximation method 2 in
%       OA_methods/OA_2_CT_Contraction_growth_Bound.m
% - Utilities/Sensitivity_bounds_choice.m
%       to compute bounds on the sensitivity matrices of a continuous-time
%       system, within submethods (4-2) and (4-3), to be used in 
%       over-approximation method 4 in
%       OA_methods/OA_4_CT_Sampled_data_MM.m
% - Utilities/Error_bound_choice.m
%       for the definition of an error bound function to be used in
%       over-approximation method 6 in
%       OA_methods/OA_6_DT_CT_Quasi_Monte_Carlo.m

% The user can either provider global bounds, or a function of the input
% arguments (t_init,t_final,x_low,x_up,p_low,p_up) returning local bounds

% Jacobian definitions:
%   to states:  J_x(t) = d(System_description(t,x,p))/dx
%   to inputs:  J_p(t) = d(System_description(t,x,p))/dp
% Note that Jacobians may contain elements inf or -inf, but not all methods
% may support this.

% List of inputs
%   t_init: initial time
%   t_final: time at which the reachable set is approximated (for continuous-time system only)
%       for discrete-time system, a dummy value can be provided
%   [x_low,x_up]: interval of initial states (at time t_init)
%   [p_low,p_up]: interval of allowed input values

% List of outputs
%   [J_x_low,J_x_up]: bounds of the Jacobian with respect to the state
%   [J_p_low,J_p_up]: bounds of the Jacobian with respect to the input

% Authors:  
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Alex Devonport, <alex_devonport -AT- berkeley.edu>, EECS, UC Berkeley
%
% Date: 21th of October 2025
% Last update: 21th of October 2025
% Last revision: th of October 2025

function [J_x_low,J_x_up,J_p_low,J_p_up] = UP_Jacobian_Bounds(t_init,t_final,x_low,x_up,p_low,p_up)
n_x = length(x_low);
n_p  = length(p_low);

%% Default values as NaN (not a number)
J_x_low = NaN(n_x);
J_x_up = NaN(n_x);
J_p_low = NaN(n_x,n_p);
J_p_up = NaN(n_x,n_p);

%% User-provided Jacobian bounds
% Can be either global bounds
% or local bounds depending on inputs: t_init,t_final,x_low,x_up,p_low,p_up

% If System_description.m has no input variable 'p', uncomment below:
% J_p_low = zeros(n_x,n_p);
% J_p_up = zeros(n_x,n_p);

% ---- 1. Try to read pre-computed Jacobians from globals ----------------
global Jacobian_x_low Jacobian_x_up
if ~isempty(Jacobian_x_low) && ~isempty(Jacobian_x_up)
    J_x_low = Jacobian_x_low;
    J_x_up = Jacobian_x_up;

    % autonomous → ∂f/∂p = 0
    J_p_low = zeros(n_x,n_p);
    J_p_up  = zeros(n_x,n_p);
    return;
end

% ---- 2. Fallback: hard-coded values (kept for backward compatibility) ----

global system_choice
global u        % Some systems need an external control input

switch system_choice
    case 14
        %% FPA System (continuous-time)

        % State Jacobian bounds
        % % for long time t=0-40
        % J_x_low = [-0   0   -1.2033 -0.0720 -0.6028; ...
        %        0   -0   0.4768 -1.5001 0.0127; ...
        %        0    0  -1.5863  0.0534 -0.7954; ...
        %        0    0   0.0344 -0.2995 0.0009; ...
        %        0    0   0.7183 -0.0846 0.0189];
        % J_x_up = [-0   0  -0.4829 -0.0549 -0.0127; ...
        %           0   -0   1.1881 -1.1430 0.6021; ...
        %           0    0  -0.6366  0.0701 -0.0167; ...
        %           0    0   0.0857 -0.2282 0.0442; ...
        %           0    0   1.7898 -0.0645 0.8975];

        % % % 1st Approach (initial step) for t=0-2
        J_x_low = [-0   0   -1.2033   -0.0637   -0.6028; ...
                   0   -0   0.6544   -1.3270    0.0273; ...
                   0    0  -1.5863    0.0534   -0.7954; ...
                   0    0   0.0472   -0.2649    0.0020; ...
                   0    0   0.9859   -0.0748    0.0408];
        J_x_up = [-0   0  -0.6628   -0.0549   -0.0274; ...
                  0   -0   1.1881   -1.1430    0.6021; ...
                  0    0  -0.8738    0.0620   -0.0361; ...
                  0    0   0.0857   -0.2282    0.0442; ...
                  0    0   1.7898   -0.0645    0.8975];

        % % 2nd Approach (incremental approach) for t= 0-1 then for t=1-2
        % J_x_low =[ -0   0  -1.0699   -0.0637   -0.0700; ...
        %            0   -0   0.7228   -1.3273    0.0272; ...
        %            0    0  -1.4105    0.0568   -0.0924; ...
        %            0    0   0.0521   -0.2650    0.0020; ...
        %            0    0   1.0888   -0.0748    0.0406];
        % 
        % J_x_up = [-0   0   -0.7320   -0.0583   -0.0272; ...
        %            0   -0   1.0564   -1.2152    0.0700; ...
        %            0    0  -0.9651    0.0620   -0.0360; ...
        %            0    0   0.0762   -0.2426    0.0051; ...
        %            0    0   1.5914   -0.0685    0.1043];

        % % From CORA
        % J_x_low =[ -0   0  -1.2033   -0.0637   -0.6015; ...
        %            0   -0   0.6554   -1.3268    0.0276; ...
        %            0    0  -1.5863    0.0534   -0.7937; ...
        %            0    0   0.0473   -0.2649    0.0020; ...
        %            0    0   0.9873   -0.0748    0.0412];
        % 
        % J_x_up = [-0   0   -0.6638   -0.0549   -0.0277; ...
        %            0   -0   1.1881   -1.1431    0.6008; ...
        %            0    0  -0.8751    0.0620   -0.0365; ...
        %            0    0   0.0857   -0.2282    0.0442; ...
        %            0    0   1.7898   -0.0645    0.8956];
    
        % No inputs as the system is autonomous
        J_p_low = zeros(5,5);
        J_p_up = zeros(5,5);

    case 15
        %% Spiral_non System (continuous-time)

        % State Jacobian bounds
        
        % % for a big time step t=0-8
        % J_x_low = [-0.8662   -2.0547; ...
        %             0.2127   -0.8719];
        % J_x_up = [0.7601   -0.2285; ...
        %           2.0383    0.7471];

        % % % 1st Approach (initial step) for t=0-2
        % J_x_low = [-0.5706   -2.0547; ...
        %             0.9213   -0.5677];
        % J_x_up = [0.4170   -0.9852; ...
        %           2.0383    0.4154];

        % % For the 1st approach for final RS @t=0-2
        % J_x_low = [-0.2355   -1.8576; ...
        %             1.4438   -0.1881];
        % J_x_up = [0.0191   -1.6097; ...
        %           1.7423    0.0660];
         

        % % 2nd Approach (incremental approach) for t= 0-1 then for t=1-2
        % J_x_low = [-0.5706   -2.0547; ...
        %             0.9213   -0.5677];
        % J_x_up = [0.4170   -0.9852; ...
        %           2.0383    0.4154];

    
        % No inputs as the system is autonomous
        J_p_low = zeros(2,2);
        J_p_up = zeros(2,2);
        
end


