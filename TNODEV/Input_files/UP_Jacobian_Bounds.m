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
% Date: 2nd of December 2021

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



