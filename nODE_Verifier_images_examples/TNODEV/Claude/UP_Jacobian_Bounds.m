% Tool Paper:
%   "Add Title here"
%
% Authors:  
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 21 October 2025
% Last update: 17 March 2026
% Last revision: October 2025

%--> handles BOTH situations: it checks for globals first (for VERIFIER_FINAL.m), and if it doesn't find them, it runs CORA dynamically (for reach_acc_TNODEV.m)

function [J_x_low,J_x_up,J_p_low,J_p_up] = UP_Jacobian_Bounds(t_init,t_final,x_low,x_up,p_low,p_up)
% UP_Jacobian_Bounds.m
% Retrieves the pre-computed Jacobian bounds for the current system.
% The actual Reachable Tube (RT) bounds are now computed via CORA 
% in the main verification script and passed here via global variables.

n_x = length(x_low);
n_p = length(p_low);

%% Default values as NaN (not a number)
J_x_low = NaN(n_x);
J_x_up = NaN(n_x);
J_p_low = NaN(n_x,n_p);
J_p_up = NaN(n_x,n_p);

%% Globals
global system_choice
global Aout_global % used only for MNIST Neural ODE
global Jacobian_x_low Jacobian_x_up
global u % kept for compatibility


%% Infer system_choice if lost in parallel workers
if isempty(system_choice)
    if n_x == 8, system_choice = 17;
    elseif n_x == 5, system_choice = 14;
    elseif n_x == 3, system_choice = 18;
    elseif n_x == 2, system_choice = 15;
    elseif n_x == 676, system_choice = 16;
    end
end

%% 0) Special case: MNIST Neural ODE (system_choice = 16)
% dx/dt = Aout_global * x + Bout_global
if system_choice == 16
    if isempty(Aout_global)
        error('UP_Jacobian_Bounds: Aout_global not set for system_choice = 16.');
    end

    % Jacobian wrt x is Aout
    % Aout_global is 676×676, and n_x = 676 for MNIST features
    J_x_low = Aout_global;
    J_x_up = Aout_global;
    
    % ODE is autonomous in our parametrization "i.e., no real inputs" => Jacobian wrt p is 0 (df/dp = 0)
    J_p_low = zeros(n_x, n_p);
    J_p_up = zeros(n_x, n_p);
    
    return;
end

%% 1) Try to read pre-computed Jacobians from globals 
% (This triggers ONLY when running VERIFIER_FINAL.m)
if ~isempty(Jacobian_x_low) && ~isempty(Jacobian_x_up) && ...
    all(size(Jacobian_x_low) == [n_x, n_x]) && ...
    all(size(Jacobian_x_up) == [n_x, n_x])

    J_x_low = Jacobian_x_low;
    J_x_up = Jacobian_x_up;
    
    % autonomous → ∂f/∂p = 0
    J_p_low = zeros(n_x, n_p);
    J_p_up = zeros(n_x, n_p);
    return;
end

%% 2) Dynamic CORA Computation
% (This triggers ONLY in parfor loops / N-step reachability where globals are empty)
if system_choice == 17
    % Construct CORA params dynamically based on TIRA's current bounds
    cora_params.timeStep      = 0.05; % the controller was 0.1 sec
    cora_params.tFinal        = t_final - t_init;
    cora_params.taylorTerms   = 3;
    cora_params.zonotopeOrder = 10;
    cora_params.alg           = 'lin';
    cora_params.tensorOrder   = 2;
    cora_params.x0            = (x_up + x_low) / 2;
    cora_params.Initial_radius = (x_up - x_low) / 2;
    
    % Get Reachable Tube from CORA
    % rt_bounds = acc_CORA_RT(cora_params);

    % Get Reachable Tube from Unified CORA function
    rt_bounds = compute_CORA_Reachable_Tube(cora_params, system_choice);
    
    % Compute Jacobian over that Tube
    [J_x_low, J_x_up] = Jacob_bounds_element_wise(rt_bounds.lb, rt_bounds.ub, 17);
    
    J_p_low = zeros(n_x, n_p);
    J_p_up = zeros(n_x, n_p);
    return;
end
    %% 3) Error Handling (If missing for other systems)
    % If we reach this point, it means TIRA was called but the global 
    % Jacobian matrices were not pre-calculated and pushed to the workspace.
    % error(['UP_Jacobian_Bounds: Pre-computed Jacobian bounds missing for system %d. ', ...
    %        'Ensure Jacob_bounds_element_wise is called over the CORA RT in VERIFIER_FINAL.m ', ...
    %        'and pushed to global Jacobian_x_low/up.'], system_choice);
    error('UP_Jacobian_Bounds: No global bounds found and dynamic RT not defined for system %d.', system_choice);
end




%------------- END OF CODE V1.9 -------------- |
