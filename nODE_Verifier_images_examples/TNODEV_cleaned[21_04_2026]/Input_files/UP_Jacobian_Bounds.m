% =========================================================================
% File: UP_Jacobian_Bounds.m
%
% Purpose:
%   Retrieves the Jacobian bounds [J_x_low, J_x_up] (and the trivial
%   input-Jacobian bounds [J_p_low, J_p_up]) for the current system,
%   using one of three sources depending on the runtime context.
%
% Description:
%   This file is the user-provided hook that TIRA calls (see
%   ./Input_files/UP_Jacobian_Bounds.m in the original TIRA layout)
%   to obtain the Jacobian bounds for the over-approximation methods
%   that need them.
%
%   In the TNODEV pipeline, the actual Reachable-Tube (RT) bounds are
%   computed in advance via CORA in the main verification script and
%   passed here through the global variables Jacobian_x_low / Jacobian_x_up.
%
%   The function tries (in order):
%     0. Special-cased systems (MNIST, Copepod, Cartpole, Linear ACC)
%        that compute their own state-Jacobian via Jacob_bounds_element_wise.
%     1. Read pre-computed Jacobians from the host-side globals
%        (typical path when running through Verifier_FINAL.m / Verifier_MAIN_CALL.m).
%     2. Dynamic CORA computation (used inside parfor workers / N-step
%        reachability where the host-side globals are empty), currently
%        implemented for the Nonlinear ACC (system 17).
%     3. Otherwise, throw a descriptive error.
%
% Inputs:
%   t_init, t_final : reachability time range
%   x_low, x_up     : state-interval bounds at t_init
%   p_low, p_up     : input-interval bounds (kept for TIRA compatibility)
%
% Outputs:
%   [J_x_low, J_x_up] : interval bounds of the state Jacobian
%   [J_p_low, J_p_up] : interval bounds of the input Jacobian
%                       (zeros for autonomous systems)
%
% Part of the TNODEV Toolbox  (TIRA framework hook)
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
% =========================================================================

%------------- BEGIN CODE --------------
function [J_x_low, J_x_up, J_p_low, J_p_up] = UP_Jacobian_Bounds(t_init, t_final, x_low, x_up, p_low, p_up)

    n_x = length(x_low);
    n_p = length(p_low);


    %% Default values as NaN (not a number)
    J_x_low = NaN(n_x);
    J_x_up  = NaN(n_x);
    J_p_low = NaN(n_x, n_p);
    J_p_up  = NaN(n_x, n_p);


    %% Globals
    global system_choice
    global Aout_global              % used only for MNIST Neural ODE
    global Jacobian_x_low Jacobian_x_up
    global u %#ok<NUSED>            % kept for compatibility


    %% Infer system_choice if lost in parallel workers
    if isempty(system_choice)
        if     n_x == 8,   system_choice = 17;
        elseif n_x == 5,   system_choice = 14;
        elseif n_x == 3,   system_choice = 18;
        elseif n_x == 2,   system_choice = 15;
        elseif n_x == 676, system_choice = 16;
        % elseif n_x == 32,  system_choice = 19; % 1st nODE Network with ReLU AF, IMG_SIZE = 8
        % elseif n_x == 256, system_choice = 19; % 1st nODE Network with ReLU AF, IMG_SIZE = 16
        elseif n_x == 32,  system_choice = 20;   % 2nd nODE Network with Tanh AF, IMG_SIZE = 8
        % elseif n_x == 256, system_choice = 20; % 2nd nODE Network with Tanh AF, IMG_SIZE = 16
        elseif n_x == 12,  system_choice = 22;   % Cartpole
        end
    end


    %% 0a) Special case: MNIST Neural ODE (system_choice = 16)
    % Linear ODE:  dx/dt = Aout_global * x + Bout_global  =>  J_x = Aout_global (constant)
    if system_choice == 16
        if isempty(Aout_global)
            error('UP_Jacobian_Bounds: Aout_global not set for system_choice = 16.');
        end

        % Aout_global is 676 x 676, n_x = 676 for MNIST features
        J_x_low = Aout_global;
        J_x_up  = Aout_global;

        % Autonomous parametrisation -> df/dp = 0
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 0b) Copepod Neural ODE (system_choice = 19) with ReLU activation
    if system_choice == 19
        [J_x_low, J_x_up] = Jacob_bounds_element_wise(x_low, x_up, 19);
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 0c) Copepod Neural ODE (system_choice = 20) with tanh activation
    if system_choice == 20
        [J_x_low, J_x_up] = Jacob_bounds_element_wise(x_low, x_up, 20);
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 0d) Cartpole 12D with CT-RNN + physics (system_choice = 22)
    if system_choice == 22 && isvector(x_low) && length(x_low) == n_x
        [J_x_low, J_x_up] = Jacob_bounds_element_wise(x_low, x_up, 22);
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 0e) Linear ACC NODE (system_choice = 24) -- constant Jacobian
    % J = W2 * W1 regardless of state, but we still compute it per-call so
    % the parfor workers inside reach_acc_TNODEV don't depend on host-side
    % globals.
    if system_choice == 24 && isvector(x_low) && length(x_low) == n_x
        [J_x_low, J_x_up] = Jacob_bounds_element_wise(x_low, x_up, 24);
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 1) Try to read pre-computed Jacobians from globals
    % (This path triggers ONLY when running through Verifier_FINAL.m /
    %  Verifier_MAIN_CALL.m)
    if ~isempty(Jacobian_x_low) && ~isempty(Jacobian_x_up) && ...
            all(size(Jacobian_x_low) == [n_x, n_x]) && ...
            all(size(Jacobian_x_up)  == [n_x, n_x])

        J_x_low = Jacobian_x_low;
        J_x_up  = Jacobian_x_up;

        % Autonomous -> df/dp = 0
        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 2) Dynamic CORA Computation
    % (This path triggers ONLY in parfor loops / N-step reachability where
    %  the host-side globals are empty)
    if system_choice == 17
        % Construct CORA params dynamically from TIRA's current bounds
        cora_params.timeStep       = 0.01;            % controller period was 0.1 s     (was 0.05)
        cora_params.tFinal         = t_final - t_init;
        cora_params.taylorTerms    = 4;               % was 3
        cora_params.zonotopeOrder  = 50;              % was 10
        cora_params.alg            = 'lin';
        cora_params.tensorOrder    = 2;
        cora_params.x0             = (x_up + x_low) / 2;
        cora_params.Initial_radius = (x_up - x_low) / 2;

        % Reachable Tube via the unified CORA helper
        rt_bounds = compute_CORA_Reachable_Tube(cora_params, system_choice);

        % Compute Jacobian bounds over that tube
        [J_x_low, J_x_up] = Jacob_bounds_element_wise(rt_bounds.lb, rt_bounds.ub, 17);

        J_p_low = zeros(n_x, n_p);
        J_p_up  = zeros(n_x, n_p);
        return;
    end


    %% 3) Error handling (no source of Jacobian bounds available)
    error(['UP_Jacobian_Bounds: No global bounds found and dynamic RT not ' ...
           'defined for system %d.'], system_choice);

end
%------------- END OF CODE --------------
