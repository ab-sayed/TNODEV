% solver_parameters.m
% Configuration file for ODE solver parameters used in NNV reachability analysis
% Adapted from TIRA's solver_parameters.m for consistency in computing final states

% ODE solver configuration
% Select ODE solver with signature: [t, y] = ode_solver(odefun, tspan, y0, options)
% Matches TIRA's default solver (ode45)
parameters.ode_solver = @ode45;

% ODE solver options
% TIRA's default is empty (MATLAB defaults: RelTol = 1e-3, AbsTol = 1e-6)
% Using tighter tolerances and MaxStep matching reachstep = 0.05 for stability
parameters.ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', 0.05);