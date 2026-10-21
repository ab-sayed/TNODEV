%% Initialization
clear

addpath('../OA_methods')
addpath('../Utilities')
addpath('../Examples')
addpath(genpath('/home/meyerpj/Bureau/Research/CORA'))
addpath(genpath('/home/meyerpj/Bureau/Research/nnv'))


% Load nODE layer for TIRA
file_path = 'odecnn_mnist_tiny.mat';
load(file_path); 

w_ode1 = Wb{9};
b_ode1 = Wb{10}';
w_ode2 = Wb{11};
b_ode2 = Wb{12}';

Aout = w_ode2 * w_ode1;          % The 'A' matrix (676x676)
Bout = w_ode2 * b_ode1 + b_ode2; % The 'B' vector (676x1)

ode_params = struct();
ode_params.system_choice = 16;   % Matches MNIST case in NODE_RA_NEW.m & System_description.m
ode_params.t_final = 1;          % Time Horizon |--->for the original `reach_cnn_tiny.m`it is 1 
ode_params.timeStep = 20;         % = t_final/timeStep = 1/20 = 0.05
ode_params.A = Aout;             % Pass A to the layer (essential for NODE_RA_NEW)
ode_params.B = Bout;             % Pass B to the layer

odelayer_NODE_RA = TNODEVODELayer(ode_params);

% Load nODE layer for NNV
states = 676;
outputs = 676;
w1 = Wb{9};
b1 = Wb{10}';
w2 = Wb{11};
b2 = Wb{12}';
Aout = w2*w1;
Bout = w2*b1+b2;
Cout = eye(states);
D = zeros(outputs,1);
tfinal = 1;
numSteps = 20;
odeblock = LinearODE(Aout,Bout,Cout,D,tfinal,numSteps);
reachStep = tfinal/numSteps;
odelayer = ODEblockLayer(odeblock,1,reachStep,false);

% NNV output of layer 7
load('S_ode_in_first_iter.mat')

%% TIRA reachability of layer 8 from Abdelrahman's code
tic
L8_output_TIRA_AR = odelayer_NODE_RA.reach(S_ode_in);
toc_TIRA_AR = toc

%% TIRA reachability of layer 8 from direct TIRA call
tic_tira = tic;
fprintf('\nConvert star to interval\n')
tic
% [lb, ub] = S_ode_in.getRanges();
a = S_ode_in.getBox();
lb = a.lb;
ub = a.ub;
toc

% fprintf('Reshape interval\n')
% tic
% lb = reshape(lb, [], 1);
% ub = reshape(ub, [], 1);
% toc

fprintf('Get Jacobian bounds\n')
tic
[J_x_low,J_x_up,J_p_low,J_p_up] = UP_Jacobian_Bounds(0,1,lb, ub, zeros(length(lb),1), zeros(length(lb),1));
toc

fprintf('Call CTMM\n')
tic
[succ_low,succ_up] = OA_3_CT_Mixed_Monotonicity(0,1,lb, ub, zeros(length(lb),1), zeros(length(lb),1),J_x_low,J_x_up,J_p_low,J_p_up);
toc

% fprintf('Call TIRA\n')
% tic
% [succ_low, succ_up] = TIRA([0, 1], lb, ub, zeros(length(lb),1), zeros(length(lb),1), 3);
% toc

fprintf('Convert interval to star\n')
tic
L8_output_TIRA_PJ = Star(succ_low, succ_up);
toc
toc_TIRA_PJ = toc(tic_tira)

%% NNV reachability of layer 8
tic
L8_output_NNV = odelayer_NODE_RA.reach(S_ode_in);
toc_NNV = toc

%% Star/interval conversions
fprintf('\nConvert star to interval\n')
tic
[lb, ub] = S_ode_in.getRanges();
toc
fprintf('Convert interval to star\n')
tic
L8_output_TIRA_PJ = Star(lb, ub);
toc
fprintf('Convert star to interval\n')
tic
[lb2, ub2] = L8_output_TIRA_PJ.getRanges();
toc

isequal(lb,lb2)
isequal(ub,ub2)

%% Box conversion comparisons
fprintf('\nGetRanges\n')
tic
[lb, ub] = S_ode_in.getRanges();
toc
fprintf('GetBox\n')
tic
a = S_ode_in.getBox();
lb2 = a.lb;
ub2 = a.ub;
toc
fprintf('GetBoxFast\n')
tic
b = S_ode_in.getBoxFast();
lb3 = b.lb;
ub3 = b.ub;
toc

isequal(lb,lb2)
isequal(lb,lb3)
isequal(ub,ub2)
isequal(ub,ub3)
