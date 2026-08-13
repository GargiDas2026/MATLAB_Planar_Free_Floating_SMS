function solver = BuildSolver(param)

import casadi.*

%% =========================================================
% Dimensions
%% =========================================================

nx = 18;
nu = 3;

Np = param.Np;
dt = param.dt;

%% =========================================================
% Cost matrices
%% =========================================================

Q = param.Q;
R = param.R;

%% =========================================================
% Decision variables
%
% Single-shooting:
% U = [tau_1 ... tau_Np]
%% =========================================================

U = MX.sym('U',nu,Np);

%% =========================================================
% Parameters supplied online
%
% P = [current_state;
%      reference_state]
%% =========================================================

X0   = MX.sym('X0',nx,1);
XREF = MX.sym('XREF',nx,1);

P = [X0;
     XREF];

%% =========================================================
% Construct CasADi SMS dynamics ONCE
%% =========================================================

f = SMS_Model_Casadi();

%% =========================================================
% Initial predicted state
%% =========================================================

x = X0;

%% =========================================================
% Objective
%% =========================================================

J = 0;

%% =========================================================
% Prediction horizon
%% =========================================================

for k = 1:Np

    % Current predicted control
    tau = U(:,k);

    % Propagate one step
    x = rk4t_Casadi( ...
        f, ...
        x, ...
        tau, ...
        dt);

    % Tracking error
    e = x - XREF;

    % Running cost
    J = J + e'*Q*e + tau'*R*tau;

end

%% =========================================================
% NLP
%% =========================================================

OPT_variables = reshape(U,nu*Np,1);

nlp = struct( ...
    'x', OPT_variables, ...
    'f', J, ...
    'p', P);

%% =========================================================
% IPOPT options - if anyone want to run IPOPT
%% =========================================================

% opts = struct;
% 
% opts.ipopt.print_level = 0;
% opts.print_time = 0;
% 
% opts.ipopt.max_iter = 100;
% opts.ipopt.tol = 1e-4;

% %% =========================================================
% % Create solver
% %% =========================================================
% 
% solver = nlpsol( ...
%     'solver', ...
%     'ipopt', ...
%     nlp, ...
%     opts);

%% =========================================================
% SQP solver options
%% =========================================================

opts = struct;

%% SQP output
opts.print_header    = false;
opts.print_iteration = false;
opts.print_status    = false;
opts.print_time       = false;
opts.max_iter = 50;

%% QP solver
opts.qpsol = 'qrqp';

opts.qpsol_options = struct( ...
    'print_header', false, ...
    'print_iter',   false, ...
    'print_info',   false);
%% =========================================================
% Create SQP solver
%% =========================================================
disp(opts)
disp('Creating SQP solver...')
solver = nlpsol( ...
    'solver', ...
    'sqpmethod', ...
    nlp, ...
    opts);
end