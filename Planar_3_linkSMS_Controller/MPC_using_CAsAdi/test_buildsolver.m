clear
clc
param.Np = 12;%15
param.dt = 0.01;
param.tau_max=[5;2.5;2.5];
param.Q = diag([0 0 0 0 0 0 200 150 150 0 0 0 0 0 0 20 20 20]);
param.R = diag([0.01 0.01 0.01]);
param.xref=zeros(18,1);
param.xref(7)= 0.872665;  % 50 deg % -0.523599 : -30 deg

import casadi.*

%% Build solver
fprintf('Building solver...\n');

tic
solver = BuildSolver(param);
t_build = toc;

fprintf('Solver build time = %.6f s\n',t_build);

%% ---------------------------------------------------------
% Parameters
%% ---------------------------------------------------------

param;

nx = 18;
nu = 3;
Np = param.Np;

%% ---------------------------------------------------------
% Initial state
%% ---------------------------------------------------------

q = [0.2;
    -0.3;
     0.4];

N_r_b_0 = Calc_rb(q);

x0 = [ ...
    N_r_b_0;
    zeros(3,1);
    q;
    zeros(9,1)];

%% ---------------------------------------------------------
% Reference
%% ---------------------------------------------------------

xref = x0;

xref(7) = x0(7) + 0.2;
xref(8) = x0(8) - 0.1;
xref(9) = x0(9) + 0.15;

%% ---------------------------------------------------------
% Initial control guess
%% ---------------------------------------------------------

U0 = zeros(nu*Np,1);

%% ---------------------------------------------------------
% Parameters
%% ---------------------------------------------------------

P = [x0;
     xref];

%% ---------------------------------------------------------
% Bounds
%% ---------------------------------------------------------

tau_max = param.tau_max(:);

LB = repmat(-tau_max,Np,1);
UB = repmat( tau_max,Np,1);

%% ---------------------------------------------------------
% Solve
%% ---------------------------------------------------------

fprintf('\nCalling IPOPT...\n');

tic

sol = solver( ...
    'x0',U0, ...
    'lbx',LB, ...
    'ubx',UB, ...
    'p',P);

t_solve = toc;

fprintf('\n============================================\n');
fprintf('IPOPT TEST\n');
fprintf('============================================\n');

fprintf('Solver build time : %.6f s\n',t_build);
fprintf('IPOPT solve time  : %.6f s\n',t_solve);

%% ---------------------------------------------------------
% Extract solution
%% ---------------------------------------------------------

U_opt = full(sol.x);

tau = U_opt(1:3);

fprintf('\nFirst optimal torque:\n');
disp(tau);

%% ---------------------------------------------------------
% Solver statistics
%% ---------------------------------------------------------

stats = solver.stats();

fprintf('IPOPT iterations   : %d\n', ...
    stats.iter_count);

fprintf('Return status      : %s\n', ...
    stats.return_status);