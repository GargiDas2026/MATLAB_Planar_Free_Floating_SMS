clc;
clear;

import casadi.*

%% Build model

tic
f = SMS_Model_Casadi();
build_time = toc;

fprintf('\nCasADi model build time: %.6f s\n',build_time);

%% ---------------------------------------------------------
% Test state
%% ---------------------------------------------------------

q = [0.2;
    -0.3;
     0.4];

% Use the same initial configuration idea as main.m
N_r_b_0 = Calc_rb(q);

x_test = [ ...
    N_r_b_0;
    zeros(3,1);
    q;
    zeros(9,1)];

%% Test torque

tau_test = [1.0;
            0.5;
            0.2];

%% ---------------------------------------------------------
% Evaluate dynamics
%% ---------------------------------------------------------

tic

xdot_casadi = full(f(x_test,tau_test));

eval_time = toc;

fprintf('CasADi dynamics evaluation: %.9f s\n',eval_time);

%% ---------------------------------------------------------
% Display result
%% ---------------------------------------------------------

disp(' ');
disp('CasADi xdot =');
disp(xdot_casadi);