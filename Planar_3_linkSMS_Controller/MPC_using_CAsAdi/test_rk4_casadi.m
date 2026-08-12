%% =========================================================
% Compare one complete RK4 step
%% =========================================================
param;
h =  0.01;

%% Old MATLAB RK4

tic

x_next_old = rk4t_MPC( ...
    @SMS_dynamics_MPC, ...
    x_test, ...
    tau_test, ...
    h);

t_old_rk4 = toc;


%% CasADi RK4

tic

x_next_casadi = full( ...
    rk4t_Casadi( ...
        f, ...
        x_test, ...
        tau_test, ...
        h));

t_casadi_rk4 = toc;


%% =========================================================
% Comparison
%% =========================================================

error_rk4 = x_next_casadi - x_next_old;

fprintf('\n============================================\n');
fprintf('RK4 COMPARISON\n');
fprintf('============================================\n');

fprintf('Old MATLAB RK4 time    : %.9f s\n',t_old_rk4);
fprintf('CasADi RK4 time        : %.9f s\n',t_casadi_rk4);

fprintf('\nMaximum absolute error : %.12e\n', ...
    max(abs(error_rk4)));

fprintf('2-norm error            : %.12e\n', ...
    norm(error_rk4));