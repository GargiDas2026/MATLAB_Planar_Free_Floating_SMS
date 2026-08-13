clc; clear; close all;
set(groot,'defaultFigureWindowStyle','normal')
%% q1 variation
q0 = [0; 0; 0]; % with j1 = 0; j2 = 0; j3 =0;
N_r_b_0 = Calc_rb(q0);
state_0 = [N_r_b_0;zeros(3,1); q0]; 

%% Initial velocity states
dstate_0 = [zeros(6,1);0; 0; 0]; 

% Initial state
state0 = [state_0; dstate_0];


param.Np = 12;
param.dt = 0.01;
param.tau_max=[5;2.5;2.5];
param.Q = diag([0 0 0 0 0 0 200 150 150 0 0 0 0 0 0 20 20 20]);
param.R = diag([0.01 0.01 0.01]);
param.xref=zeros(18,1);
param.xref(7)= 0.872665;  % 50 deg %% -0.523599 : -30 deg

Tf=10;
tspan = (0:param.dt:Tf)';
N=round(Tf/param.dt);
X=zeros(18,N+1); X(:,1)=state0;
Tau=zeros(3,N);
%% Choose which control you want to run
f_sms = SMS_Model_Casadi();
tic
for k = 1:N

    tic

    tau = MPC_Controller_Casadi(X(:,k),param);

    t_mpc(k) = toc;

    Tau(:,k) = tau;

    X(:,k+1) = full( ...
        rk4t_Casadi( ...
            f_sms, ...
            X(:,k), ...
            tau, ...
            param.dt));

    fprintf('Step %d: MPC = %.4f s\n', ...
        k,t_mpc(k));

end
toc

% %%  Plot Results
figure (1);
plot(tspan,X(1,:),'r', 'LineWidth', 3); hold on;
plot(tspan,X(2,:),'y', 'LineWidth', 3);
plot(tspan,X(3,:),'g','LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('base satellite position (m)');
grid on

figure (2);
plot(tspan,X(4,:),'r', 'LineWidth', 3); hold on;
plot(tspan,X(5,:),'y', 'LineWidth', 3);
plot(tspan,X(6,:),'g','LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('base satellite orientation (rad)');
grid on


figure(3);
plot(tspan,X(7,:),'r', 'LineWidth', 3); hold on;
plot(tspan,X(8,:),'y', 'LineWidth', 3); 
plot(tspan,X(9,:),'g', 'LineWidth', 3); 
hold off;
xlabel('Time (s)');
ylabel('$q$ (rad)');
grid on


figure(8)
plot(tspan(1:end-1), Tau(1,:), 'r', 'LineWidth', 3); hold on;
plot(tspan(1:end-1), Tau(2,:), 'b', 'LineWidth', 3);
plot(tspan(1:end-1), Tau(3,:), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Joint Control Torque$ (Nm)');
grid on


