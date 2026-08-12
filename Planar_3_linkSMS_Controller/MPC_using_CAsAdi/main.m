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


param.Np = 12;%15
param.dt = 0.01;
param.tau_max=[5;2.5;2.5];
param.Q = diag([0 0 0 0 0 0 200 150 150 0 0 0 0 0 0 20 20 20]);
param.R = diag([0.01 0.01 0.01]);
param.xref=zeros(18,1);
param.xref(7)= 0.872665;  % 50 deg % -0.523599 : -30 deg

Tf=12;
tspan = (0:param.dt:Tf)';
N=round(Tf/param.dt);
X=zeros(18,N+1); X(:,1)=state0;
Tau=zeros(3,N);
%% Choose which control you want to run
tic
%% To run the PID control
for k=1:N
    tau = MPC_Controller_Casadi(X(:,k),param);
    Tau(:,k)=tau;
    X(:,k+1)=rk4t_MPC(@SMS_dynamics_MPC,X(:,k),tau,param.dt);

    % f = SMS_Model_Casadi();
    % 
    % X(:,k+1) = rk4t_Casadi(f,X(:,k),tau,param.dt);

    disp(k);
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

% figure(4);
% plot(tspan,Y(:,10),'r', 'LineWidth', 3); hold on;
% plot(tspan,Y(:,11),'y', 'LineWidth', 3); 
% plot(tspan,Y(:,12),'g', 'LineWidth', 3); 
% hold off;
% xlabel('Time (s)');
% ylabel('base satellite linear velocity (m/s)');
% grid on
% 
% figure(5);
% plot(tspan,Y(:,13),'r', 'LineWidth', 3); hold on;
% plot(tspan,Y(:,14),'y', 'LineWidth', 3);
% plot(tspan,Y(:,15),'g', 'LineWidth', 3);
% hold off;
% xlabel('Time (s)');
% ylabel('$\omega_b$ (rad/s)');
% grid on
% 
% figure(7);
% plot(tspan, Y(:,16), 'LineWidth', 3); hold on;
% plot(tspan,Y(:,17),'y', 'LineWidth', 3);
% plot(tspan,Y(:,18),'g', 'LineWidth', 3);
% hold off;
% xlabel('Time (s)');
% ylabel('$\dot{q}$ (rad/s)');
% grid on
% 
% 
% 
figure(8)
plot(tspan(1:end-1), Tau(1,:), 'r', 'LineWidth', 3); hold on;
plot(tspan(1:end-1), Tau(2,:), 'b', 'LineWidth', 3);
plot(tspan(1:end-1), Tau(3,:), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Joint Control Torque$ (Nm)');
grid on


