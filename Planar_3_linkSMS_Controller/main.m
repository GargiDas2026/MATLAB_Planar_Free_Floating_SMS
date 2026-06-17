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

% Time span
dt = 0.01;
%code run time:
t = 20;
tspan = (0:dt:t)';

%% Choose which control you want to run
ctrl_pref = 1; % 0 = PID, 1 = SMC

tic
if ctrl_pref ==0
%% To run the PID control
[Y,tau_q,KE,P,M] = rk4t_PID(@SMS_dynamics_PID,tspan,state0);
elseif ctrl_pref == 1
%% To run the sliding mode control
[Y,tau_q,S] = rk4t_SMC(@SMS_dynamics_SMC,tspan,state0);
toc
end 

% Plot Results
figure (1);
plot(tspan,Y(:,1),'r', 'LineWidth', 3); hold on;
plot(tspan,Y(:,2),'y', 'LineWidth', 3);
plot(tspan,Y(:,3),'g','LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('base satellite position (m)');
grid on

figure (2);
plot(tspan,Y(:,4),'r', 'LineWidth', 3); hold on;
plot(tspan,Y(:,5),'y', 'LineWidth', 3);
plot(tspan,Y(:,6),'g','LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('base satellite orientation (rad)');
grid on


figure(3);
plot(tspan,Y(:,7),'r', 'LineWidth', 3); hold on;
plot(tspan,Y(:,8),'y', 'LineWidth', 3); 
plot(tspan,Y(:,9),'g', 'LineWidth', 3); 
hold off;
xlabel('Time (s)');
ylabel('$q$ (rad)');
grid on

figure(4);
plot(tspan,Y(:,10),'r', 'LineWidth', 3); hold on;
plot(tspan,Y(:,11),'y', 'LineWidth', 3); 
plot(tspan,Y(:,12),'g', 'LineWidth', 3); 
hold off;
xlabel('Time (s)');
ylabel('base satellite linear velocity (m/s)');
grid on

figure(5);
plot(tspan,Y(:,13),'r', 'LineWidth', 3); hold on;
plot(tspan,Y(:,14),'y', 'LineWidth', 3);
plot(tspan,Y(:,15),'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$\omega_b$ (rad/s)');
grid on

figure(7);
plot(tspan, Y(:,16), 'LineWidth', 3); hold on;
plot(tspan,Y(:,17),'y', 'LineWidth', 3);
plot(tspan,Y(:,18),'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$\dot{q}$ (rad/s)');
grid on



figure(8)
plot(tspan, tau_q(:,1), 'r', 'LineWidth', 3); hold on;
plot(tspan, tau_q(:,2), 'b', 'LineWidth', 3);
plot(tspan, tau_q(:,3), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Joint Control Torque$ (Nm)');
grid on

if ctrl_pref ==0
% plot Kinetic Energy
figure(9)
plot(tspan, KE, 'r', 'LineWidth', 3)
xlabel('Time (s)');
ylabel('$KE$ (J)');
grid on

% plot Linear momentum
figure(10)
plot(tspan, P(:,1), 'r', 'LineWidth', 3); hold on;
plot(tspan, P(:,2), 'b', 'LineWidth', 3);
plot(tspan, P(:,3), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Linear momentum$ (kg.m/s)');
grid on

% plot Angular momentum
figure(11)
plot(tspan, M(:,1), 'r', 'LineWidth', 3); hold on;
plot(tspan, M(:,2), 'b', 'LineWidth', 3);
plot(tspan, M(:,3), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Angular momentum$ (kg.m^2/s)');
grid on

elseif ctrl_pref == 1  
% plot sliding surface     
figure(9)
plot(tspan, S(:,1), 'r', 'LineWidth', 3); hold on;
plot(tspan, S(:,2), 'b', 'LineWidth', 3);
plot(tspan, S(:,3), 'g', 'LineWidth', 3);
hold off;
xlabel('Time (s)');
ylabel('$Sliding Surface$');
grid on

end
