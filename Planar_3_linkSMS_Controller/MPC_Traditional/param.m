% Define base satellite parameters

k = [0;0;1]; % unit vector of joint 1 (rotation axis of joint 1) 

% parameter setting for SMS (Air bearing system)
m_b = 33.015;% Base mass (kg)
m_1 = 1.2;% Link1 mass (kg)
m_2 = 1.2;% Link2 mass (kg)
m_3 = 0.5;% Link3 mass (kg)


lb = 0.464; % length of the base (m)
wb = 0.464; % width of the base (m)
hb = 0.483; % height of the base (m)

l_1 = 0.38; % length of the link 1 (m)
l_2 = 0.38; % length of the link 2 (m)
l_3 = 0.38;  % length of the link 3 (m)

L_l_1 = [l_1; 0; 0];% vector from joint 1  to joint 2 (m)
L_l_2 = [l_2; 0; 0];% vector from joint 2  to joint 3 (m)
L_l_3 = [l_3; 0; 0];% vector from joint 3  to end effector (m)

w1 = 0.1*l_1; % width of the link 1 (m)
h1 = 0.1*l_1; % height of the link 1 (m)


w2 = 0.1*l_2; % width of the link 2 (m)
h2 = 0.1*l_2; % height of the link 2 (m)

w3 = 0.1*l_3; % width of the link 3 (m)
h3 = 0.1*l_3; % height of the link 3 (m)

% Define controller parameters
%% PID Controller gain
K_p = diag([1 1 1]);
K_d = diag([0.5 0.5 0.5]); 

%% SMC Controller gain
lambda = 5 * eye(3,3);
Ks = 2 * [1;1;1]; %2 * diag([1; 1; 1]);
phi_s = [0.05; 0.05; 0.05];