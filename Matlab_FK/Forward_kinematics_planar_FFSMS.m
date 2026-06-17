function [N_r_e] = Forward_kinematics_planar_FFSMS(q)
% This is MATLAB code for solving forward-kinematics
% of a planar 3 DOF free-floating SMS.
%% First define the system parameters
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Frame Tranformation matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quaternion for rotation about z axis
Q = @(q) [0*sin(q/2); 0*sin(q/2); 1*sin(q/2);cos(q/2)];
% Transformation matrices
R_from_Q = @(Q) [1 - 2*(Q(2)^2 + Q(3)^2), 2*(Q(1)*Q(2) - Q(3)*Q(4)), 2*(Q(1)*Q(3) + Q(2)*Q(4));
    2*(Q(1)*Q(2) + Q(3)*Q(4)), 1 - 2*(Q(1)^2 + Q(3)^2), 2*(Q(2)*Q(3) - Q(1)*Q(4));
    2*(Q(1)*Q(3) - Q(2)*Q(4)), 2*(Q(2)*Q(3) + Q(1)*Q(4)), 1 - 2*(Q(1)^2 + Q(2)^2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Calculate initial base position and attitude %%%%%%%%%%%
% The initial base attitude
yaw = 0 * pi/180; % since its planar motion only yaw angle might vary
baseAttitude = [0; 0; yaw]; 

% We consider that the inertial frame conincide with the whole
% systems center of mass and therefore we need to calculate the base
% initial position with respect to the inertial frame (whole systems center
% of mass) as r_cb = r_b - r_c, where r_b base c.m and r_c whole systems
% c.m
% Quaternion transformation from base to inertial frame (since only yaw motion is there thus only include state 6)
QNB = Q(yaw); %base yaw motion 
R_NB = R_from_Q(QNB);
% Trasformation from link 1 to base
QB1 = Q(q(1));
R_B1 = R_from_Q(QB1);
% Trasformation from link 2 to link 1
QB2 = Q(q(2));
R_12 = R_from_Q(QB2);
% Trasformation from link 2 to base
R_B2 = R_B1 * R_12;
% Trasformation from link 3 to link 2
QB3 = Q(q(3));
R_23 = R_from_Q(QB3);
% Trasformation from link 3 to base
R_B3 = R_B2 * R_23;


% Transformation from link 1 to inertial frame
R_N1 = R_NB * R_B1;
% Transformation from link 2 to inertial frame
R_N2 = R_NB * R_B2;
% Transformation from link 3 to inertial frame
R_N3 = R_NB * R_B3;
% Calculation of B_m_b (vector of satellite base CM pointing to the robotic
% arm mounting point)
B_L_b = [lb/2; 0; 0]; % B_M_b in body frame
N_L_b = R_NB * B_L_b;  % N_M_b in inertial frame
N_l_1 = R_N1 * L_l_1;
N_l_2 = R_N2 * L_l_2;
N_l_3 = R_N3 * L_l_3;
% Calculate the base position w.r.t inertial frame
N_r_b =-(((m_1+m_2+m_3) * N_L_b)+ ((0.5 *m_1 + m_2 +m_3)* N_l_1) + ...
          ((0.5 *m_2 +m_3)* N_l_2) + (0.5* m_3 * N_l_3))/(m_b + m_1 + m_2 + m_3);


% Calcualte end-effector position
N_r_e = N_r_b + N_L_b + R_N1 * L_l_1 + R_N2 * L_l_2 + R_N3 * L_l_3; 


end