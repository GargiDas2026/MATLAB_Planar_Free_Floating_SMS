function[N_r_b_0] = Calc_rb(q)
%%%%%%%%%% Frame Tranformation matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param
% Quaternion for rotation about z axis
Q = @(q) [0*sin(q/2); 0*sin(q/2); 1*sin(q/2);cos(q/2)];
% Transformation matrices
R_from_Q = @(Q) [1 - 2*(Q(2)^2 + Q(3)^2), 2*(Q(1)*Q(2) - Q(3)*Q(4)), 2*(Q(1)*Q(3) + Q(2)*Q(4));
    2*(Q(1)*Q(2) + Q(3)*Q(4)), 1 - 2*(Q(1)^2 + Q(3)^2), 2*(Q(2)*Q(3) - Q(1)*Q(4));
    2*(Q(1)*Q(3) - Q(2)*Q(4)), 2*(Q(2)*Q(3) + Q(1)*Q(4)), 1 - 2*(Q(1)^2 + Q(2)^2)];

r_X = @(r) [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];

% Quaternion transformation from base to inertial frame (since only yaw motion is there thus only include state 6)
QNB = Q(0); %Q(state(6))
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
N_L_b = eye(3,3) * [lb/2;0;0];
N_l_1 = R_N1 * L_l_1;
N_l_2 = R_N2 * L_l_2;
N_l_3 = R_N3 * L_l_3;
N_r_b_0 =-(((m_1+m_2+m_3) * N_L_b)+ ((0.5 *m_1 + m_2 +m_3)* N_l_1) + ...
          ((0.5 *m_2 +m_3)* N_l_2) + (0.5* m_3 * N_l_3))/(m_b + m_1 + m_2 + m_3);