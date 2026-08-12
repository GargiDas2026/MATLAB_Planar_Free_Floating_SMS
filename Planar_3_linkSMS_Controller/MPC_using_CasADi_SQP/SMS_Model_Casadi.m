function f = SMS_Model_Casadi()

import casadi.*

%% =========================================================
%  SMS DYNAMICS USING CASADI + LAGRANGIAN FORMULATION
%
%  State:
%       x = [Phi;
%            dPhi]
%
%  Phi = [x y z roll pitch yaw q1 q2 q3]
%
%  Input:
%       tau_q = [tau1; tau2; tau3]
%
%  Model based on:
%       three_arm_SMS_Symbolic.jl
%
%  The large symbolic H and C matrices are NOT used.
%  CasADi generates H and C through automatic differentiation.
%% =========================================================

%% ---------------------------------------------------------
% Load physical parameters
%% ---------------------------------------------------------

param;
%% =========================================================
% Inertia parameters
%% =========================================================

% Base inertia
I_bx = (1/12)*m_b*(wb^2 + hb^2);
I_by = (1/12)*m_b*(lb^2 + hb^2);
I_bz = (1/12)*m_b*(lb^2 + wb^2);

% Link 1 inertia
I_1x = (1/12)*m_1*(w1^2 + h1^2);
I_1y = (1/12)*m_1*(l_1^2 + h1^2);
I_1z = (1/12)*m_1*(l_1^2 + w1^2);

% Link 2 inertia
I_2x = (1/12)*m_2*(w2^2 + h2^2);
I_2y = (1/12)*m_2*(l_2^2 + h2^2);
I_2z = (1/12)*m_2*(l_2^2 + w2^2);

% Link 3 inertia
I_3x = (1/12)*m_3*(w3^2 + h3^2);
I_3y = (1/12)*m_3*(l_3^2 + h3^2);
I_3z = (1/12)*m_3*(l_3^2 + w3^2);
%% ---------------------------------------------------------
% Symbolic state and input
%% ---------------------------------------------------------
Phi  = MX.sym('Phi',9,1);
dPhi = MX.sym('dPhi',9,1);
x = [Phi; dPhi];

tau_q = MX.sym('tau_q',3,1);

%% ---------------------------------------------------------
% Generalized coordinates
%% ---------------------------------------------------------

t1 = Phi(1);
t2 = Phi(2);
t3 = Phi(3);
t4 = Phi(4);
t5 = Phi(5);
t6 = Phi(6);

q1 = Phi(7);
q2 = Phi(8);
q3 = Phi(9);

%% ---------------------------------------------------------
% Generalized velocities
%% ---------------------------------------------------------

t1dot = dPhi(1);
t2dot = dPhi(2);
t3dot = dPhi(3);
t4dot = dPhi(4);
t5dot = dPhi(5);
t6dot = dPhi(6);

q1dot = dPhi(7);
q2dot = dPhi(8);
q3dot = dPhi(9);

%% ---------------------------------------------------------
% Rotation matrix about z-axis
%% ---------------------------------------------------------

Rz = @(a) [ cos(a), -sin(a), 0;
            sin(a),  cos(a), 0;
                 0,       0, 1 ];

%% ---------------------------------------------------------
% Frame transformations
%% ---------------------------------------------------------

R_NB = Rz(t6);

R_B1 = Rz(q1);
R_B2 = R_B1 * Rz(q2);
R_B3 = R_B2 * Rz(q3);

R_N1 = R_NB * R_B1;
R_N2 = R_NB * R_B2;
R_N3 = R_NB * R_B3;

%% ---------------------------------------------------------
% Position vectors
%% ---------------------------------------------------------

r_b = [t1;
       t2;
       t3];

B_L_b = [lb/2;
         0;
         0];

N_L_b = R_NB * B_L_b;

% Link 1 COM
r_1 = r_b ...
    + N_L_b ...
    + 0.5 * R_N1 * [l_1;0;0];

% Link 2 COM
r_2 = r_b ...
    + N_L_b ...
    + R_N1 * [l_1;0;0] ...
    + 0.5 * R_N2 * [l_2;0;0];

% Link 3 COM
r_3 = r_b ...
    + N_L_b ...
    + R_N1 * [l_1;0;0] ...
    + R_N2 * [l_2;0;0] ...
    + 0.5 * R_N3 * [l_3;0;0];

%% ---------------------------------------------------------
% Translational velocities
%
% Julia computes time derivatives of r_i.
%
% Since r_i = r_i(Phi),
%
%       dr_i/dt = (dr_i/dPhi) * dPhi
%
% CasADi computes these Jacobians automatically.
%% ---------------------------------------------------------

v_b = jacobian(r_b,Phi) * dPhi;
v_1 = jacobian(r_1,Phi) * dPhi;
v_2 = jacobian(r_2,Phi) * dPhi;
v_3 = jacobian(r_3,Phi) * dPhi;

%% ---------------------------------------------------------
% Angular velocities
%% ---------------------------------------------------------

k = [0;0;1];

omega_b = dPhi(4:6);

omega_1 = omega_b ...
        + R_N1 * k * q1dot;

omega_2 = omega_b ...
        + R_N1 * k * q1dot ...
        + R_N2 * k * q2dot;

omega_3 = omega_b ...
        + R_N1 * k * q1dot ...
        + R_N2 * k * q2dot ...
        + R_N3 * k * q3dot;

%% ---------------------------------------------------------
% Mass moments of inertia
%% ---------------------------------------------------------

I_b = diag([I_bx, I_by, I_bz]);

I_1 = diag([I_1x, I_1y, I_1z]);
I_2 = diag([I_2x, I_2y, I_2z]);
I_3 = diag([I_3x, I_3y, I_3z]);

%% ---------------------------------------------------------
% Inertia tensors expressed in inertial frame
%% ---------------------------------------------------------

I_b_N = R_NB * I_b * R_NB';

I_1_N = R_N1 * I_1 * R_N1';

I_2_N = R_N2 * I_2 * R_N2';

I_3_N = R_N3 * I_3 * R_N3';

%% ---------------------------------------------------------
% Kinetic energy
%
% T = translational KE + rotational KE
%% ---------------------------------------------------------

T = 0.5 * ( ...
      m_b * (v_b' * v_b) ...
    + m_1 * (v_1' * v_1) ...
    + m_2 * (v_2' * v_2) ...
    + m_3 * (v_3' * v_3) ...
    + omega_b' * I_b_N * omega_b ...
    + omega_1' * I_1_N * omega_1 ...
    + omega_2' * I_2_N * omega_2 ...
    + omega_3' * I_3_N * omega_3 );

%% ---------------------------------------------------------
% Euler-Lagrange terms
%
% p = dT/ddPhi
%
% d/dt(p) =
%       d(p)/dPhi * dPhi
%     + d(p)/ddPhi * ddPhi
%
% Therefore:
%
% H = d(p)/d(ddPhi)
%
% C = d(p)/dPhi*dPhi - dT/dPhi
%% ---------------------------------------------------------

p = gradient(T,dPhi);

H = jacobian(p,dPhi);

C = jacobian(p,Phi) * dPhi ...
    - gradient(T,Phi);

%% ---------------------------------------------------------
% Generalized force vector
%
% Free-floating base:
% first six generalized forces are zero.
%
% Only joint torques are applied.
%% ---------------------------------------------------------

tau = [0;
       0;
       0;
       0;
       0;
       0;
       tau_q];

%% ---------------------------------------------------------
% Equations of motion
%
%       H*ddPhi + C = tau
%
%       ddPhi = H\(tau-C)
%% ---------------------------------------------------------

ddPhi = solve(H,tau-C);

%% ---------------------------------------------------------
% State derivative
%% ---------------------------------------------------------

xdot = [dPhi;
        ddPhi];

%% ---------------------------------------------------------
% CasADi function
%% ---------------------------------------------------------

f = Function( ...
    'SMS_Model_Casadi', ...
    {x,tau_q}, ...
    {xdot}, ...
    {'x','tau_q'}, ...
    {'xdot'} );

end