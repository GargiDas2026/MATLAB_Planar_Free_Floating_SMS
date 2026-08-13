<p align="center">
  <img src="Docs/Figure/SMS_Diagram.png"
       alt="Planar free-floating space manipulator system"
       width="700">
</p>
# Planar Free-Floating Space Manipulator: Modeling and Control

This repository contains the modeling, simulation, and control implementation of a planar free-floating Space Manipulator System (SMS) with a three-link robotic manipulator mounted on a freely floating spacecraft base.

The project is developed progressively, beginning with forward-kinematics verification and dynamic modeling, followed by the development of PID, Sliding Mode Control (SMC), and nonlinear Model Predictive Control (MPC).
Two MPC implementations are provided: a conventional MATLAB implementation using `fmincon` and a CasADi-based implementation using symbolic dynamics, automatic differentiation, single-shooting prediction, and Sequential Quadratic Programming (SQP).

The current validated CasADi implementation uses SQP with an exact Hessian and QRQP as the underlying quadratic-programming solver.

---

## Project Objectives

The main objectives of this project are:

1. Develop and validate the forward kinematics of a planar free-floating space manipulator.
2. Formulate the equations of motion of the coupled spacecraft-manipulator system.
3. Analyze the dynamic coupling between manipulator motion and spacecraft-base motion.
4. Implement classical control approaches for the manipulator.
5. Develop a nonlinear Model Predictive Controller using conventional MATLAB optimization.
6. Develop a CasADi-based MPC implementation using symbolic dynamics and automatic differentiation.
7. Compare the conventional and CasADi-based MPC implementations in terms of computational performance and control behavior.

---

# System Description

The system considered in this project consists of:

- A freely floating spacecraft base.
- A three-link planar robotic manipulator.
- Revolute joints connecting the manipulator links.
- No external force or torque acting on the free-floating system.
- Dynamic coupling between the spacecraft base and manipulator motion.

The generalized coordinates describe the spacecraft base motion together with the manipulator joint configuration.

For the general SMS formulation, the generalized coordinates are represented by the spacecraft translational and rotational states together with the manipulator joint coordinates.

---

# Mathematical Formulation

The complete derivation of the free-floating space manipulator equations of motion is available here.

📄 **(docs/Derivation.pdf)**

## Development Stages

The repository follows a progressive controller-development workflow:

### Stage 1 — Forward Kinematics

Verification of the planar free-floating SMS geometry and end-effector
kinematics.

### Stage 2 — Dynamic Modeling

Derivation and implementation of the coupled spacecraft-manipulator dynamics,
including the dynamic coupling between manipulator motion and spacecraft-base
motion.

### Stage 3 — PID Control

Development of a classical PID controller as an initial control baseline.

### Stage 4 — Sliding Mode Control

Development of a nonlinear Sliding Mode Controller for the SMS.

### Stage 5 — Traditional MPC

Development of a nonlinear single-shooting MPC using MATLAB numerical
optimization (`fmincon`).

### Stage 6 — CasADi MPC

Replacement of the repeated numerical optimization-model evaluations with a
CasADi symbolic model and automatic differentiation.

The current validated implementation uses:

CasADi → RK4 → Single Shooting → SQP → Exact Hessian → QRQP

## Forward Kinematics

The first stage of the project is the verification of the planar
free-floating SMS forward kinematics.

The implementation is located in:

Matlab_FK/

with:

- `Forward_kinematics_planar_FFSMS.m` — forward-kinematics implementation.
- `Matlab_FK_Test.m` — forward-kinematics verification/testing.

The FK stage establishes the geometric relationship between the spacecraft
base, manipulator configuration, and end-effector before introducing the
dynamic model and controllers.

## PD Controller (PID baseline directory)

The first controller implemented for the planar free-floating SMS is a
joint-space PD controller used as a classical baseline.

The controller tracks a desired manipulator configuration while the
spacecraft base remains unactuated. Consequently, manipulator motion
induces spacecraft-base motion through the dynamic coupling inherent to
the free-floating system.

### Control Law

The implemented controller is

\[
\tau_q =
K_p(q_d-q) + K_d(\dot q_d-\dot q),
\]

where

\[
q =
\begin{bmatrix}
q_1 & q_2 & q_3
\end{bmatrix}^T
\]

is the manipulator joint configuration.

The controller parameters are

\[
K_p = \mathrm{diag}(1,1,1),
\qquad
K_d = \mathrm{diag}(0.5,0.5,0.5).
\]

The desired configuration used in the baseline simulation is

\[
q_d =
\begin{bmatrix}
-0.523599\\
0\\
0
\end{bmatrix}
\]

corresponding approximately to

\[
q_d=[-30^\circ,\ 0^\circ,\ 0^\circ]^T,
\]

with zero desired joint velocity.

> **Note:** Although this controller is organized under the `PID`
> directory, the current implementation contains proportional and
> derivative terms only; therefore, it is technically a PD controller
> rather than a full PID controller.

### Free-Floating Dynamics

Only the three manipulator joints are actuated:

\[
\tau =
\begin{bmatrix}
0_{6\times1}\\
\tau_q
\end{bmatrix}.
\]

The spacecraft base is therefore unactuated and its motion arises from
the coupled SMS dynamics.

The generalized acceleration is obtained from

\[
\ddot{\Phi}
=
H^{-1}
\left(
\tau-C_{\dot{\Phi}}
\right),
\]

where \(H\) is the generalized inertia matrix and
\(C_{\dot{\Phi}}\) contains the velocity-dependent dynamic terms.

### Simulation

The nonlinear dynamics are integrated using a custom fourth-order
Runge–Kutta (RK4) implementation with

\[
\Delta t = 0.01\ {\rm s}
\]

over a 20-second simulation.

The simulation records:

- spacecraft base position
- spacecraft base orientation
- manipulator joint angles
- spacecraft linear velocity
- spacecraft angular velocity
- joint velocities
- joint torques
- kinetic energy
- linear momentum
- angular momentum

### Files

| File | Description |
|---|---|
| `main.m` | Initializes and runs the PID/PD simulation and generates plots |
| `param.m` | Defines SMS physical parameters and controller gains |
| `Calc_rb.m` | Computes the initial base position from the system center-of-mass condition |
| `SMS_dynamics_PID.m` | Implements the nonlinear coupled SMS dynamics and PD controller |
| `rk4t_PID.m` | Fourth-order Runge–Kutta integration |

### Simulation Workflow

```text
Initial manipulator configuration
            ↓
     Calculate base position
            ↓
      Initialize 18 states
            ↓
       PD controller
            ↓
       Joint torques
            ↓
    Coupled SMS dynamics
            ↓
          RK4
            ↓
      Updated system state
            ↓
        Repeat in time