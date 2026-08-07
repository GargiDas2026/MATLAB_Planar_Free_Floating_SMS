
function J = CostFunction(U,x0,param)
Q = param.Q;
R = param.R;
Np = param.Np;
dt = param.dt;
x = x0;
xref = param.xref;
J = 0;
for k=1:Np
    tau = U(3*k-2:3*k);
    x = rk4t_MPC(@SMS_dynamics_MPC,x,tau,dt);
    e = x - xref;
    J = J + e'*Q*e + tau'*R*tau;
end
end
