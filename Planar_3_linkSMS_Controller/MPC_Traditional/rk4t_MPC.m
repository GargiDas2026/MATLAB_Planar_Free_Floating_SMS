function x_next = rk4t_MPC(fun,x,u,h)

k1 = fun(x,u);
k2 = fun(x + h/2*k1,u);
k3 = fun(x + h/2*k2,u);
k4 = fun(x + h*k3,u);

x_next = x + h/6*(k1+2*k2+2*k3+k4);

end