function [Y, tau_q, KE,P,M] = rk4t_PID(X,tspan,y0)
% rk4t function for solving equations using 4TH ORDER RK METHOD.
% INPUTS
% X function handle 
% tspan simulation running time
% y0 intial state
%
% OUTPUT
% y solution matrix
%

h = diff(tspan);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y0 = y0(:);   % Make a column vector.
neq = length(y0);
N = length(tspan);
Y = zeros(neq,N);%output vector
F = zeros(neq,4);
tau_q = zeros(N,3);% joint control torque
M = zeros(N,3); % Angular momentum
P = zeros(N,3); % Linear momentum
KE = zeros(N,1); % Kinetic Energy

Y(:,1) = y0;

for i = 2:N
  ti = tspan(i-1);
  hi = h(i-1);
  yi = Y(:,i-1);

  [F(:,1),tau_q(i-1,:),KE(i-1,:),P(i-1,:), M(i-1,:)] = feval(X,ti,yi);
  [F(:,2),~,~,~,~] = feval(X,ti+0.5*hi,yi+0.5*hi*F(:,1));
  [F(:,3),~,~,~,~] = feval(X,ti+0.5*hi,yi+0.5*hi*F(:,2));  
  [F(:,4),~,~,~,~] = feval(X,ti,yi+hi*F(:,3));
  
  Y(:,i) = yi + (hi/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));



end
  [~,tau_q(N,:),~,~,~] = (feval(X,tspan(N),Y(:,N)));
  [~,~,KE(N,:),~,~] = (feval(X,tspan(N),Y(:,N)));
  [~,~,~,P(N,:),~] = (feval(X,tspan(N),Y(:,N)));
  [~,~,~,~,M(N,:)] = (feval(X,tspan(N),Y(:,N)));
  

Y = Y.';

end