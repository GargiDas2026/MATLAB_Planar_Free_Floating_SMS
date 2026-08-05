
function tau = MPC_Controller(x0,param)
% USER MUST COMPLETE COST FUNCTION WEIGHTS IF DESIRED
Np = param.Np;
dt = param.dt;
tau_max = param.tau_max(:);
U0 = zeros(3*Np,1);
LB = repmat(-tau_max,Np,1);
UB = repmat( tau_max,Np,1);

opts = optimoptions('fmincon','Algorithm','sqp','Display','off',...
    'MaxFunctionEvaluations',5e4);

U = fmincon(@(U) CostFunction(U,x0,param),U0,...
    [],[],[],[],LB,UB,[],opts);

tau = U(1:3);
end
