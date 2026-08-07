function tau = MPC_Controller(x0,param)

Np = param.Np;
tau_max = param.tau_max(:);

%% ---------- Warm Start ----------

persistent U_prev

if isempty(U_prev)

    U0 = zeros(3*Np,1);

else

    U0 = [U_prev(4:end);
          U_prev(end-2:end)];

end

%% ---------- Bounds ----------

LB = repmat(-tau_max,Np,1);
UB = repmat( tau_max,Np,1);

%% ---------- Optimizer ----------

opts = optimoptions('fmincon',...
    'Algorithm','sqp',...
    'Display','off',...
    'MaxFunctionEvaluations',5e4);

%% ---------- Solve ----------

U = fmincon(@(U) CostFunction(U,x0,param),...
            U0,...
            [],[],[],[],...
            LB,UB,...
            [],...
            opts);

%% ---------- Save optimum ----------

U_prev = U;

%% ---------- Apply first control ----------

tau = U(1:3);

end