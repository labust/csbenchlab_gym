otter_optim_params;
Ts = 0.05;
seed = 105;
rng(seed);
is_incremental = 1;
ident_n = 3;
sim_region_models;


model_idx = 1;
model = sss{model_idx};
timeseries_data = tts{model_idx};
D_u = Db_u(model_idx, :)';
D_y = Db_y(model_idx, :)';


Tini = 3;
L = 10;
Lc = L;
scale_u = 1;
mux.Inputs = params.m;
mux.Outputs = params.p;

params.Lc = Lc;
params.L = L;
params.Tini = Tini;
fit.na = 2;
fit.nb = 1;
fit.nk = 0;

[O, Cl] = construct_oc_matrices(model.A, model.B, model.C, model.D, params.Tini);
as_tf = tf(model);

H_u = DDHelpers.hankel_matrix(D_u, L*m+n+5);
if DDHelpers.sigma_rank(H_u, 0.5) < n + m*L+5
    error("Input Hankel matrix is singular..");
end

params.D_u = D_u * scale_u;
params.D_y = D_y;
params.end_point = 0;


y_ref = 0;

data = DeePCHelpers.create_basic_data_model(params, mux);
idx = data.idx;

[data.b, data.A_lt, ...
     data.b_lt, data.optim_f, data.x_op] = ...
        DeePCHelpers.update_matrices(y_ref, y, ...
            data.vel_stop_idx, params.end_point, ...
            data.b, ...
            data.A_lt, data.b_lt, ...
            data.optim_f, data.x_op, ...
            idx, params);


u_ini = params.D_u(1:params.Tini);
y_ini = params.D_y(1:params.Tini);

xini = pinv(O) * (y_ini - Cl * u_ini);

Up = data.A(idx.uini_v.r, idx.a.r);
Yp = data.A(idx.yini_v.r, idx.a.r);
Uf = data.A(idx.u_v.r, idx.a.r);
Yf = data.A(idx.y_v.r, idx.a.r);


Qb = 1e0;
% R = 0;
R = 3e-6; %1
% R = 0.9e-6; %4
% R = 5e-6; %2
% R = 2e-6; %3

% R = 1e-7;
Q = Qb;
Rd = 1e-6;


du_max = 200 * scale_u; du_min = -du_max; 
u_max = 500 * scale_u; u_min = -u_max;
y_max = 2.5; y_min = -y_max; 

% explicit_full;
explicit_subspace_incremental;

Model = [Model, u_min <= u0 <= u_max];
Model = [Model, y_min <= y0 <= y_max];
Model = [Model, y_min+0.5 <= yref <= y_max-0.5];

u00 = 0;
yref0 = 1;
x00 = 0;

l1 = 1e4;
Model = [Model, uf(end) == 0];
sigma = sdpvar(1, 1, 'full');
Model = [Model, yf(end) == eps(end) + sigma];
Model = [Model, du_min <= uf <= du_max];
Model = [Model, y_min <= yf <= y_max];

Objective = Objective + l1*sigma*sigma;

% Objective = replace(Objective, u0, u00);
% Model = replace(Model, u0, u00);

% 
% Objective = replace(Objective, yref, yref0);
% Model = replace(Model, yref, yref0);

% Objective = replace(Objective, x0, x00);
% Model = replace(Model, x0, x00);

options = sdpsettings('verbose', 1, 'debug', 1);


[sol, diagnostics, aux, Valuefunction, OptimalSolution] = solvemp(Model, Objective, options, [x0; u0; yref], uf(1));
explicit_sol2 = mpt_mpsol2pu(sol);


% 
% figure(1); plot(Valuefunction)
% figure(2); plot(OptimalSolution)


% sss.feval([0; -2], 'primal', 'tiebreak', 'obj')



function [O, Cl] = construct_oc_matrices(A, B, C, D, L)

    O = C;
    Cl = D;
    
    a = A;
    m = size(B, 2);
    p = size(C, 1);
    h = C * B;
    for i=2:L
        O = [O; C * a];
        
        Cl = [Cl zeros(height(Cl), m); zeros(p, width(Cl)), D];
        Cl(end-p+1:end, m+1:(i*m)) = Cl(end-2*p+1:end-p, 1:((i-1)*m));
        Cl(end-p+1:end, 1:m) = h;
        h = C * a * B;
        a = a*a;
    end
   
end