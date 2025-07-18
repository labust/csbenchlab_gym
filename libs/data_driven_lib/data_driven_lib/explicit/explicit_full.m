
% x0 = sdpvar(1, 1, 'full');
x0 = sdpvar(size(O, 2), 1, 'full');
yref = sdpvar(1, 1, 'full');
u0 = sdpvar(1, 1);
uf = sdpvar(L, size(u_ini, 2), 'full');
yf = sdpvar(L, size(y_ini, 2), 'full');
g = sdpvar(idx.a.sz, 1, 'full');

u0rep = repmat(u0, size(u_ini, 1), size(u_ini, 2));

% y0 = O * repmat(x0, size(xini, 1), size(xini, 2))  + Cl * u0rep;
y0 = O * x0  + Cl * u0rep;

uref = yref / params.end_point;
uref = 0;

l1 = 1e4;
sigma = sdpvar(L, size(y_ini, 2), 'full');

Objective = (yref - yf') * Q * (yref - yf) + (uf' - uref) * R * (uf - uref) + l1*sigma*sigma;
Model = Up * g == u0rep;
Model = [Model, Yp * g == y0];
Model = [Model, Uf * g == uf];
Model = [Model, Yf * g == yf + sigma];
Model = [Model, u_min <= uf <= u_max];
Model = [Model, y_min <= yf <= y_max];
