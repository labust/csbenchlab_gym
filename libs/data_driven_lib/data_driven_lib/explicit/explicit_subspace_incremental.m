d = [ones(L, 1) * du_max; -ones(L, 1) * du_min; ones(L, 1) * y_max; -ones(L, 1) * y_min; ones(L, 1) * u_max; -ones(L, 1) * u_min];
Mu = zeros(2*L * m, L* m); 
Mu(1:L, 1:L) = eye(L);
Mu(L+1:end, 1:L) = -eye(L);
My = zeros(2*L * m, L* m); 
My(1:L, 1:L) = eye(L);
My(L+1:end, 1:L) = -eye(L);


ut = tril(ones(L));
G = [Mu * Uf; My * Yf; ut * Uf; -ut * Uf];

FW = diag(ones(L-1, 1), 1);
FW = FW(1:end-1, :);
BW = eye(L);
BW = BW(1:end-1, :);
D = FW - BW;
D = [1, zeros(1, L-1); D];


H = 2 * Yf' * Q * Yf + 2 * Uf' * R  * Uf + 2 * Uf' * D' * Rd * D * Uf;
H = (H + H') / 2;

F = [zeros(size(H, 1), Tini * (m + p)), -2 * 0 * sum(Uf, 1)', -2 * Q * sum(Yf, 1)'];
F(:, (Tini-1)*m+1:Tini*m) = -2 * Rd * Uf(1, :);

Wp = [Up; Yp; zeros(2, size(Up, 2))];
Wp_inv = pinv(Wp);


Vp = null(Wp);

H_hat = Vp' * H * Vp;
G_hat = G * Vp;
F_hat = Vp' * (H * Wp_inv  + F);

u00E = zeros(2*L, size(Wp_inv, 2));
u00E(1:L, end-1) = -1;
u00E(L+1:end, end-1) = 1;
E_hat = -G * Wp_inv;
E_hat(end-2*L+1:end, :) = u00E;

% F_hat = Vp' * (H * Wp_inv);
% E_hat = - G * Wp_inv;

K = Uf * Vp;
Vf = null(K);
% Kf = 0.1*K';
Kf = null(Vf');

H2 = Kf' * H_hat * Kf;
% H2 = (H2 + H2') / 2;
F2 = Kf' * F_hat;
G2 = G_hat * Kf;


u0 = sdpvar(1, 1, 'full');
u0rep = zeros(size(u_ini, 1), size(u_ini, 2));
g = sdpvar(size(H2, 1), 1, 'full');
% x0 = sdpvar(1, 1, 'full');
% y0 = O * repmat(x0, size(O, 2), 1) + Cl * u0rep;
x0 = sdpvar(size(O, 2), 1, 'full');
y0 = O * x0;
uf = sdpvar(L, size(u_ini, 2), 'full');
yf = sdpvar(L, size(y_ini, 2), 'full');

yref = sdpvar(1, 1, 'full');

eps = [u0rep; y0; u0; yref];



G2(abs(G2) < 1e-8) = 0;

Objective = 0.5 * g' * (H2 + eye(size(H2, 1)) * 1e-6) *  g + eps' * F2' * g;
Model = G2 * g <= E_hat * eps + d;


yy = Yf * Vp * Kf;
yy(abs(yy) < 1e-8) = 0;

Model = [Model, uf == Uf * Wp_inv * eps + Uf * Vp * Kf * g];
Model = [Model, yf == Yf * Wp_inv * eps + yy * g];
