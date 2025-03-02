function get_qp_matrices = OAMPC_formulation(p)

n_hor = p.n_hor;
nX = p.nX;
nU = p.nU;
mu = p.mu;
lheel = p.lheel;
lfoot = p.lfoot;
dpxMax = 0.6;

%% construct casadi opti stack
opti = casadi.Opti('conic');
optVar = opti.variable(nU+nX,n_hor);
U_ = optVar(1:nU,:);
X_ = optVar(nU+1:nU+nX,:);
dpx_ = opti.variable(2,1);

X0 = opti.parameter(nX,1);
U0 = opti.parameter(nU,1);
Xd = opti.parameter(nX,n_hor);
Ud = opti.parameter(nU,n_hor);
eta_vec = opti.parameter(1,n_hor);
idx_vec = opti.parameter(1,n_hor);
params = [X0;U0;Xd(:);Ud(:);eta_vec';idx_vec'];

obj = casadi.MX(0);

[A, B, D] = get_ABD(X0, U0, p);

% objective
for kk = 1:n_hor
    discount = p.decay^(kk-1);
    eX = Xd(:,kk) - X_(:,kk);
    eU = Ud(:,kk) - U_(:,kk);
    obj = obj + discount * eX' * p.Q * eX;
    obj = obj + discount * eU' * p.R * eU;
    obj = obj + discount * U_(:,kk)' * p.Ru * U_(:,kk);
    if kk == 1
        dU = U_(:,1) - U0;
    else
        dU = U_(:,kk) - U_(:,kk-1);
    end
    obj = obj + dU' * p.Rdu * dU;
end
obj = obj + dpx_' * p.Qdp * dpx_;

% Dynamics
for kk = 1:n_hor
    if kk == 1
        opti.subject_to(X_(:,1) == A * X0 + B * U_(:,1) + D +...
            eta_vec(1) * A(:,7) * dpx_(idx_vec(1)));
    else
        opti.subject_to(X_(:,kk) == A * X_(:,kk-1) + B * U_(:,kk) + D +...
            eta_vec(kk) * A(:,7) * dpx_(idx_vec(kk)));
    end
end

% inequality constraint
for kk = 1:n_hor
    opti.subject_to(-mu * U_(2,kk) <= U_(1,kk));
    opti.subject_to(U_(1,kk) <= mu * U_(2,kk));
    opti.subject_to(U_(2,kk) <= p.Umax);
    opti.subject_to(-lheel * U_(2,kk) <= U_(3,kk));
    opti.subject_to(U_(3,kk) <= lfoot * U_(2,kk));
end
opti.subject_to(-dpxMax <= dpx_);
opti.subject_to(dpx_ <= dpxMax);

opti.minimize(obj);

opti.solver('osqp');

ff = opti.f;
gg = opti.g;
xx = opti.x;
pp = opti.p;
lbg = opti.lbg;
ubg = opti.ubg;
get_qp_matrices = opti.to_function('matQP',{params},...
    {jacobian(ff,xx)-xx'*hessian(ff,xx),hessian(ff,xx),...
    jacobian(gg,xx),jacobian(gg,xx)*xx-gg,...
    lbg,ubg});

get_qp_matrices.save('get_qp_matrices.casadi');

end

function [A, B, D] = get_ABD(Xt, Ut, p)
% --- continuous system ---
% state: X = [x;z;th;dx;dz;dth;c]
% control: U = [fx;fz;tau]
% Xdot = Ac * X + Bc * U + Dc

mass = p.mass;
J = p.J;
g = p.g;
Ts = p.dt_mpc;
nX = p.nX;
nU = p.nU;

pc = Xt(1:2);
c = [Xt(p.idx_c);0];

F = Ut(1:2);
tau = Ut(3);

ddpc = 1/mass * F + [0;-g];

moment = wedgeMap(c-pc)*F + tau;
ddth = 1/J * moment;

f = [Xt(4:6);ddpc;ddth;0];
dfdX = jacobian(f,Xt);
dfdU = jacobian(f,Ut);

A = eye(nX) + Ts * dfdX;
B = Ts * dfdU;
D = Ts * f(1:nX) - Ts * dfdX * Xt(1:nX) - Ts * dfdU * Ut;

end

function out = wedgeMap(in)
out = [-in(2) in(1)];
end






