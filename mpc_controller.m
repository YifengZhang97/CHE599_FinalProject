function u = mpc_controller(state, state_d, params)
% params
n_horizons = params.n_horizons;
dt_mpc = params.dt_mpc;
Qmpc = params.Qmpc;
Rmpc = params.Rmpc;
tau_w = params.tau_w;
I = params.I;
minF = params.minF;
maxF = params.maxF;
% desired_state is a n_state by n_horizons array
x0 = state(1);
y0 = state(2);
theta0 = state(3);
dx0 = state(4);
dy0 = state(5);
dtheta0 = state(6);
wx0 = state(7);
s0 = sin(theta0);
c0 = cos(theta0);
% calculate the wind perturbation over n_horizons-1
t_vec = (0:n_horizons-1) * dt_mpc;
wx = wx0 * exp(-t_vec/tau_w);
B1W = [zeros(3, n_horizons); wx * dt_mpc; zeros(2, n_horizons)];
% F0 matrix
F0 = eye(6);
F0(1, 4) = dt_mpc;
F0(2, 5) = dt_mpc;
F0(3, 6) = dt_mpc;
F0(4, 3) = -dt_mpc * c0;
F0(5, 3) = -dt_mpc * s0;
B0 = dt_mpc * [zeros(3, 2);
                -s0, 0;
                c0, 0;
                0, 1/I];
B1W_f0_F0x0 = [0; 0; 0; (theta0 * c0 - s0) * dt_mpc; (theta0 * s0 + c0 -1) * dt_mpc; 0];
B1W_f0_F0x0 = repmat(B1W_f0_F0x0, 1, n_horizons);
B1W_f0_F0x0 = B1W_f0_F0x0 + B1W;
n_state = size(B0, 1);
n_u = size(B0, 2);

% cost function
% H matrix: 1/2 * x^T * Q * x
Qblocks = repmat({Qmpc}, 1, n_horizons);
Q_stacked = blkdiag(Qblocks{:});
Rblocks = repmat({Rmpc}, 1, n_horizons);
R_stacked = blkdiag(Rblocks{:});
H = blkdiag(Q_stacked, R_stacked);
% f matrix: f^T * x
% f_state_d = Qmpc * state_d;
% f = [-f_state_d(:); zeros(n_u*n_horizons, 1)];
% desired u = [-1; 0]
f_state_d = - Qmpc * state_d;
u_d = [-ones(1, n_horizons); zeros(1, n_horizons)];
f_u_d = -Rmpc * u_d;
f = [f_state_d(:); f_u_d(:)];
% inequalities constraint
A_single = [1, -1;
    -1, 1;
    1, 1;
    -1, -1];
Asingleblocks = repmat({A_single}, 1, n_horizons);
Asingle_stacked = blkdiag(Asingleblocks{:});
A = [zeros(size(A_single, 1)*n_horizons, n_state*n_horizons), Asingle_stacked];
bsingle = [2 * maxF - 1;
    -2 * minF + 1;
    2 * maxF - 1;
    -2 * minF + 1];
b = repmat(bsingle, n_horizons, 1);
% equalities constraint
F0blocks = repmat({F0}, 1, n_horizons-1);
F0_stacked = blkdiag(F0blocks{:});
B0blocks = repmat({B0}, 1, n_horizons);
B0_stacked = blkdiag(B0blocks{:});
Aeq = [eye(n_state*n_horizons), -B0_stacked];
Aeq(n_state+1:n_state*n_horizons, 1:n_state*(n_horizons-1)) = ...
    Aeq(n_state+1:n_state*n_horizons, 1:n_state*(n_horizons-1)) - F0_stacked;
beq = B1W_f0_F0x0(:);
beq(1:6) = beq(1:6) + F0 * state(1:6);

% lower bound and upper bound
lb = [];
ub = [];

options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
% initial guess XU0
u0 = zeros(2,1);
XU0 = [state_d(:); repmat(u0, n_horizons, 1)];
XU = quadprog(H,f,A,b,Aeq,beq,lb,ub,XU0,options);
u = XU(n_state*n_horizons+1 : n_state*n_horizons+n_u);
end