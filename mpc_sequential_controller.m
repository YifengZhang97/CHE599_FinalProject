function u = mpc_sequential_controller(state, state_d, params)
% params
n_horizons = params.n_horizons;
dt_mpc = params.dt_mpc;
Qmpc = params.Qmpc;
Rmpc = params.Rmpc;
tau_w = params.tau_w;
I = params.I;
minF = params.minF;
maxF = params.maxF;
n_state = 6;
n_u = 2;
% desired_state is a n_state by n_horizons array
state0 = state(1:6);
wx0 = state(7);
% calculate the wind perturbation over n_horizons-1
t_vec = (0:n_horizons-1) * dt_mpc;
wx = wx0 * exp(-t_vec/tau_w);

% objective function and inequality constraints never change over
% iterations
% H matrix: 1/2 * x^T * Q * x
Qblocks = repmat({Qmpc}, 1, n_horizons);
Q_stacked = blkdiag(Qblocks{:});
Rblocks = repmat({Rmpc}, 1, n_horizons);
R_stacked = blkdiag(Rblocks{:});
H = blkdiag(Q_stacked, R_stacked);
% f matrix: f^T * x
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

% lower bound and upper bound
lb = [];
ub = [];

options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');

% initial guess of u
U = zeros(2, n_horizons);
XU = [zeros(n_state * n_horizons, 1); U(:)];

for k = 1:100
    % update equality constraints
    Aeq = [eye(n_state*n_horizons), zeros(n_state*n_horizons, n_u*n_horizons)];
    beq = zeros(n_state*n_horizons, 1);
    state = state0;
    for i = 1:n_horizons
        u_current = XU(n_state*n_horizons+(i-1)*n_u+1 : n_state*n_horizons+i*n_u);
        [Ac, Bc, Cc] = C2Linearization(state, u_current, wx(i), params);
        [Ad, Bd, Cd] = LinearizedC2D(Ac, Bc, Cc, dt_mpc);
        % update state
        state = Ad * state + Bd * u_current + Cd;
        % incorporate into XU0
        XU((i-1)*n_state+1 : i*n_state) = state;
        Aeq((i-1)*n_state+1 : i*n_state, n_state*n_horizons+(i-1)*n_u+1 : n_state*n_horizons+i*n_u) = -Bd;
        if i == 1
            beq(1 : n_state) = Ad * state0 + Cd;
        else
            Aeq((i-1)*n_state+1 : i*n_state, (i-2)*n_state+1 : (i-1)*n_state) = -Ad;
            beq((i-1)*n_state+1 : i*n_state) = Cd;
        end
    end
    lb = XU - 0.05;
    ub = XU + 0.05;
    for i = 1:n_horizons
        lb((i-1)*n_state + 3) = -0.5;
        ub((i-1)*n_state + 3) = 0.5;
    end
    XU_new = quadprog(H,f,A,b,Aeq,beq,lb,ub,XU,options);
    err = norm(XU_new - XU);
    if err < 1e-3
        break;
    end
    XU = XU_new;
end
u = XU(n_state*n_horizons+1 : n_state*n_horizons+n_u);

end


function [Ac, Bc, Cc] = C2Linearization(state, u, Wx, params)
% params
I = params.I;
% state
theta = state(3);
ct = cos(theta);
st = sin(theta);
Ac = zeros(6, 6);
% control input
u1 = u(1);
% here calculate u1+1 for computational efficiency
u1 = u1 + 1;
u2 = u(2);
u1_ct = u1 * ct;
u1_st = u1 * st;
% A matrix
Ac(1, 4) = 1;
Ac(2, 5) = 1;
Ac(3, 6) = 1;
Ac(4, 3) = -u1_ct;
Ac(5, 3) = -u1_st;
% B matrix
Bc = zeros(6, 2);
Bc(4, 1) = -st;
Bc(5, 1) = ct;
Bc(6, 2) = 1/I;

f0 = [state(4);
    state(5);
    state(6);
    -u1_st + Wx;
    u1_ct - 1;
    u2/I];
Cc = f0 - Ac * state - Bc * u;

end

function [Ad, Bd, Cd] = LinearizedC2D(Ac, Bc, Cc, dt)
% x_{k+1} = Ad * x_k + Bd * u_k + Cd
A_dt = Ac * dt;
Ad = expm(A_dt);
% calculate \int_{0}^{\Delta t} e^{A\tau} \, d\tau
% here for computational efficiency, we pick i = 0:5
int_At = eye(size(Ac, 1)) * dt;
for i = 1:5
    int_At = int_At + dt * A_dt^i / factorial(i+1);
end
Bd = int_At * Bc;
Cd = int_At * Cc;

end