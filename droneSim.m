function [t_out, state_out, state_hat_out, u] = droneSim(t_ctrl, t_sim, state0, controller, traj, params)
% tSpan = [t_initial, t_final]
% t_ctrl = sampling rate
% t_sim = simulation rate

% sigma values for adding state observer noise
sigma_x = params.sigma_x;
sigma_y = params.sigma_y;
sigma_theta = params.sigma_theta;

% check initial state vector shape
if size(state0, 1) == 1
    state = state0.';
elseif size(state0, 2) == 1
    state = state0;
else
    error("Invalid state0.")
end

% Sytem -----------------------------
[A, B, E] = linearized_dynamics(params);

A_aug = [A, E;
         zeros(1, size(A,2)), -1/params.tau_w];

B_aug = [B;
         zeros(1, size(B,2))];

G_aug = [zeros(size(A,1),1);
         params.sigma_w];  % process noise enters through this

sys = ss(A_aug, [B_aug G_aug], eye(7), 0);
sys_d = c2d(sys, t_sim);

Ad = sys_d.A;
Bd = sys_d.B(:,1:size(B,2));
Gd = sys_d.B(:,end);

% LQR controller computation
Q = diag([100 100 10 10 10 10 0]);
Qf = Q;
R = diag([1 1]);
N = length(traj.t);
params.K = dlqr(Ad, Bd, Q, R);
params.Kk = fh_dlqr(Ad, Bd, Q, R, Qf, N);

tspan = 0:t_sim:traj.t(end);
state_out = zeros(7, length(tspan));
state_hat_out = zeros(7, length(tspan));
u = zeros(size(Bd,2), length(tspan));


% initialize state_hat with the initial state0
state_hat = state0;
state_out(:,1) = state0;
state_hat_out(:,1) = state0;

% initialize the variance of the estimator Pk
Pk = 0.01 * eye(7);

% simulation loop
for k = 1:length(tspan)-1
    params.k = k;
    t = tspan(k);

    % check simulation stop condition - drone crashing
    if state(2) < 0.01
        break
    end

    % check for controller rate
    if mod(k-1, round(t_ctrl/t_sim)) == 0
        if k > 1
            % observer gets x, y, theta value with disturbance
            xytheta_obs = [state(1) + sigma_x * randn;
                state(2) + sigma_y * randn;
                state(3) + sigma_theta * randn];
            % extended Kalman filter for observed state
            [state_hat, Pk] = kalman_filter(state_hat, u_curr, xytheta_obs, Pk, t_ctrl, params);
        end
        % query desired trajectory point at current timestep
        state_des = trajhandle(t, traj, params);
        u_curr = controller(state_hat, state_des, params);
        % rotor force clamping
        u_curr = u_clamp(u_curr, params);
        % record to u
        u(:,k) = u_curr;
    else
        u(:,k) = u(:,k-1);  % ZOH
    end


    % update true state using discretied dynamics within function - k+1
    state = stateUpdate(state, t_sim, u_curr, params);
    state_out(:, k+1) = state; % state = k+1

    % state_hat = state; % assume we know every state, for debugging
    state_hat_out(:, k+1) = state_hat;

end

state_out = state_out.';
state_hat_out = state_hat_out.';
u = u';
t_out = tspan;

end