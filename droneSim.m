function [t_out, state_out, u] = droneSim(t_ctrl, t_sim, state0, controller, traj, params)
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
C = eye(size(A));
D = zeros(size(B,1), size(B,2)+1);
sys_c = ss(A,[B E],C,D);
sys_d = c2d(sys_c, t_ctrl, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B(:,1:2);
Ed = sys_d.B(:,3);

tspan = 0:t_sim:traj.t(end);
state_out = zeros(size(Ad,1)+1, length(tspan));
u = zeros(size(Bd,2), length(tspan));


% n = floor((tSpan(2) - tSpan(1)) / ts) + 1;
% t_out = zeros(n + 1, 1);
% state_out = zeros(n + 1, length(state0));
% u_out = zeros(n + 1, 2);

% initialize state_hat with the initial state0
state_hat = state0;
state_out(:,1) = state0;

% initialize the variance of the estimator Pk
Pk = eye(7);


% simulation loop
for k = 1:length(tspan)-1
    t = tspan(k);

    % check simulation stop condition - drone crashing
    if state(2) < 0.01
        break
    end

    % check for controller rate
    if mod(k-1, round(t_ctrl/t_sim)) == 0
        % query desired trajectory point at current timestep
        state_des = trajhandle(t, traj, params);
        u_curr = controller(state_out(:,k), state_des, params);
        u(:,k) = u_curr;
    else
        u(:,k) = u(:,k-1);  % ZOH
    end

    % update true state using discretied dynamics within function - k+1
    [state, u_curr] = stateUpdate(state, t_sim, u(:,k), params);
    state_out(:, k+1) = state; % state = k+1

    % observer gets x, y, theta value with disturbance
    xytheta = [state(1) + sigma_x * randn;
        state(2) + sigma_y * randn;
        state(3) + sigma_theta * randn];

    % extended Kalman filter for observed state
    % [state_hat, Pk] = kalman_filter(state_hat, Pk, xytheta, ts, params);
    state_hat = state;

end

state_out = state_out';
u = u';
t_out = tspan;

end