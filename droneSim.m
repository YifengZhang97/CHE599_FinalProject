function [t_out, state_out, state_hat_out, u_out] = droneSim(state0, traj, params)
dt_sim = params.dt_sim;
dt_sensor = params.dt_sensor;
dt_ctrl = params.dt_ctrl;
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

tspan = 0:dt_sim:traj.t(end);
state_out = zeros(7, length(tspan));
state_hat_out = zeros(7, length(tspan));
u_out = zeros(2, length(tspan));

% initialize state_hat with the initial state0
state_hat = state0;
state_out(:,1) = state0;
state_hat_out(:,1) = state0;

% initialize the variance of the estimator Pk
Pk = 0.01 * eye(7);

t_sensor_last = tspan(1);
t_ctrl_last = tspan(1);

u_curr = zeros(2, 1);

% simulation loop
for k = 1:length(tspan)-1
    t = tspan(k);
    % display t evey 1000 loop for simulation progress bar
    if mod(k-1, 1000) == 0
        fprintf('t = %8.4f\n', t)
    end
    % check for controller rate, apply controller
    if t >= t_ctrl_last + dt_ctrl
        t_ctrl_last = t;
        if strcmp(params.controller, 'pd')
            % query desired trajectory point at current timestep
            state_d = trajhandle(t, traj, params);
            u_curr = pd_controller(state_hat, state_d, params);
            % rotor force clamping
            u_curr = u_clamp(u_curr, params);
        elseif strcmp(params.controller, 'lqg')
            % query desired trajectory point at current timestep
            state_d = trajhandle(t, traj, params);
            u_curr = params.K * (state_d - state_hat(1:6));
            % rotor force clamping
            u_curr = u_clamp(u_curr, params);
        elseif strcmp(params.controller, 'mpc')
            % query desired trajectory point at current timestep
            dt_mpc = params.dt_mpc;
            n_horizons = params.n_horizons;
            t_mpc = t + dt_mpc : dt_mpc : t + n_horizons * dt_mpc;
            state_d = trajhandle(t_mpc, traj, params);
            u_curr = mpc_controller(state_hat, state_d, params);
        end
    end
    % record to u_out
    u_out(:, k) = u_curr;

    % update true state using discretied dynamics within function - k+1
    state = stateUpdate(state, dt_sim, u_curr, params);
    state_out(:, k+1) = state; % state = k+1
    % check for sensor rate, apply observer
    if t >= t_sensor_last + dt_sensor
        t_sensor_last = t;
        % observer gets x, y, theta value with disturbance
        xytheta_obs = [state(1) + sigma_x * randn;
            state(2) + sigma_y * randn;
            state(3) + sigma_theta * randn];
        % extended Kalman filter for observed state
        [state_hat, Pk] = kalman_filter(state_hat, u_curr, xytheta_obs, Pk, dt_sensor, params);
        % state_hat = state; % assume we know every state, for debugging
    end
    % record to state_hat_out
    state_hat_out(:, k+1) = state_hat;
end

state_out = state_out.';
state_hat_out = state_hat_out.';
u_out = u_out';
t_out = tspan;

end