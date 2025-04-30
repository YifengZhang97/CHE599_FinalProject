clear; close all;

% original unit
params_origin.m = 0.18;          % mass (kg)
params_origin.l = 0.086;          % arm length (m)
params_origin.I = params_origin.m*params_origin.l^2;      % mass moment of inertia (kg*m^2)
params_origin.g = 9.81;       % gravit. accel. (m/s^2)

% dimensionless params
% dimensionless unit
units.M = params_origin.m;
units.L = params_origin.l;
units.T = sqrt(params_origin.l / params_origin.g);
units.V = sqrt(params_origin.l * params_origin.g);
units.A = params_origin.g;

% dimensionless params
params.I = 1; % params.I_nrom = params.I/(params.M * params.L^2) = 1

% minF, maxF is for single rotor
params.minF = 0;
params.maxF = 1.5;

% wind perturbation
params.tau_w = 1 / units.T;
params.sigma_w = 0.02;
% observer error
params.sigma_x = 0.01 / units.L;
params.sigma_y = 0.01 / units.L;
params.sigma_theta = 0.01;

params.dt_sim = 1e-4 / units.T;
params.dt_sensor = 1e-3 / units.T;

% controller type
params.controller = 'lqg';

% controller params
if strcmp(params.controller, 'pd')
    params.dt_ctrl = params.dt_sensor;
    params.Kp = [8.0, 25.0, 25.0];
    params.Kd = [20.0, 20.0, 10.0];
elseif strcmp(params.controller, 'lqg')
    params.dt_ctrl = params.dt_sensor;
    params.Q = diag([1.0, 1.0, 0.25, 0.001, 0.001, 0.001]);
    params.R = diag([0.001, 0.001]);
    [params.Ad, params.Bd] = AdBd(params);
    [params.K, ~, ~] = dlqr(params.Ad, params.Bd, params.Q, params.R);
elseif strcmp(params.controller, 'mpc')
    params.dt_mpc = 5e-3 / units.T;
    params.dt_ctrl = params.dt_mpc;
    params.n_horizons = 20;
    params.Qmpc = diag([1.0, 1.0, 0.25, 0.001, 0.001, 0.001]);
    params.Rmpc = diag([0.001, 0.001]);
end

% trajectory generation
vmax = 6; amax = 0.25; s0 = [0;1;0;0;0;0];
% traj = traj_trap([0, 30], vmax, amax, params.dt_sim);
traj = traj_figure8(10, 5, 80, params.dt_sim, amax, s0);

state0 = [traj.x(1);traj.y(1);traj.theta(1);
    traj.vx(1);traj.vy(1);traj.omega(1); 0];

[t_out, state_out, state_hat_out, u_out] = droneSim(state0, traj, params);

%% Performance evaluation
% [e_pos, ctrl_effort] = controller_evaluate(traj, t_out, state_out, state_hat_out, u_out, params);
plot_quad(traj, t_out, state_out, u_out)

%% Quadrotor Animation with Optional Video Saving
% video_save(t_out, state_out, traj)

function [Ad, Bd] = AdBd(params)
dt_ctrl = params.dt_ctrl;
Ad = eye(6);
Ad(1, 4) = dt_ctrl;
Ad(2, 5) = dt_ctrl;
Ad(3, 6) = dt_ctrl;
Ad(4, 3) = -dt_ctrl;
Bd = zeros(6, 2);
Bd(5, 1) = dt_ctrl;
Bd(6, 2) = dt_ctrl / params.I;
end