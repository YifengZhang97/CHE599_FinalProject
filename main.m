clear; close all;

%{
params.m = 0.18;          % mass (kg)
params.l = 0.086;          % arm length (m)
params.I = params.m*params.l^2;      % mass moment of inertia (kg*m^2)
params.g = 9.81;       % gravit. accel. (m/s^2)
%}

% dimensionless params
% dimensionless unit
% params.M = params.m;
% params.L = params.l;
% params.T = sqrt(params.L/params.g);
% params.V = params.L/params.T;
% params.A = params.g;
% dimensionless params
params.I = 1; % params.I_nrom = params.I/(params.M * params.L^2) = 1

% minF, maxF is for single rotor
params.minF = 0;
params.maxF = 1.5;

% wind perturbation
params.tau_w = 1;
params.sigma_w = 0.05;
params.sigma_x = 0.01;
params.sigma_y = 0.01;
params.sigma_theta = 0.01;
tSpan = [0, 10];

t_ctrl = 1e-2;
t_sim = 1e-3;

% controller params
params.Kp = [4.0, 25.0, 4.0];
params.Kd = [4.0, 10.0, 4.0];

% trajectory generation
vmax = 2; amax = 0.2; s0 = [0;1;0;0;0;0];
traj = traj_trap([0, 30], vmax, amax, t_ctrl);
% traj = traj_figure8(2, 1, tSpan(2), t_sim, amax, s0);

state0 = [traj.x(1);traj.y(1);traj.theta(1);
    traj.vx(1);traj.vy(1);traj.omega(1); 0];

[t_out, state_out, state_hat_out, u_out] = droneSim(t_ctrl, t_sim, state0, @pd_controller, traj, params);

%% Performance evaluation
% [e_pos, ctrl_effort] = controller_evaluate(traj, t_out, state_out, state_hat_out, u_out, params);
plot_quad(traj, t_out, state_out, u_out)

%% Quadrotor Animation with Optional Video Saving
video_save(t_out, state_out, traj)
