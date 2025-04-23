clc; clear; close all;

params.m = 0.18;          % mass (kg)
params.l = 0.086;          % arm length (m)
params.I = params.m*params.l^2;      % mass moment of inertia (kg*m^2)
params.g = 9.81;       % gravit. accel. (m/s^2)

% dimensionless params
params.normalize = false;
params.M = params.m;
params.L = params.l;
params.T = sqrt(params.L/params.g);
params.V = params.L/params.T;
params.A = params.L/params.T^2;
params.Ibar = params.I/(params.M * params.L^2);

params.minF = 0;
params.maxF = 2*params.m*params.g;

params.tau_w = 1;
params.sigma_w = 0.1;
params.sigma_x = 0.01;
params.sigma_y = 0.01;
params.sigma_theta = 0.01;
tSpan = [0, 10];

t_ctrl = 1e-3;
t_sim = 1e-4;

% controller params
params.Kp = [30,30,150];
params.Kd = [10,10,20];

% trajectory generation
vmax = 2; amax = 2; s0 = [0;1;0;0;0;0];
traj = traj_trap(tSpan, vmax, amax, t_ctrl, params);
% traj = traj_figure8(2, 1, tSpan(2), t_sim, amax, s0);

state0 = [traj.x(1);traj.y(1);traj.theta(1);
    traj.vx(1);traj.vy(1);traj.omega(1); 0];

% [t_out, state_out, state_hat_out, u_out] = droneSim(t_ctrl, t_sim, state0, @pd_controller, traj, params);
% [t_out, state_out, state_hat_out, u_out] = droneSim(t_ctrl, t_sim, state0, @lqr_controller, traj, params);
[t_out, state_out, state_hat_out, u_out] = droneSim(t_ctrl, t_sim, state0, @fh_lqr_controller, traj, params);

%% Performance evaluation

% [e_pos, ctrl_effort] = controller_evaluate(traj, t_out, state_out, state_hat_out, u_out, params);
plot_quad(traj, t_out, state_out, u_out)


%% Quadrotor Animation with Optional Video Saving

t = t_out;            % time vector
s = state_out(:,1:6);            % states: [x, y, theta, vx, vy, omega]
arm_length = params.l;     % quad arm length
save_video = true;    % set to true to save animation
video_name = 'quadrotor_trajectory.mp4';

% Playback rate
fps = 30;
dt_draw = 1 / fps;

% Interpolated time vector
t_anim = t(1):dt_draw:t(end);
s_anim = interp1(t, s, t_anim);

% Setup figure
fig = figure; clf;
axis equal; hold on; grid on;
xlim([min(s(:,1)) - 0.5, max(s(:,1)) + 0.5]);
ylim([min(s(:,2)) - 0.5, max(s(:,2)) + 0.5]);
% xlim([-2.5,2.5]);
% ylim([3.5,6.5]);
xlabel('x [m]'); ylabel('y [m]');
title('2D Quadrotor Animation');

% Quad body (arm)
quad = plot(NaN, NaN, 'k-', 'LineWidth', 3);

% Center of mass
marker = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Thrust direction arrow
thrust_arrow = quiver(NaN, NaN, NaN, NaN, 0.2, 'r', 'LineWidth', 1.5);

% Static planned trajectory
trail = plot(traj.x, traj.y, 'r--', 'LineWidth', 1.2);

% Actual trajectory trail (grows dynamically)
trail_x = []; trail_y = [];
trail_line = plot(NaN, NaN, 'b--', 'LineWidth', 1.5);

% Setup video writer
if save_video
    v = VideoWriter(video_name, 'MPEG-4');
    v.FrameRate = fps;
    open(v);
end

% Animation loop
for i = 1:length(t_anim)
    x = s_anim(i,1);
    y = s_anim(i,2);
    theta = s_anim(i,3);

    % Arm endpoints
    dx = (arm_length / 2) * cos(theta);
    dy = (arm_length / 2) * sin(theta);
    x1 = x - dx;  y1 = y - dy;
    x2 = x + dx;  y2 = y + dy;

    % Update graphics
    set(quad, 'XData', [x1, x2], 'YData', [y1, y2]);
    set(marker, 'XData', x, 'YData', y);

    thrust_x = -sin(theta) * 0.3;
    thrust_y =  cos(theta) * 0.3;
    set(thrust_arrow, 'XData', x, 'YData', y, 'UData', thrust_x, 'VData', thrust_y);

    % Trail
    trail_x(end+1) = x;
    trail_y(end+1) = y;
    set(trail_line, 'XData', trail_x, 'YData', trail_y);

    drawnow;

    % Save frame if needed
    if save_video
        writeVideo(v, getframe(fig));
    end

    pause(dt_draw);
end

% Close video writer
if save_video
    close(v);
end


