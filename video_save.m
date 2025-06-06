function video_save(t_out, state_out, traj, L)

t = t_out;            % time vector
s = state_out(:,1:6);            % states: [x, y, theta, vx, vy, omega]
save_video = true;    % set to true to save animation
video_name = 'quadrotor_trajectory.mp4';

% Playback rate
fps = 30;
% speed ratio
speed_ratio = 1;
dt_draw = 1 / fps * speed_ratio;

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
    dx = L * cos(theta);
    dy = L * sin(theta);
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
    
    title(sprintf('t = %4.2f sec', (i-1) * dt_draw))
    % pause(dt_draw);
end

% Close video writer
if save_video
    close(v);
end

end