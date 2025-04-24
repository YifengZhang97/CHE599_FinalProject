function traj = traj_trap(waypoints, v_max, a_max, dt)
% waypoints = [start, end] positions
% T = total duration in s

x_start = waypoints(1); x_end = waypoints(2);

D = x_end - x_start;     % total distance
dir = sign(D);           % motion direction (+1 or -1)
D = abs(D);              % work with magnitude

% Compute acceleration time and cruise time
t_accel = v_max / a_max;
D_accel = 0.5 * a_max * t_accel^2;
D_cruise = D - 2 * D_accel;
t_cruise = D_cruise / v_max;

% Total duration
T = 2 * t_accel + t_cruise;
t = 0:dt:T;

% Initialize trajectory vectors
x = zeros(size(t));
vx = zeros(size(t));
ax = zeros(size(t));

for i = 1:length(t)
    ti = t(i);
    if ti <= t_accel
        % Acceleration phase
        a = a_max;
        v = a * ti;
        p = 0.5 * a * ti^2;
    elseif ti <= t_accel + t_cruise
        % Constant velocity phase
        dt_cruise = ti - t_accel;
        a = 0;
        v = v_max;
        p = D_accel + v * dt_cruise;
    else
        % Deceleration phase
        dt_decel = ti - t_accel - t_cruise;
        a = -a_max;
        v = v_max + a * dt_decel;
        p = D_accel + D_cruise + v_max * dt_decel + 0.5 * a * dt_decel^2;
    end

    x(i) = x_start + dir * p;
    vx(i) = dir * v;
    ax(i) = dir * a;
end

traj.t = t;
traj.x = x;
traj.vx = vx;
traj.ax = ax;

% Fixed y-dimension and orientation
traj.y = ones(size(traj.t));
traj.vy = zeros(size(traj.t));
traj.ay = zeros(size(traj.t));
traj.theta = zeros(size(traj.t));
traj.omega = zeros(size(traj.t));

end