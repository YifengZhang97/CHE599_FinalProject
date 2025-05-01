function traj = traj_spline(num_turns, total_length, dt)
% Generates a figure-8 trajectory with acceleration capped at a_max
% Inputs:

x_waypoints = linspace(0, total_length, num_turns + 2);  % N+2 points
y_range = 1.0;  % magnitude of zigzag variation
y_waypoints = y_range * (2 * rand(1, num_turns + 2) - 1);  % random in [-1, 1]

waypoints = [x_waypoints(:), y_waypoints(:)];

avg_v = 1.0;  % desired average velocity (dimensionless or m/s)
segment_dists = vecnorm(diff(waypoints), 2, 2);
segment_times = segment_dists / avg_v;
t_total = sum(segment_times);
t = 0:dt:t_total;

cumulative_time = [0; cumsum(segment_times)];
x = waypoints(:,1);
y = waypoints(:,2);

ppx = spline(cumulative_time, x);
ppy = spline(cumulative_time, y);

x_t = ppval(ppx, t);
y_t = ppval(ppy, t);

vx_t = ppval(fnder(ppx,1), t);
vy_t = ppval(fnder(ppy,1), t);

ax_t = ppval(fnder(ppx,2), t);
ay_t = ppval(fnder(ppy,2), t);

theta_t = atan2(-ax_t, ay_t + 1);  % assuming g = 1 for normalized
omega_t = gradient(theta_t, dt);


traj.t = t;
traj.x = x_t;
traj.y = y_t;
traj.vx = vx_t;
traj.vy = vy_t;
traj.ax = ax_t;
traj.ay = ay_t;
traj.theta = theta_t;
traj.omega = omega_t;

% traj_dimless.t     = traj.t     / units.T;
% traj_dimless.x     = traj.x     / units.L;
% traj_dimless.y     = traj.y     / units.L;
% traj_dimless.vx    = traj.vx    / units.V;
% traj_dimless.vy    = traj.vy    / units.V;
% traj_dimless.theta = traj.theta;                 % radians — no scaling
% traj_dimless.omega = traj.omega * units.T;       % rad/s → dimensionless

end
