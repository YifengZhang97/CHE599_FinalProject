function traj = traj_figure8(A, B, T, dt, a_max, s0)
% Generates a figure-8 trajectory with acceleration capped at a_max
% Inputs:
%   A     = half-width
%   B     = half-height
%   T     = total time
%   dt    = timestep
%   a_max = max acceleration allowed

% Compute maximum allowable omega
omega_limit = sqrt(min(a_max / A, a_max / (4*B)));

% Choose omega to complete N cycles in time T
% Example: 1 cycle in T => omega = 2*pi / T
omega_nominal = 2 * pi / T;

% Choose the minimum of both
omega = min(omega_limit, omega_nominal);

% Time vector
t = 0:dt:T;

% Position
x = A * sin(omega * t);
y = B * sin(2 * omega * t);

% Velocity
vx = A * omega * cos(omega * t);
vy = 2 * B * omega * cos(2 * omega * t);

% Acceleration
ax = -A * omega^2 * sin(omega * t);
ay = -4 * B * omega^2 * sin(2 * omega * t);

% Fill trajectory struct
traj.t = t;
traj.x = x + s0(1);
traj.y = y + s0(2);
traj.vx = vx;
traj.vy = vy;
traj.ax = ax;
traj.ay = ay;
traj.theta = zeros(size(t));
traj.omega = zeros(size(t));
end
