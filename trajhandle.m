function desired_state = trajhandle(t, traj, params)
% This function interpolates the next desired state based on the current
% timestep and a fully defined trajectory, where traj is in the form:
% traj.t      = [t0, t1, ..., tn];           % time vector
% traj.x      = [x0, x1, ..., xn];           % x positions
% traj.y      = [y0, y1, ..., yn];           % y positions
% traj.theta  = [...];                       % orientation (optional)
% traj.vx     = [...];                       % x velocity
% traj.vy     = [...];
% traj.vtheta  = [...];                      % angular velocity
% traj.ax     = [...];                       % optional acceleration
% traj.ay     = [...];

    x = interp1(traj.t, traj.x, t, 'linear', 'extrap');
    y = interp1(traj.t, traj.y, t, 'linear', 'extrap');
    vx = interp1(traj.t, traj.vx, t, 'linear', 'extrap');
    vy = interp1(traj.t, traj.vy, t, 'linear', 'extrap');
    ax = interp1(traj.t, traj.ax, t, 'linear', 'extrap');
    ay = interp1(traj.t, traj.ay, t, 'linear', 'extrap');
    
    desired_state.x = x;
    desired_state.y = y;
    desired_state.vx = vx;
    desired_state.vy = vy;
    desired_state.ax = ax;
    desired_state.ay = ay;
    
end