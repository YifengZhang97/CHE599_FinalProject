function u = pd_controller(state, desired_state, params)

x_des = desired_state.x;
y_des = desired_state.y;
vx_des = desired_state.vx;
vy_des = desired_state.vy;
% often we do not feedforward ax_des and ay_des
% ax_des = desired_state.ax;
% ay_des = desired_state.ay;

Kp = params.Kp;
Kd = params.Kd;

% linear dynamics with small tilt assumption
F = Kp(2) * (y_des - state(2)) + Kd(2) * (vy_des - state(5));
thetad = - Kp(1) * (x_des - state(1)) - Kd(1) * (vx_des - state(4));
% clamp thetad
theta_threshold = pi/12;
thetad = min(max(thetad, -theta_threshold), theta_threshold);
M = params.I*(Kp(3)*(thetad - state(3)) - Kd(3) * state(6));

u = [F; M];

end