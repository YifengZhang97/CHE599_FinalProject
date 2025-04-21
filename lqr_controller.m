function u = lqr_controller(state, desired_state, K, params)
    % Desired hover state at desired position
    x_des = [desired_state.x;
             desired_state.y;
             0;
             desired_state.vx;
             desired_state.vy;
             0];

    u_lin = -K * (state - x_des);

    F = params.m*params.g + u_lin(1); 
    M = u_lin(2);

    u = [F; M];
end
