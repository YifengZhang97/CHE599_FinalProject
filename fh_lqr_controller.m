function u = fh_lqr_controller(state, desired_state, params)
    Kk = params.Kk;

    % Desired hover state at desired position
    x_des = [desired_state.x;
             desired_state.y;
             0;
             desired_state.vx;
             desired_state.vy;
             0;
             0];

    u = -Kk{params.k} * (state - x_des);
end
