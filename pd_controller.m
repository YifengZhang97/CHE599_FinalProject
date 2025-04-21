function u = pd_controller(state, desired_state, params)

    x_des = desired_state.x;
    y_des = desired_state.y;
    vx_des = desired_state.vx;
    vy_des = desired_state.vy;
    ax_des = desired_state.ax;
    ay_des = desired_state.ay;

    Kp = params.Kp;
    Kd = params.Kd;

    m = params.m;
    g = params.g;
    I = params.I;
    
    
    if params.normalize
        thetad = -(params.A/g)*(ax_des + Kp(1)*(x_des-state(1)) + Kd(1)*(vx_des-state(4)));
        F = 1 + (params.A/g)*(ay_des + Kp(2)*(y_des-state(2)) + Kd(2)*(vy_des-state(5)));
        M = params.Ibar*(Kp(3)*(thetad-state(3)) + Kd(3)*(-state(6)));
    else
        % linear dynamics with small tilt assumption
        thetad = -(1/g)*(ax_des + Kp(1)*(x_des-state(1)) + Kd(1)*(vx_des-state(4)));
        F = m*g + m*(ay_des + Kp(2)*(y_des-state(2)) + Kd(2)*(vy_des-state(5)));
        M = I*(Kp(3)*(thetad-state(3)) + Kd(3)*(-state(6)));
    end

    u = [F; M];

end