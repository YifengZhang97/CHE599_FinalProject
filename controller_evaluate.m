function [e_pos, ctrl_effort] = controller_evaluate(traj, t_out, state_out, state_hat_out, u_out, params)
    % Preallocate
    N = length(t_out);
    e_pos = zeros(N, 1);
    % e_vel = zeros(N, 1);
    % e_theta = zeros(N, 1);
    ctrl_effort = zeros(N, 1);
    
    for k = 1:N
        % current time
        t = t_out(k);
        
        % current state
        xk = state_out(k, :)'; % 7x1 [x; y; theta; vx; vy; omega; extra?]
        x = xk(1); y = xk(2); 
        % theta = xk(3);
        % vx = xk(4); vy = xk(5);
    
        % desired state
        x_des = trajhandle(t, traj, params);
    
        % position error
        e_pos(k) = norm([x - x_des.x; y - x_des.y]);
    
        % velocity error
        % e_vel(k) = norm([vx - x_des.vx; vy - x_des.vy]);
    
        % orientation error (optional if used)
        % e_theta(k) = abs(theta);  % assuming desired theta is 0
    
        % control effort (u = [F; M])
        ctrl_effort(k) = u_out(k,:) * u_out(k,:)';
    end
    
    % --- Summary statistics ---
    RMS_pos_error = sqrt(mean(e_pos.^2));
    % RMS_vel_error = sqrt(mean(e_vel.^2));
    Total_ctrl_effort = sum(ctrl_effort) * (t_out(2) - t_out(1));
    
    fprintf('--- Tracking Performance Summary ---\n');
    fprintf('RMS Position Error: %.4f m\n', RMS_pos_error);
    % fprintf('RMS Velocity Error: %.4f m/s\n', RMS_vel_error);
    fprintf('Total Control Effort: %.4f\n', Total_ctrl_effort);
    
    % --- Plots ---
    figure;
    subplot(2,1,1);
    plot(t_out, e_pos); ylabel('Position Error [m]');
    title('Tracking Error Metrics');
    
    % subplot(3,1,2);
    % plot(t_out, e_vel); ylabel('Velocity Error [m/s]');
    % 
    subplot(2,1,2);
    plot(t_out, ctrl_effort); ylabel('Control Effort'); xlabel('Time [s]');
    

end
