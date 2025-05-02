function plot_quad(traj, t_sol, s_sol, u_sol)

% figure; hold on; grid on; fontsize(16,"points")
% plot(traj.x,traj.y)
% ylabel('y-pos [m]')
% xlabel('x-pos [m]')
% title('Figure 8 Positions')
% % legend({'y-vel', 'x-vel'})

figure; hold on; grid on;fontsize(16,"points")
plot(s_sol(:,1), s_sol(:,2), 'DisplayName', 'actual trajectory')
plot(traj.x, traj.y, 'DisplayName', 'desired trajectory')
axis equal
legend
xlabel('x-pos [m]')
ylabel('y-pos [m]')

figure; hold on; grid on;fontsize(16,"points")
plot(t_sol, s_sol(:,1))
plot(t_sol, s_sol(:,2))
plot(traj.t,traj.x)
plot(traj.t,traj.y)
xlabel('time [s]')
ylabel('position [m]')
legend({'x','y', 'desired x', 'desired y'})
title('positions over time')

figure; hold on; grid on;fontsize(16,"points")
plot(t_sol, s_sol(:,4))
plot(t_sol, s_sol(:,5))
plot(traj.t,traj.vx)
plot(traj.t,traj.vy)
xlabel('time [s]')
ylabel('velocity [m/s]')
legend({'vx','vy','desired vx','desired vy'})
title('velocity over time')

figure;
subplot(2, 1, 1)
hold on; grid on;fontsize(16,"points")
plot(t_sol, u_sol(:,1))
xlabel('time [s]')
ylabel('force [N]')
title('thrust force over time')

subplot(2, 1, 2)
hold on; grid on;fontsize(16,"points")
plot(t_sol, u_sol(:,2))
xlabel('time [s]')
ylabel('moment [Nm]')
title('moment over time')



end