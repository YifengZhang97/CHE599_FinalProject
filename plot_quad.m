function plot_quad(traj, t_sol, s_sol, u_sol)

% hold on; grid on; fontsize(16,"points")
% scatter(traj.x,traj.y,'.')
% ylabel('y-pos [m]')
% xlabel('x-pos [m]')
% title('Figure 8 Positions')
% % legend({'y-vel', 'x-vel'})

figure; hold on; grid on;fontsize(16,"points")
scatter(s_sol(:,1), s_sol(:,2), '.')
scatter(traj.x, traj.y,'.')
xlabel('x-pos [m]')
ylabel('y-pos [m]')

figure; hold on; grid on;fontsize(16,"points")
scatter(t_sol, s_sol(:,1), '.')
scatter(t_sol, s_sol(:,2), '.')
scatter(traj.t,traj.x,'.')
scatter(traj.t,traj.y,'.')
xlabel('time [s]')
ylabel('position [m]')
legend({'x','y', 'desired x', 'desired y'})
title('positions over time')

figure; hold on; grid on;fontsize(16,"points")
scatter(t_sol, s_sol(:,4), '.')
scatter(t_sol, s_sol(:,5), '.')
scatter(traj.t,traj.vx,'.')
scatter(traj.t,traj.vy,'.')
xlabel('time [s]')
ylabel('velocity [m/s]')
legend({'vx','vy','desired vx','desired vy'})
title('velocity over time')

figure; hold on; grid on;fontsize(16,"points")
scatter(t_sol, u_sol(:,1), '.')
xlabel('time [s]')
ylabel('force [N]')
title('thrust force over time')

figure; hold on; grid on;fontsize(16,"points")
scatter(t_sol, u_sol(:,2), '.')
xlabel('time [s]')
ylabel('moment [Nm]')
title('moment over time')



end