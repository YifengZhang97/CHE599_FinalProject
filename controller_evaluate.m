function [e_pos, ctrl_effort] = controller_evaluate(traj, t_out, state_out, u_out, params)
state_d = trajhandle(t_out, traj, params);
ex = state_d(1, :).' - state_out(:,1);
ey = state_d(2, :).' - state_out(:,2);
e2 = ex.^2 + ey.^2;
e2 = e2.';
e_pos = sqrt(e2(1:end-1) * diff(t_out)) / (t_out(end) - t_out(1));
u = u_out(1:end-1, :).';
u2 = u.^2;
ctrl_effort = sqrt(u2 * diff(t_out)) / (t_out(end) - t_out(1));
end