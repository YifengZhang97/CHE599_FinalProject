function u_curr = u_clamp(u_curr, params)
% rotor force clamping
F = u_curr(1); M = u_curr(2);
F1 = 0.5*(F - M/params.l);
F2 = 0.5*(F + M/params.l);

F1_clamped = min(max(params.minF/2, F1), params.maxF/2);
F2_clamped = min(max(params.minF/2, F2), params.maxF/2);

u_curr(1) = F1_clamped + F2_clamped;
u_curr(2) = (F2_clamped - F1_clamped)*params.l;

end