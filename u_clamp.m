function u_curr = u_clamp(u_curr, params)
% rotor force clamping
F = u_curr(1) + 1.0; M = u_curr(2);
F1 = 0.5*(F - M);
F2 = 0.5*(F + M);

F1_clamped = min(max(params.minF, F1), params.maxF);
F2_clamped = min(max(params.minF, F2), params.maxF);

u_curr(1) = F1_clamped + F2_clamped - 1.0;
u_curr(2) = F2_clamped - F1_clamped;

end