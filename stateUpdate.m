function [nextState, u_out] = stateUpdate(currentState, ts, u, params)

tau_w = params.tau_w;
sigma_w = params.sigma_w;

kw_rand = sigma_w * sqrt(2 * ts / tau_w);

% rotor force clamping
F = u(1); M = u(2);
u1 = 0.5*(F - M/params.l);
u2 = 0.5*(F + M/params.l);

u1_clamped = min(max(params.minF/2, u1), params.maxF/2);
u2_clamped = min(max(params.minF/2, u2), params.maxF/2);

F_clamped = u1_clamped + u2_clamped;
M_clamped = (u2_clamped - u1_clamped)*params.l;


if params.normalize
    sdot = [currentState(4);
        currentState(5);
        currentState(6);
        -F_clamped * sin(currentState(3)) + currentState(7);
        F_clamped * cos(currentState(3)) - 1;
        M_clamped / params.Ibar;
        0];
else
    sdot = [currentState(4);
        currentState(5);
        currentState(6);
        (-F_clamped * sin(currentState(3)) + currentState(7)) / params.m;
        F_clamped * cos(currentState(3))/params.m - params.g;
        M_clamped / params.I;
        0];
end

% update the system dynamics under discretization
nextState = currentState + ts * sdot;

% update the wind error
nextState(7) = (1-ts/tau_w) * nextState(7) + kw_rand * randn;
% nextState(7) = 0;

u_out = [F_clamped; M_clamped];

end