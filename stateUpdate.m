function nextState = stateUpdate(currentState, ts, u_curr, params)

tau_w = params.tau_w;
sigma_w = params.sigma_w;

kw_rand = sigma_w * sqrt(2 * ts / tau_w);
% controller input
F_clamped = u_curr(1);
M_clamped = u_curr(2);

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

end