function nextState = stateUpdate(currentState, ts, u_curr, params)

tau_w = params.tau_w;
sigma_w = params.sigma_w;

% controller input
u1 = u_curr(1);
u2 = u_curr(2);

sdot = [currentState(4);
    currentState(5);
    currentState(6);
    -(u1 + 1) * sin(currentState(3)) + currentState(7);
    (u1 + 1) * cos(currentState(3)) - 1;
    u2 / params.I;
    0];

% update the system dynamics under discretization
nextState = currentState + ts * sdot;

% update the wind error
nextState(7) = (1-ts/tau_w) * nextState(7) + sigma_w * sqrt(ts) * randn;

end