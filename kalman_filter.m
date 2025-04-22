function [state_hat, Pk] = kalman_filter(state_hat, u_curr, xytheta_obs, Pk, ts, params)
% (1) Prediction
state_hat = stateUpdate(state_hat, ts, u_curr, params);
F = eye(7);
F(1, 4) = ts;
F(2, 5) = ts;
F(3, 6) = ts;
F(4, 7) = ts / params.m;
F(7, 7) = 1 - ts/params.tau_w;
Q = zeros(7, 7);
Q(7, 7) = params.sigma_w^2 * 2 * ts / params.tau_w;
% update the variance
Pk = F * Pk * F.' + Q;
% (2) now observe
H = [eye(3), zeros(3,4)];
R = diag([params.sigma_x^2, params.sigma_y^2, params.sigma_theta^2]);
K = Pk * H.' / (H * Pk * H.' + R);
state_hat = state_hat + K * (xytheta_obs - H * state_hat);
% update Pk
Pk = (eye(7) - K * H) * Pk;

end