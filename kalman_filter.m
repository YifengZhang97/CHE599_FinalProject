function [state_hat, Pk] = kalman_filter(state_hat, u_curr, xytheta_obs, Pk, ts, params)
% (1) Prediction
u1 = u_curr(1);
theta_hat = state_hat(3);
% calculate the Jacobian
F = eye(7);
F(1, 4) = ts;
F(2, 5) = ts;
F(3, 6) = ts;
F(4, 7) = ts;
F(4, 3) = -(u1 + 1) * cos(theta_hat) * ts;
F(5, 3) = -(u1 + 1) * sin(theta_hat) * ts;
F(7, 7) = 1 - ts/params.tau_w;
% the model uncertainty
Q = zeros(7, 7);
Q(7, 7) = params.sigma_w^2 * ts;
% update the state_hat
state_hat = stateUpdate(state_hat, ts, u_curr, params);
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