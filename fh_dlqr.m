function Kk = fh_dlqr(Ad, Bd, Q, R, Qf, N)
    % Pre-allocate
    P = Qf;
    Kk = cell(N,1);

    for k = N:-1:1
        K = (R + Bd' * P * Bd) \ (Bd' * P * Ad);
        Kk{k} = K;
        P = Q + Ad' * P * (Ad - Bd * K);
    end
end
