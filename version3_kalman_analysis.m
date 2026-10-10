xtrue_ts = out.xtrue;
xhat_ts  = out.state_hat_kf;

%% Kalman estimator accuracy

t_hat = xhat_ts.Time;
xhat  = squeeze(xhat_ts.Data);

% True robot is continuous, Kalman estimator is discrete,
% so evaluate the true state at the Kalman timestamps
xtrue_interp = interp1( ...
    xtrue_ts.Time, ...
    squeeze(xtrue_ts.Data), ...
    t_hat, ...
    'linear');

error = xhat - xtrue_interp;

rmse = sqrt(mean(error.^2,1));

fprintf('\nKalman Estimator RMSE\n');
fprintf('x:         %.6f m\n', rmse(1));
fprintf('x_dot:     %.6f m/s\n', rmse(2));
fprintf('theta:     %.4f deg\n', rad2deg(rmse(3)));
fprintf('theta_dot: %.4f deg/s\n', rad2deg(rmse(4)));