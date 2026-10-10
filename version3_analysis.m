% Version 3 analysis
robot_params

theta0 = deg2rad(5);
disturbance_mag = 0.0;
disturbance_start = 4;
disturbance_end   = 5.1;
% out_lqr = sim("self_balancing_robot_lqr_rebuilt");

theta_true = out.theta_lqr;
theta_meas = out.theta_lqr_measured;

theta_error = theta_meas.Data - theta_true.Data;

theta_rmse = sqrt(mean(theta_error.^2));

fprintf('Theta measurement RMSE: %.4f deg\n', ...
    rad2deg(theta_rmse));

theta_true = out.theta_lqr;
theta_meas = out.theta_lqr_measured;
theta_comp = out.theta_hat_comp;

raw_error = theta_meas.Data - theta_true.Data;
comp_error = theta_comp.Data - theta_true.Data;

raw_rmse = sqrt(mean(raw_error.^2));
comp_rmse = sqrt(mean(comp_error.^2));

fprintf('Raw measurement RMSE: %.4f deg\n', rad2deg(raw_rmse));
fprintf('Complementary filter RMSE: %.4f deg\n', rad2deg(comp_rmse));

raw_rmse_deg = rad2deg(raw_rmse);
comp_rmse_deg = rad2deg(comp_rmse);

filter_results = table(raw_rmse_deg, comp_rmse_deg, ...
    'VariableNames', {'Raw_RMSE_deg','Complementary_RMSE_deg'});

if ~exist('results','dir')
    mkdir('results');
end

writetable(filter_results, ...
    'results/V3_complementary_filter_nominal.csv');

%% Version 3 measurement model

C_meas = [1 0 0 0;
    0 0 1 0];

D_meas = zeros(2,1);