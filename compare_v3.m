%% VERSION 3 FINAL COMPARISON
%% TODO before final report/resume metrics: run zero-state, zero-disturbance sanity check on Version 3 logging.
% Perfect-state LQR vs Kalman-estimated-state LQR

clearvars;

%% Load robot/controller parameters
robot_params;

%% Model names
perfectModel = "self_balancing_robot_lqr_perfect_state";
kalmanModel  = "self_balancing_robot_lqr_kalman";

%% Evaluation settings
theta_tol = deg2rad(0.5);

%% Define tests
testNames = [
    "T1_5deg_initial"
    "T2_0p1N_0p1s"
    "T3_0p5N_0p1s"
    "T4_0p5N_1s"
];

theta0_list = [
    deg2rad(5)
    0
    0
    0
];

distMag_list = [
    0
    0.1
    0.5
    0.5
];

distStart_list = [
    4
    4
    4
    4
];

distEnd_list = [
    4.1
    4.1
    4.1
    5
];

%% Empty final table
finalResults = table();

%% Run all four tests
for i = 1:length(testNames)

    fprintf('\n====================================\n');
    fprintf('Running %s\n', testNames(i));
    fprintf('====================================\n');

    %% Current test conditions
    theta0 = theta0_list(i);

    disturbance_mag   = distMag_list(i);
    disturbance_start = distStart_list(i);
    disturbance_end   = distEnd_list(i);

    % For initial-tilt test, measure from t = 0
    if i == 1
        measurement_start = 0;
        recovery_reference = 0;
    else
        measurement_start = disturbance_start;
        recovery_reference = disturbance_end;
    end

    %% -------------------------------
    % PERFECT-STATE LQR
    % --------------------------------

    outPerfect = sim(perfectModel);

    thetaP = outPerfect.theta_lqr;
    xP     = outPerfect.x_lqr;
    FP     = outPerfect.F_lqr;

    metricsP = calculateMetrics( ...
        thetaP, xP, FP, ...
        measurement_start, ...
        recovery_reference, ...
        theta_tol);

    %% -------------------------------
    % KALMAN-LQR
    % --------------------------------

    outKalman = sim(kalmanModel);

    thetaK = outKalman.theta_lqr;
    xK     = outKalman.x_lqr;
    FK     = outKalman.F_lqr;

    metricsK = calculateMetrics( ...
        thetaK, xK, FK, ...
        measurement_start, ...
        recovery_reference, ...
        theta_tol);

    %% Add rows to final table

    newRows = table( ...
        [testNames(i); testNames(i)], ...
        ["Perfect-State LQR"; "Kalman-LQR"], ...
        [metricsP.RecoveryTime; metricsK.RecoveryTime], ...
        [metricsP.MaxPitchDeg; metricsK.MaxPitchDeg], ...
        [metricsP.MaxX; metricsK.MaxX], ...
        [metricsP.FinalX; metricsK.FinalX], ...
        [metricsP.MaxForce; metricsK.MaxForce], ...
        'VariableNames', { ...
        'Test', ...
        'Controller', ...
        'RecoveryTime_s', ...
        'MaxPitch_deg', ...
        'MaxX_m', ...
        'FinalX_m', ...
        'MaxForce_N'});

    finalResults = [finalResults; newRows];

end

%% Display complete Version 3 table
disp(finalResults)

%% Save results
if ~exist('results','dir')
    mkdir('results');
end

writetable(finalResults, ...
    'results/V3_final_perfect_vs_kalman.csv');

save('results/V3_final_perfect_vs_kalman.mat', ...
    'finalResults');

fprintf('\nVersion 3 results saved.\n');


%% =========================================================
% LOCAL FUNCTION
% ==========================================================

function M = calculateMetrics(theta_ts, x_ts, F_ts, ...
                              measurement_start, ...
                              recovery_reference, ...
                              theta_tol)

    t = theta_ts.Time;

    theta = squeeze(theta_ts.Data);
    x     = squeeze(x_ts.Data);
    F     = squeeze(F_ts.Data);

    %% Maximum pitch after event begins
    idxMeasure = t >= measurement_start;

    maxPitch = max(abs(theta(idxMeasure)));
    maxPitchDeg = rad2deg(maxPitch);

    %% Recovery time
    idxRecovery = find(t >= recovery_reference, 1);

    settleTime = NaN;

    for k = idxRecovery:length(t)
        if all(abs(theta(k:end)) <= theta_tol)
            settleTime = t(k);
            break;
        end
    end

    if isnan(settleTime)
        recoveryTime = NaN;
    else
        recoveryTime = settleTime - recovery_reference;
    end

    %% Maximum horizontal displacement
    maxX = max(abs(x));

    %% Final horizontal position
    idxFinal = t >= (t(end) - 0.5);
    finalX = abs(mean(x(idxFinal)));

    %% Maximum actual actuator force
    maxForce = max(abs(F));

    %% Return metrics
    M.RecoveryTime = recoveryTime;
    M.MaxPitchDeg   = maxPitchDeg;
    M.MaxX          = maxX;
    M.FinalX        = finalX;
    M.MaxForce      = maxForce;

end

if ~exist('results','dir')
    mkdir('results');
end

writetable(finalResults, ...
    'results/V3_control_comparison.csv');

save('results/V3_control_comparison.mat', ...
    'finalResults');

Condition = [
    "Nominal"
    "Disturbance"
    ];

X_RMSE_m = [
    0.001104
    0.001102
    ];

Xdot_RMSE_mps = [
    0.009228
    0.011114
    ];

Theta_RMSE_deg = [
    0.0899
    0.0985
    ];

ThetaDot_RMSE_degps = [
    0.7490
    1.3009
    ];

estimatorResults = table( ...
    Condition, ...
    X_RMSE_m, ...
    Xdot_RMSE_mps, ...
    Theta_RMSE_deg, ...
    ThetaDot_RMSE_degps);

disp(estimatorResults)

writetable(estimatorResults, ...
    'results/V3_kalman_estimator_accuracy.csv');

save('results/V3_kalman_estimator_accuracy.mat', ...
    'estimatorResults');
filterResults = table( ...
    0.2061, ...
    0.0506, ...
    'VariableNames', ...
    {'RawPitch_RMSE_deg','ComplementaryFilter_RMSE_deg'});

writetable(filterResults, ...
    'results/V3_complementary_filter.csv');