%% PID vs LQR comparison
robot_params
test_name = 'T5_parameter_mismatch';

initial_tilt = deg2rad(5);

disturbance_mag   = 0.5;
disturbance_start = 4;
disturbance_end   = 4.1;

measurement_start = disturbance_start;
recovery_reference_time = disturbance_end;

%% Run both models

out_pid = sim("self_balancing_robot_pid_rebuilt");
out_lqr = sim("self_balancing_robot_lqr_rebuilt");

theta_pid = out_pid.theta_pid;
x_pid     = out_pid.x_pid;
F_pid     = out_pid.F_pid;

theta_lqr = out_lqr.theta_lqr;
x_lqr     = out_lqr.x_lqr;
F_lqr     = out_lqr.F_lqr;



theta_tol = deg2rad(0.5);

%% ----- PID -----

t_pid     = theta_pid.Time;
theta_p   = theta_pid.Data;
x_p       = x_pid.Data;
F_p       = F_pid.Data;

% Maximum pitch after disturbance begins
idx = t_pid >= disturbance_start;
max_theta_pid = max(abs(theta_p(idx)));
max_theta_pid_deg = rad2deg(max_theta_pid);

% Recovery time
idx_after = find(t_pid >= disturbance_end);
settle_pid = NaN;

for k = idx_after(1):length(t_pid)
    if all(abs(theta_p(k:end)) <= theta_tol)
        settle_pid = t_pid(k);
        break
    end
end

recovery_pid = settle_pid - disturbance_end;

% Maximum horizontal displacement
max_x_pid = max(abs(x_p));

% Final horizontal displacement
idx_final = t_pid >= (t_pid(end) - 0.5);
final_x_pid = abs(mean(x_p(idx_final)));

% Maximum actual actuator force
max_F_pid = max(abs(F_p));


%% ----- LQR -----
%% LQR design

Q_lqr = diag([10 1 100 10]);
R_lqr = 1;

sys_ss = ss(A_ss, B_ss, eye(4), zeros(4,1));

[K_lqr, S_lqr, poles_lqr] = lqr(sys_ss, Q_lqr, R_lqr);
t_lqr     = theta_lqr.Time;
theta_l   = theta_lqr.Data;
x_l       = x_lqr.Data;
F_l       = F_lqr.Data;

% Maximum pitch after disturbance begins
idx = t_lqr >= disturbance_start;
max_theta_lqr = max(abs(theta_l(idx)));
max_theta_lqr_deg = rad2deg(max_theta_lqr);

% Recovery time
idx_after = find(t_lqr >= disturbance_end);
settle_lqr = NaN;

for k = idx_after(1):length(t_lqr)
    if all(abs(theta_l(k:end)) <= theta_tol)
        settle_lqr = t_lqr(k);
        break
    end
end

recovery_lqr = settle_lqr - disturbance_end;

% Maximum horizontal displacement
max_x_lqr = max(abs(x_l));

% Final horizontal displacement
idx_final = t_lqr >= (t_lqr(end) - 0.5);
final_x_lqr = abs(mean(x_l(idx_final)));

% Maximum actual actuator force
max_F_lqr = max(abs(F_l));


%% ----- Display comparison -----

results = table( ...
    [recovery_pid; recovery_lqr], ...
    [max_theta_pid_deg; max_theta_lqr_deg], ...
    [max_x_pid; max_x_lqr], ...
    [final_x_pid; final_x_lqr], ...
    [max_F_pid; max_F_lqr], ...
    'VariableNames', { ...
    'RecoveryTime_s', ...
    'MaxPitch_deg', ...
    'MaxX_m', ...
    'FinalX_m', ...
    'MaxForce_N'}, ...
    'RowNames', {'PID','LQR'});

if ~exist('results','dir')
    mkdir('results');
end

filename = fullfile('results', [test_name '.csv']);

writetable(results, filename, 'WriteRowNames', true);

fprintf('Saved results to %s\n', filename);