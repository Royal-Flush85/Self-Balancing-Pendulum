%% Physical constants
M = 0.5;
I = 0.006;
g = 9.81;

%% =========================================================
%  NOMINAL PARAMETERS -- controller is designed for these
% ==========================================================

m_nom = 0.20;
l_nom = 0.30;

a_nom = M + m_nom;
b_nom = m_nom*l_nom;
c_nom = I + m_nom*l_nom^2;
q_nom = a_nom*c_nom - b_nom^2;

A_nom = [0 1 0 0;
    0 0 -(b_nom*m_nom*g*l_nom)/q_nom 0;
    0 0 0 1;
    0 0 (a_nom*m_nom*g*l_nom)/q_nom 0];

B_nom = [0;
    c_nom/q_nom;
    0;
    -b_nom/q_nom];

%% =========================================================
%  LQR DESIGN -- always based on nominal robot
% ==========================================================

Q_lqr = diag([10 1 100 10]);
R_lqr = 1;

sys_nom = ss(A_nom, B_nom, eye(4), zeros(4,1));

[K_lqr_nominal, S_lqr, poles_lqr] = ...
    lqr(sys_nom, Q_lqr, R_lqr);

% Keep existing Simulink Gain block working
K_lqr = K_lqr_nominal;

%% =========================================================
%  ACTUAL SIMULATED PLANT
% ==========================================================

% For nominal tests:
m = 0.22;
l = 0.33;

% For the later robustness test, change ONLY these two, e.g.
% m = 0.22;
% l = 0.33;

a = M + m;
b = m*l;
c = I + m*l^2;
q = a*c - b^2;

A_ss = [0 1 0 0;
    0 0 -(b*m*g*l)/q 0;
    0 0 0 1;
    0 0 (a*m*g*l)/q 0];

B_ss = [0;
    c/q;
    0;
    -b/q];

C_ss = eye(4);
D_ss = zeros(4,1);

%% Analysis
Co = ctrb(A_ss, B_ss);
Ob = obsv(A_ss, C_ss);

fprintf('Controllability rank: %d\n', rank(Co));
fprintf('Observability rank: %d\n', rank(Ob));

disp('Nominal LQR gain:')
disp(K_lqr_nominal)

disp('Nominal LQR closed-loop poles:')
disp(poles_lqr)


%% Simulated sensor parameters
Ts_sensor = 0.01; %100 Hz sensor sampling

% example sensor
sigma_x = 0.002; % 2 mm position noise
sigma_theta = deg2rad(0.2); % 0.2 degree angle noise

%% Simulated IMU parameters, Gyro, wheel mechanic
% gyro sensor, measures angular velocity
Ts_sensor = 0.01;                 % 100 Hz

sigma_theta = deg2rad(0.2);       % angle-measurement noise
sigma_gyro  = deg2rad(0.5);       % gyro noise, rad/s
gyro_bias   = deg2rad(0.1);       % constant gyro bias, rad/s


% Complementary filter
alpha_comp = 0.98;

%% =========================================================
%  VERSION 3D -- DISCRETE KALMAN STATE ESTIMATOR
% ==========================================================

Ts_sensor = 0.01;   % 100 Hz

% Convert continuous robot model to discrete time
sys_d = c2d(ss(A_ss, B_ss, eye(4), zeros(4,1)), ...
    Ts_sensor);

Ad = sys_d.A;
Bd = sys_d.B;

% We measure only:
% y = [x_measured; theta_measured]

C_meas = [1 0 0 0;
    0 0 1 0];

% Assumed process uncertainty
Q_kf = diag([1e-6 1e-4 1e-6 1e-4]);

% Measurement noise covariance
R_kf = diag([sigma_x^2 sigma_theta^2]);

% Steady-state Kalman estimator gain
[L_kf, P_kf, poles_kf] = ...
    dlqe(Ad, eye(4), C_meas, Q_kf, R_kf);

disp('Kalman estimator gain:')
disp(L_kf)

disp('Kalman estimator poles:')
disp(poles_kf)
