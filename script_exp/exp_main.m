%% Experiment Regularization with Nominal parameters
clc
addpath('lib_2r', 'lib_ctrl', 'lib_utils', 'script_exp');
% USED CONTROLLER
USED_CTRL = 'lqr';
% EXPERIMENT TYPE
EXP_TYPE = 'reg';
EXP_COND = 'nom';
EXTRA_NAME = '_2';
% Simulation time (ms)
T = 2000;
% Simulation step (ms)
dt  = 1;
idt = 1;
qi  = [0; 0]; % Initial joint state
dqi = [0; 0]; % Initial joint velocity
ddqi= [0; 0]; % Initial joint acceleration
ui  = [0; 0]; % Initial torque input
% Final conditions
qd  = [pi/2; -pi/2]; % Final joint state
dqd = [0; 0]; % Final joint velo city
ddqd= [0; 0]; % Final joint acceleration
% Robot variables
q  = qi;   % Joint state (variable)
dq = dqi;  % Joint velocity
ddq= ddqi; % Joint Acceleration
u  = ui;   % Torque input
%% 2R Parameters
m = [10; 10];
l = [1; 1];
d = [0.5; 0.5];
g0 = 0.00981;

N = diag([4, 4]); % Reduction Ratio
D = diag([1., 1.]); % Viscous Friction Matrix

ddqconstr = [
    -5.0, 5.0;
    -5.0, 5.0
    ];

dqconstr = [
    -5.01, 5.01;
    -5.01, 5.01
    ];
qconstr  = [
    deg2rad(-360), deg2rad(360);
    deg2rad(-360), deg2rad(360)
    ];

uconstr = [
    -5.0, 5.0;
    -5.0, 5.0
    ];

if strcmp(EXP_COND, "nom")
    nr = 0.0; % Nominal-Real additive factor for real parameters
else
    nr = 0.5;
end
[a, m] = eval_2r_params(l, d, m, g0);
[ar, mr] = eval_2r_params_real(l, d, m, g0, nr);
Dr = D - diag([nr / 5, -nr / 5]); % Real Viscous Friction Matrix
%% PD-Controller Parameters
% FBL parameters
kp_fbl = 0.05  * eye(2);
kd_fbl = 1.0 * eye(2);
% PD parameters
kp_pd = 1.  * eye(2);
kd_pd = 3 * eye(2);
err= qd - q;
err_prec = err;
%% PP-Controller Parameters
P_pp = [-1.3, -1.5, -1.1, -1.8];
P_pp_er = [-.5, -.4, -.2, -.6];
%% PP-SlidingMode Parameters
gs_sm = diag([1 1]); % Weight of error for the sliding surface
phi_sm = [1 1]; % Boundary layers
P_sm = [-.6, -.3, -.5, -.4]; 
Ks_sm = diag([0.8, 0.3]);
%% LQR-Controller Parameters
Q = diag([1.2, 1.2, 1.5, 1.5]); % dq1 dq2 ddq1 ddq2 
R = diag([1, 1]); % u1 u2
%% Simulation Phase
num_steps = floor(T / dt);
% Insert all elements that must be stored
data_q(1:2, num_steps) = 0.0;
data_dq(1:2, num_steps) = 0.0;
data_ddq(1:2, num_steps) = 0.0;
data_qd(1:2, num_steps) = 0.0;
data_err(1:2, num_steps) = 0.0;
data_u(1:2, num_steps) = 0.0;

data_mat(1:12, num_steps) = 0.0;

for i = 1:dt:T
    % Store experiment data
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u, qd);
    data_q(1:2, i) = q;
    data_dq(1:2, i) = dq;
    data_ddq(1:2, i) = ddq;
    data_qd(1:2, i) = qd;
    data_err(1:2, i) = err;
    data_u(1:2, i) = u;
    if strcmp(EXP_TYPE, 'track')
        % Tracking qd update
        dqd = [1e-2 * cos(i / 100); -.5e-2 * sin(i / 50)];
        qd = qd + integrate(dqd, dt);
    end
    %qd = [sin(i / 100); cos(i / 50)];
    % Update control term
    err = double(qd - q);
    derr = (err - err_prec) * idt;
    switch USED_CTRL
        case 'fbl'
            u = controller_fbl(q, dq, ddq, ar, D, N, kp_fbl, kd_fbl, err, err_prec);
        case 'pp'
            u = controller_pp(q, dq, ddq, ar, D, N, P_pp, qd, dqd);
        case 'pp_er'
            u = controller_pp_er(q, dq, ddq, ar, D, N, P_pp_er, qd, dqd);
        case 'pd'
            u = controller_pd(q, dq, ddq, ar, D, N, kp_pd, kd_pd, err, err_prec);
        case 'lqr'
            u = controller_lqr(q, dq, ddq, ar, D, N, Q, R, qd, dqd);
        case 'pp_sm'
            u = controller_pp_sm(q, dq, ddq, a, D, N, gs_sm, phi_sm, Ks_sm, P_sm, qd, dqd, err, derr);
        case 'free'
            u = [0;0];
    end    
    % Clamp u
    u(1) = max(min(uconstr(1, 2), u(1)), uconstr(1, 1));
    u(2) = max(min(uconstr(2, 2), u(2)), uconstr(2, 1));
    % Step the model
    [q, dq, ddq] = step_2r_model(dt, q, dq, u, a, Dr, qconstr, dqconstr, ddqconstr);
    err_prec = err;    
end

t_q = array2table(data_q', 'VariableNames', {'q1';'q2'});
t_dq = array2table(data_dq', 'VariableNames', {'dq1';'dq2'});
t_ddq = array2table(data_ddq', 'VariableNames', {'ddq1';'ddq2'});
t_qd = array2table(data_qd', 'VariableNames', {'qd1';'qd2'});
t_err = array2table(data_err', 'VariableNames', {'e1';'e2'});
t_u   = array2table(data_u', 'VariableNames', {'u1';'u2'});

T = [t_q, t_dq, t_ddq, t_qd, t_err, t_u];

labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);

%% Store experimental data and images
writetable(T, "data/exp_" + EXP_TYPE + "_" + EXP_COND + "_" + USED_CTRL + EXTRA_NAME + ".csv");
saveas(gcf, "images/exp_" + EXP_TYPE + "_" + EXP_COND + "_" + USED_CTRL + EXTRA_NAME + ".fig");