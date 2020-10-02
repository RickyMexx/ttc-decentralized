%% Experiment Tracking with Nominal parameters
clc
% USED CONTROLLER
USED_CTRL = 'pd';
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
qd  = [0; 0]; % Final joint state
dqd = [0; 0]; % Final joint velo city
ddqd= [0; 0]; % Final joint acceleration
% Robot variables
q  = qi;   % Joint state (variable)
dq = dqi;  % Joint velocity
ddq= ddqi; % Joint Acceleration
u  = ui;   % Torque input
%% 2R Parameters
m = [20; 20];
l = [10; 10];
d = [5; 5];
g0 = 0.00981;

N = diag([10, 10]); % Reduction Ratio
D = diag([1., 1.]); % Viscous Friction Matrix
J = diag([1., 1.]); % Inertia Matrix

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

nr = 0.0; % Nominal-Real additive factor for real parameters
[a, m] = eval_2r_params(l, d, m, g0);
[ar, mr] = eval_2r_params_real(l, d, m, g0, nr);
Dr = D - diag([nr / 5, -nr / 5]); % Real Viscous Friction Matrix
%% PD-Controller Parameters
% FBL good parameters
%kp = 0.05  * eye(2);
%kd = 1.0 * eye(2);
% PD good parameters
kp = 50  * eye(2);
kd = 40 * eye(2);
err= qd - q;
err_prec = err;
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
    % Tracking qd update
    qd = [sin(i / 100); cos(i / 50)];
    % Update control term
    err = double(qd - q);
    switch USED_CTRL
        case 'fbl'
            u = controller_fbl(q, dq, ddq, ar, D, N, kp, kd, err, err_prec);
        case 'pp'
            u = controller_pp(q, dq, ddq, ar, D, N, qd);
        case 'pd'
            u = controller_pd(q, dq, ddq, ar, D, N, kp, kd, err, err_prec);
    end    
    % Clamp u
    u(1) = max(min(uconstr(1, 2), u(1)), uconstr(1, 1));
    u(2) = max(min(uconstr(2, 2), u(2)), uconstr(2, 1));
    % Step the model
    [q, dq, ddq] = step_2r_model(dt, q, dq, u, a, Dr, qconstr, dqconstr);
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
writetable(T, "data/exp_track_nom_" + USED_CTRL + ".csv");
saveas(gcf, "images/exp_track_nom_" + USED_CTRL + ".fig");


