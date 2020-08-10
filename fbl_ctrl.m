clear all;
% FeedBack Linearization Control
% 2R robot w/ gravity

% Simulation time (ms)
T = 200;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [1; 1];
l = [2; 2];
d = [1; 1];
g0 = 9.8;
[a, m] = eval_2r_params(l, d, m, g0);

% Initial conditions
qi  = [0; 0]; % Initial joint state
dqi = [0; 0]; % Initial joint velocity
ddqi= [0; 0]; % Initial joint acceleration
ui  = [0; 0]; % Initial torque input
% Final conditions
qd  = [pi/2; pi/2]; % Final joint state
dqd = [0; 0]; % Final joint velocity
ddqd= [0; 0]; % Final joint acceleration

% Robot variables
q  = qi;   % Joint state (variable)
dq = dqi;  % Joint velocity
ddq= ddqi; % Joint Acceleration
u  = ui;   % Torque input

% Joint bounds
min_dq = [deg2rad(-400); deg2rad(-400)];
max_dq = [deg2rad(400); deg2rad(400)];

%% PD-Controller Parameters
kp = 0.01 * eye(2);
kd = 0.1  * eye(2);
err= qd - q;
err_prec = err;

num_steps = floor(T / dt);
data_mat(1:10,num_steps) = 0.0;

%% Simulation phase 
for i = 1:dt:T
    %% Update Plots
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u);
    %% simulate system
    err = double(qd - q);
    % Controller term
    ai = kd * (err - err_prec) * idt + kp * err;
    % Compute robot's model in real time
    M = eval_2r_M(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute model's torque
    % Working in nominal conditions -> use nominal M and C, othewrise
    % estimate them
    
    % ui = M * ai + C; 
    u = M * ai;
    ddq = double(M \ u);
    dq  = dq + integrate(ddq, dt);
    q   = q  + integrate(dq, dt);
    % Update error
    err_prec = err;
end

%% Display results
%data_plot(data_mat, [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]], 2, 2, ["q1"; "q2"; "dq1"; "dq2"]);
labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);