clear all

% Simulation time (ms)
T = 2000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [10; 1];
l = [1; 1];
d = [1; 1];
g0 = 0.0098;
[a, m] = eval_2r_params(l, d, m, g0);
D = 1; % Coefficent of viscous friction

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

min_q = [deg2rad(-180); deg2rad(-60)];
max_q = [deg2rad(+180); deg2rad(+60)];

num_steps = floor(T / dt);

data_mat(1:12,num_steps) = 0.0;

%% Simulation phase 
for i = 1:dt:T
    %% Update Plots
    err = [0.0; 0.0];
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u, qd);
    %% simulate system
    % Compute robot's model in real time
    M = eval_2r_M(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute model's torque
    % Working in nominal conditions -> use nominal M and C, othewrise
    % estimate them
    % ui = M * ai + C;
    N = C * dq + D * dq + G;
    ddq = M \ (u - N);
    dq  = dq + integrate(ddq, dt);
    %Constrain velocities
    dq(1) = max(min(max_dq(1), dq(1)), min_dq(1));
    dq(2) = max(min(max_dq(2), dq(2)), min_dq(2));
    q   = q  + integrate(dq, dt);
    % Constrain joints
    q(1) = max(min(max_q(1), q(1)), min_q(1));
    q(2) = max(min(max_q(2), q(2)), min_q(2));
    % Update error
    err_prec = err;
end
%% Display results
labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);